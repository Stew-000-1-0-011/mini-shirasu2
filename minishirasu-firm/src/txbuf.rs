//! 送信ストリームごとのリングバッファ

/// 1ストリームぶんのバッファ長
const CAPACITY: usize = 64;

struct Ring {
    buf: [u8; CAPACITY],
    head: usize,
    len: usize,
}

impl Ring {
    const fn new() -> Ring {
        Ring { buf: [0; CAPACITY], head: 0, len: 0 }
    }

    /// 全部入るときだけ入れる
    fn push_all(&mut self, data: &[u8]) -> bool {
        if data.len() > CAPACITY - self.len {
            return false;
        }
        for &b in data {
            self.buf[(self.head + self.len) % CAPACITY] = b;
            self.len += 1;
        }
        true
    }

    fn pop(&mut self, out: &mut [u8]) -> usize {
        let n = out.len().min(self.len);
        for o in out[..n].iter_mut() {
            *o = self.buf[self.head];
            self.head = (self.head + 1) % CAPACITY;
            self.len -= 1;
        }
        n
    }
}

#[derive(Clone, Copy, PartialEq, Debug)]
pub enum TxStream {
    Status,
    Response,
}

pub struct TxQueues {
    status: Ring,
    response: Ring,
}

impl TxQueues {
    pub const fn new() -> TxQueues {
        TxQueues { status: Ring::new(), response: Ring::new() }
    }

    /// 符号化済みのメッセージを積む。入りきらなければ何も積まずfalseを返す
    /// (途中で切れたメッセージをストリームに流さないため)
    pub fn push(&mut self, stream: TxStream, message: &[u8]) -> bool {
        match stream {
            TxStream::Status => self.status.push_all(message),
            TxStream::Response => self.response.push_all(message),
        }
    }

    /// 次に送る最大8バイトを取り出す。両方に溜まっていれば、status_firstの側を先にする
    pub fn pop_chunk(&mut self, status_first: bool, out: &mut [u8; 8]) -> Option<(TxStream, usize)> {
        let order = if status_first {
            [TxStream::Status, TxStream::Response]
        } else {
            [TxStream::Response, TxStream::Status]
        };
        for stream in order {
            let ring = match stream {
                TxStream::Status => &mut self.status,
                TxStream::Response => &mut self.response,
            };
            let n = ring.pop(out);
            if n > 0 {
                return Some((stream, n));
            }
        }
        None
    }
}


#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn pops_bytes_in_order_in_chunks_of_eight() {
        let mut q = TxQueues::new();
        let message: Vec<u8> = (1..=20).collect();
        assert!(q.push(TxStream::Status, &message));

        let mut out = [0u8; 8];
        assert_eq!(q.pop_chunk(true, &mut out), Some((TxStream::Status, 8)));
        assert_eq!(out, [1, 2, 3, 4, 5, 6, 7, 8]);
        assert_eq!(q.pop_chunk(true, &mut out), Some((TxStream::Status, 8)));
        assert_eq!(out, [9, 10, 11, 12, 13, 14, 15, 16]);
        assert_eq!(q.pop_chunk(true, &mut out), Some((TxStream::Status, 4)));
        assert_eq!(out[..4], [17, 18, 19, 20]);
        assert_eq!(q.pop_chunk(true, &mut out), None);
    }

    #[test]
    fn preferred_stream_goes_first() {
        let mut q = TxQueues::new();
        assert!(q.push(TxStream::Response, &[0xA0]));
        assert!(q.push(TxStream::Status, &[0xB0]));

        let mut out = [0u8; 8];
        assert_eq!(q.pop_chunk(true, &mut out), Some((TxStream::Status, 1)));
        assert_eq!(q.pop_chunk(true, &mut out), Some((TxStream::Response, 1)));

        assert!(q.push(TxStream::Response, &[0xA0]));
        assert!(q.push(TxStream::Status, &[0xB0]));
        assert_eq!(q.pop_chunk(false, &mut out), Some((TxStream::Response, 1)));
        assert_eq!(q.pop_chunk(false, &mut out), Some((TxStream::Status, 1)));
    }

    #[test]
    fn message_that_does_not_fit_is_dropped_whole() {
        let mut q = TxQueues::new();
        assert!(q.push(TxStream::Status, &[1u8; 60]));
        // 残り4バイトに24バイトは入らない。途中まで入れたりしない
        assert!(!q.push(TxStream::Status, &[2u8; 24]));
        assert!(q.push(TxStream::Status, &[3u8; 4]));

        let mut out = [0u8; 8];
        let mut all = Vec::new();
        while let Some((_, n)) = q.pop_chunk(true, &mut out) {
            all.extend_from_slice(&out[..n]);
        }
        assert_eq!(all.len(), 64);
        assert!(all.iter().all(|&b| b != 2));
    }

    #[test]
    fn streams_do_not_share_capacity() {
        let mut q = TxQueues::new();
        assert!(q.push(TxStream::Status, &[1u8; 64]));
        assert!(q.push(TxStream::Response, &[2u8; 64]));
        assert!(!q.push(TxStream::Response, &[2u8; 1]));
    }

    #[test]
    fn buffer_wraps_around() {
        let mut q = TxQueues::new();
        let mut out = [0u8; 8];
        // 書き込み位置がバッファの終端をまたぐまで繰り返す
        for round in 0..20u8 {
            assert!(q.push(TxStream::Response, &[round; 7]));
            assert_eq!(q.pop_chunk(true, &mut out), Some((TxStream::Response, 7)));
            assert_eq!(out[..7], [round; 7]);
        }
    }
}
