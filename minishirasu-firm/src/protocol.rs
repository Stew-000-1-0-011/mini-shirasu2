//! バイトストリーム上のメッセージ。
//! フレーミングは COBS( 種別 | ペイロード | CRC-8 ) のあとに区切りの 0x00。数値はリトルエンディアン

use crc::{CRC_8_SMBUS, Crc};

const CRC8: Crc<u8> = Crc::<u8>::new(&CRC_8_SMBUS);

/// 符号化後の1メッセージの最大長(区切りを含む)。パーサのバッファ長でもある
pub const MAX_ENCODED: usize = 32;
/// 種別 + ペイロード + CRC の最大長。Statusの22バイト
const MAX_BODY: usize = 22;

/// メッセージの種別
pub mod kind {
    pub const TARGET_CURRENT: u8 = 0x01;
    pub const TARGET_VELOCITY: u8 = 0x02;
    pub const TARGET_POSITION: u8 = 0x03;
    pub const SET_MODE: u8 = 0x10;
    pub const SET_PARAM: u8 = 0x11;
    pub const RESET_FAULT: u8 = 0x12;
    pub const SET_ORIGIN: u8 = 0x13;
    pub const STATUS: u8 = 0x20;
    pub const ACK: u8 = 0x30;
    pub const NACK: u8 = 0x31;
    pub const FAULT_NOTICE: u8 = 0x32;
}

#[derive(Clone, Copy, PartialEq, Debug)]
pub struct Status {
    pub mode: u8,
    /// bit0: 上側に飽和、bit1: 下側に飽和、bit2: 母線電圧が低く更新できていない
    pub flags: u8,
    pub current: f32,  // [A]
    pub velocity: f32, // [rad/s]
    pub position: i32, // [回転] Q16.16
    pub vdc: f32,      // [V]
    pub temp: u16,     // ADCの生値
}

#[derive(Clone, Copy, PartialEq, Debug)]
pub enum Message {
    TargetCurrent { current: f32 },
    TargetVelocity { velocity: f32, accel_ff: f32 },
    /// positionは回転単位のQ16.16
    TargetPosition { position: i32, velocity_ff: f32, accel_ff: f32 },
    SetMode { mode: u8 },
    SetParam { id: u8, value: f32 },
    ResetFault,
    SetOrigin,
    Status(Status),
    Ack { command: u8 },
    Nack { command: u8, reason: u8, param: u8 },
    FaultNotice,
}

#[derive(Clone, Copy, PartialEq, Debug)]
pub enum DecodeError {
    /// COBSとして復号できない
    Cobs,
    /// 種別とCRCの2バイトに満たない
    TooShort,
    Crc,
    UnknownKind(u8),
    /// 種別に対して長さが合わない
    Length(u8),
    /// 区切りが来ないままバッファがあふれた
    Overflow,
}

/// ペイロードの長さ。未知の種別はNone
fn payload_len(kind: u8) -> Option<usize> {
    Some(match kind {
        kind::TARGET_CURRENT => 4,
        kind::TARGET_VELOCITY => 8,
        kind::TARGET_POSITION => 12,
        kind::SET_MODE => 1,
        kind::SET_PARAM => 5,
        kind::RESET_FAULT | kind::SET_ORIGIN | kind::FAULT_NOTICE => 0,
        kind::STATUS => 20,
        kind::ACK => 1,
        kind::NACK => 3,
        _ => return None,
    })
}

struct Writer {
    buf: [u8; MAX_BODY],
    len: usize,
}

impl Writer {
    fn bytes(&mut self, v: &[u8]) {
        self.buf[self.len..self.len + v.len()].copy_from_slice(v);
        self.len += v.len();
    }

    fn u8(&mut self, v: u8) {
        self.bytes(&[v]);
    }

    fn u16(&mut self, v: u16) {
        self.bytes(&v.to_le_bytes());
    }

    fn i32(&mut self, v: i32) {
        self.bytes(&v.to_le_bytes());
    }

    fn f32(&mut self, v: f32) {
        self.bytes(&v.to_le_bytes());
    }
}

/// 長さを確かめたあとのペイロードを順に読む
struct Reader<'a> {
    buf: &'a [u8],
}

impl Reader<'_> {
    fn take<const N: usize>(&mut self) -> [u8; N] {
        let (head, rest) = self.buf.split_at(N);
        self.buf = rest;
        let mut out = [0u8; N];
        out.copy_from_slice(head);
        out
    }

    fn u8(&mut self) -> u8 {
        self.take::<1>()[0]
    }

    fn u16(&mut self) -> u16 {
        u16::from_le_bytes(self.take())
    }

    fn i32(&mut self) -> i32 {
        i32::from_le_bytes(self.take())
    }

    fn f32(&mut self) -> f32 {
        f32::from_le_bytes(self.take())
    }
}

impl Message {
    pub fn kind(&self) -> u8 {
        match self {
            Message::TargetCurrent { .. } => kind::TARGET_CURRENT,
            Message::TargetVelocity { .. } => kind::TARGET_VELOCITY,
            Message::TargetPosition { .. } => kind::TARGET_POSITION,
            Message::SetMode { .. } => kind::SET_MODE,
            Message::SetParam { .. } => kind::SET_PARAM,
            Message::ResetFault => kind::RESET_FAULT,
            Message::SetOrigin => kind::SET_ORIGIN,
            Message::Status(_) => kind::STATUS,
            Message::Ack { .. } => kind::ACK,
            Message::Nack { .. } => kind::NACK,
            Message::FaultNotice => kind::FAULT_NOTICE,
        }
    }

    /// 区切りの0x00まで含めて書き、その長さを返す
    pub fn encode(&self, out: &mut [u8; MAX_ENCODED]) -> usize {
        let mut w = Writer { buf: [0; MAX_BODY], len: 0 };
        w.u8(self.kind());
        match *self {
            Message::TargetCurrent { current } => w.f32(current),
            Message::TargetVelocity { velocity, accel_ff } => {
                w.f32(velocity);
                w.f32(accel_ff);
            }
            Message::TargetPosition { position, velocity_ff, accel_ff } => {
                w.i32(position);
                w.f32(velocity_ff);
                w.f32(accel_ff);
            }
            Message::SetMode { mode } => w.u8(mode),
            Message::SetParam { id, value } => {
                w.u8(id);
                w.f32(value);
            }
            Message::ResetFault | Message::SetOrigin | Message::FaultNotice => {}
            Message::Status(s) => {
                w.u8(s.mode);
                w.u8(s.flags);
                w.f32(s.current);
                w.f32(s.velocity);
                w.i32(s.position);
                w.f32(s.vdc);
                w.u16(s.temp);
            }
            Message::Ack { command } => w.u8(command),
            Message::Nack { command, reason, param } => {
                w.u8(command);
                w.u8(reason);
                w.u8(param);
            }
        }
        let crc = CRC8.checksum(&w.buf[..w.len]);
        w.u8(crc);

        let n = cobs::encode(&w.buf[..w.len], &mut out[..]);
        out[n] = 0;
        n + 1
    }

    /// COBSを解いたあとの 種別 | ペイロード | CRC を読む
    pub fn decode_body(body: &[u8]) -> Result<Message, DecodeError> {
        if body.len() < 2 {
            return Err(DecodeError::TooShort);
        }
        let (content, crc) = body.split_at(body.len() - 1);
        if CRC8.checksum(content) != crc[0] {
            return Err(DecodeError::Crc);
        }
        let kind = content[0];
        let payload = &content[1..];
        let len = payload_len(kind).ok_or(DecodeError::UnknownKind(kind))?;
        if payload.len() != len {
            return Err(DecodeError::Length(kind));
        }

        let mut r = Reader { buf: payload };
        Ok(match kind {
            kind::TARGET_CURRENT => Message::TargetCurrent { current: r.f32() },
            kind::TARGET_VELOCITY => Message::TargetVelocity { velocity: r.f32(), accel_ff: r.f32() },
            kind::TARGET_POSITION => {
                Message::TargetPosition { position: r.i32(), velocity_ff: r.f32(), accel_ff: r.f32() }
            }
            kind::SET_MODE => Message::SetMode { mode: r.u8() },
            kind::SET_PARAM => Message::SetParam { id: r.u8(), value: r.f32() },
            kind::RESET_FAULT => Message::ResetFault,
            kind::SET_ORIGIN => Message::SetOrigin,
            kind::STATUS => Message::Status(Status {
                mode: r.u8(),
                flags: r.u8(),
                current: r.f32(),
                velocity: r.f32(),
                position: r.i32(),
                vdc: r.f32(),
                temp: r.u16(),
            }),
            kind::ACK => Message::Ack { command: r.u8() },
            kind::NACK => Message::Nack { command: r.u8(), reason: r.u8(), param: r.u8() },
            // payload_lenがSomeを返す種別はここまでで尽きている
            _ => Message::FaultNotice,
        })
    }
}

/// 区切りのないバイト列からメッセージを切り出す
pub struct StreamParser {
    buf: [u8; MAX_ENCODED],
    len: usize,
    overflow: bool,
}

impl StreamParser {
    pub const fn new() -> StreamParser {
        StreamParser { buf: [0; MAX_ENCODED], len: 0, overflow: false }
    }

    /// 1バイト進める。区切りを受けてメッセージが1つ確定したら、その結果を返す
    pub fn push(&mut self, byte: u8) -> Option<Result<Message, DecodeError>> {
        if byte != 0 {
            if self.len < MAX_ENCODED {
                self.buf[self.len] = byte;
                self.len += 1;
            } else {
                self.overflow = true;
            }
            return None;
        }

        let len = core::mem::replace(&mut self.len, 0);
        if core::mem::replace(&mut self.overflow, false) {
            return Some(Err(DecodeError::Overflow));
        }
        // 区切りが続いただけ
        if len == 0 {
            return None;
        }

        let mut body = [0u8; MAX_ENCODED];
        Some(match cobs::decode(&self.buf[..len], &mut body) {
            Ok(report) => Message::decode_body(&body[..report.frame_size()]),
            Err(_) => Err(DecodeError::Cobs),
        })
    }
}


#[cfg(test)]
mod tests {
    use super::*;

    fn encode(m: &Message) -> Vec<u8> {
        let mut out = [0u8; MAX_ENCODED];
        let n = m.encode(&mut out);
        out[..n].to_vec()
    }

    fn feed(p: &mut StreamParser, bytes: &[u8]) -> Vec<Result<Message, DecodeError>> {
        bytes.iter().filter_map(|&b| p.push(b)).collect()
    }

    fn samples() -> Vec<Message> {
        vec![
            Message::TargetCurrent { current: 1.5 },
            Message::TargetVelocity { velocity: -120.0, accel_ff: 30.5 },
            Message::TargetPosition { position: -98304, velocity_ff: 2.0, accel_ff: 0.0 },
            Message::SetMode { mode: 2 },
            Message::SetParam { id: 0x21, value: 8192.0 },
            Message::ResetFault,
            Message::SetOrigin,
            Message::Status(Status {
                mode: 3,
                flags: 0b101,
                current: 0.25,
                velocity: 0.0,
                position: 0,
                vdc: 24.0,
                temp: 2048,
            }),
            Message::Ack { command: kind::SET_MODE },
            Message::Nack { command: kind::SET_PARAM, reason: 3, param: 0x04 },
            Message::FaultNotice,
        ]
    }

    #[test]
    fn crc_is_crc8_smbus() {
        // CRC-8/SMBUS の検査値
        assert_eq!(CRC8.checksum(b"123456789"), 0xF4);
    }

    #[test]
    fn every_message_roundtrips() {
        for m in samples() {
            let mut p = StreamParser::new();
            assert_eq!(feed(&mut p, &encode(&m)), vec![Ok(m)], "{m:?}");
        }
    }

    #[test]
    fn encoded_lengths_match_spec() {
        let lengths: Vec<usize> = samples().iter().map(|m| encode(m).len()).collect();
        assert_eq!(lengths, vec![8, 12, 16, 5, 9, 4, 4, 24, 5, 7, 4]);
    }

    #[test]
    fn encoded_message_has_zero_only_at_the_end() {
        for m in samples() {
            let bytes = encode(&m);
            let (last, body) = bytes.split_last().unwrap();
            assert_eq!(*last, 0, "{m:?}");
            assert!(body.iter().all(|&b| b != 0), "{m:?}");
        }
    }

    #[test]
    fn kinds_match_spec() {
        let kinds: Vec<u8> = samples().iter().map(|m| m.kind()).collect();
        assert_eq!(kinds, vec![0x01, 0x02, 0x03, 0x10, 0x11, 0x12, 0x13, 0x20, 0x30, 0x31, 0x32]);
    }

    #[test]
    fn consecutive_messages_are_split() {
        let a = Message::SetMode { mode: 1 };
        let b = Message::TargetCurrent { current: -0.5 };
        let mut bytes = encode(&a);
        bytes.extend(encode(&b));
        let mut p = StreamParser::new();
        assert_eq!(feed(&mut p, &bytes), vec![Ok(a), Ok(b)]);
    }

    #[test]
    fn extra_delimiters_are_ignored() {
        let m = Message::ResetFault;
        let mut bytes = vec![0, 0];
        bytes.extend(encode(&m));
        bytes.push(0);
        let mut p = StreamParser::new();
        assert_eq!(feed(&mut p, &bytes), vec![Ok(m)]);
    }

    #[test]
    fn joining_mid_message_drops_only_the_broken_one() {
        let a = Message::TargetVelocity { velocity: 1.0, accel_ff: 2.0 };
        let b = Message::SetOrigin;
        let first = encode(&a);
        // 1つ目は末尾の3バイト(区切りを含む)しか受け取れなかった
        let mut bytes = first[first.len() - 3..].to_vec();
        bytes.extend(encode(&b));
        let mut p = StreamParser::new();
        let got = feed(&mut p, &bytes);
        assert_eq!(got.len(), 2);
        assert!(got[0].is_err());
        assert_eq!(got[1], Ok(b));
    }

    #[test]
    fn lost_byte_drops_only_the_broken_message() {
        let a = Message::TargetPosition { position: 65536, velocity_ff: 1.0, accel_ff: 2.0 };
        let b = Message::Ack { command: 0x10 };
        let mut bytes = encode(&a);
        bytes.remove(5);
        bytes.extend(encode(&b));
        let mut p = StreamParser::new();
        let got = feed(&mut p, &bytes);
        assert_eq!(got.len(), 2);
        assert!(got[0].is_err());
        assert_eq!(got[1], Ok(b));
    }

    #[test]
    fn corrupted_byte_is_rejected() {
        let mut bytes = encode(&Message::SetParam { id: 0x04, value: 6.0 });
        // 区切りを作らないよう、0にも1にもならないバイトを選んで1bit反転する
        let i = bytes.iter().position(|&b| b != 0 && b != 1).unwrap();
        bytes[i] ^= 1;
        let mut p = StreamParser::new();
        let got = feed(&mut p, &bytes);
        assert_eq!(got.len(), 1);
        assert!(got[0].is_err());
    }

    #[test]
    fn overlong_input_is_dropped_and_parser_recovers() {
        let m = Message::FaultNotice;
        let mut bytes = vec![0x55u8; 100];
        bytes.push(0);
        bytes.extend(encode(&m));
        let mut p = StreamParser::new();
        assert_eq!(feed(&mut p, &bytes), vec![Err(DecodeError::Overflow), Ok(m)]);
    }

    #[test]
    fn decode_body_rejects_wrong_crc() {
        let body = [kind::SET_MODE, 1, 0x00];
        assert_ne!(CRC8.checksum(&body[..2]), 0x00);
        assert_eq!(Message::decode_body(&body), Err(DecodeError::Crc));
    }

    #[test]
    fn decode_body_rejects_unknown_kind() {
        let mut body = [0x7F, 0];
        body[1] = CRC8.checksum(&body[..1]);
        assert_eq!(Message::decode_body(&body), Err(DecodeError::UnknownKind(0x7F)));
    }

    #[test]
    fn decode_body_rejects_wrong_length() {
        // SetModeのペイロードは1バイトだが、2バイトある
        let mut body = [kind::SET_MODE, 1, 2, 0];
        body[3] = CRC8.checksum(&body[..3]);
        assert_eq!(Message::decode_body(&body), Err(DecodeError::Length(kind::SET_MODE)));
    }

    #[test]
    fn decode_body_rejects_too_short_input() {
        assert_eq!(Message::decode_body(&[]), Err(DecodeError::TooShort));
        assert_eq!(Message::decode_body(&[0x12]), Err(DecodeError::TooShort));
    }
}
