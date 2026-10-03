#![cfg_attr(target_os = "none", no_std, no_main)]

// ホストでは何もしない。ワークスペース全体を `cargo build` できるようにするためだけにある
#[cfg(not(target_os = "none"))]
fn main() {}

#[cfg(target_os = "none")]
use defmt_rtt as _; // global logger
#[cfg(target_os = "none")]
use panic_probe as _;

#[cfg(target_os = "none")]
mod board;
#[cfg(target_os = "none")]
mod can;

// same panicking *behavior* as `panic-probe` but doesn't print a panic message
// this prevents the panic message being printed *twice* when `defmt::panic` is invoked
#[cfg(target_os = "none")]
#[defmt::panic_handler]
fn panic() -> ! {
    cortex_m::asm::udf()
}

#[cfg(target_os = "none")]
#[rticx_cortex_m::app(device = stm32f1::stm32f103, dispatchers = [SPI1, SPI2])]
mod app {
    #[shared]
    struct Shared {
        count: u32,
    }

    #[init]
    fn init() -> (Shared, TaskInits) {
        (
            Shared { count: 0 },
            TaskInits { idle: Idle, current_loop: CurrentLoop { tick: 0 }, outer_loop: OuterLoop { seen: 0 }, command: Command },
        )
    }

    #[idle]
    struct Idle;

    impl RticIdleTask for Idle {
        fn exec(&mut self) -> ! {
            loop {
                cortex_m::asm::wfi();
            }
        }
    }

    #[task(binds = ADC1_2, priority = 4, shared = [count])]
    struct CurrentLoop {
        tick: u32,
    }

    impl RticTask for CurrentLoop {
        fn exec(&mut self) {
            let mut s = self.shared();
            s.count.lock(|c| *c += 1);
            self.tick += 1;
            if self.tick >= 20 {
                self.tick = 0;
                let _ = OuterLoop::spawn(self.tick as u16);
            }
        }
    }

    #[sw_task(priority = 3, shared = [count])]
    struct OuterLoop {
        seen: u32,
    }

    impl RticSwTask for OuterLoop {
        type SpawnInput = u16;

        fn exec(&mut self, input: u16) {
            let mut s = self.shared();
            self.seen = s.count.lock(|c| *c) + input as u32;
            let _ = Command::spawn(());
        }
    }

    #[sw_task(priority = 1, capacity = 4)]
    struct Command;

    impl RticSwTask for Command {
        type SpawnInput = ();

        fn exec(&mut self, _input: ()) {
            defmt::info!("command");
        }
    }
}
