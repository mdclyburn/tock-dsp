//! DSP stats access interface.

use kernel::dsp::engine::Statistics;
use kernel::errorcode::ErrorCode;
use kernel::process::ProcessId;
use kernel::sync::Mutex;
use kernel::syscall::{CommandReturn, SyscallDriver};
use kernel::utilities::cells::MapCell;

pub struct DSPStats {
    stats: MapCell<Mutex<Statistics>>,
}

impl DSPStats {
    pub fn new() -> DSPStats {
        DSPStats {
            stats: MapCell::empty(),
        }
    }

    pub fn add_stats(&self, stats: Mutex<Statistics>) {
        self.stats.put(stats)
    }
}

impl SyscallDriver for DSPStats {
    fn allocate_grant(&self, _pid: ProcessId) -> Result<(), kernel::process::Error> { Ok(()) }

    fn command(&self, no: usize, r2: usize, r3: usize, _pid: ProcessId) -> CommandReturn {
        match (no, r2, r3) {
            (0, _, _) => if self.stats.is_some() {
                CommandReturn::success()
            } else {
                CommandReturn::failure(ErrorCode::OFF)
            },

            // Statistics query
            (1, stat_no, _) => {
                self.stats.map_or(CommandReturn::failure(ErrorCode::OFF), |stats| {
                    stats.map_or(CommandReturn::failure(ErrorCode::BUSY), |stats| {
                        match stat_no {
                            1 => CommandReturn::success_u32(stats.collect_process_us),
                            _ => CommandReturn::failure(ErrorCode::INVAL),
                        }
                    })
                })
            },

            (_, _, _) => CommandReturn::failure(ErrorCode::INVAL),
        }
    }
}
