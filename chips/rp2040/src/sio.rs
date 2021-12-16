//! Single-cycle I/O  hardware.

use kernel::debug;
use kernel::hil;
use kernel::utilities::cells::OptionalCell;

use crate::gpio::SIO;

/// RP2040 interprocessor FIFO.
pub struct FIFO {
    sio: SIO,
    client: OptionalCell<&'static dyn hil::fifo::FIFOClient<Publisher = Self>>,
}

impl FIFO {
    pub const fn new() -> FIFO {
        FIFO {
            sio: SIO::new(),
            client: OptionalCell::empty(),
        }
    }

    pub fn handle_interrupt(&self) {
        if self.sio.fifo_error() {
            let state = self.sio.fifo_state();
            debug!("Warning: core{} FIFO {}",
                   self.sio.get_processor() as u8,
                   if state & (1 << 3) != 0 {
                       "read on empty"
                   } else {
                       "write on full"
                   });
            self.clear_error();
        } else {
            if let Some(fifo_client) = self.client.extract() {
                while self.sio.fifo_valid() {
                    let data = self.sio.read_fifo();
                    fifo_client.data_received(data);
                }
            } else {
                debug!("core{} discarding data from FIFO.",
                       self.sio.get_processor() as u8);
                // Clear out FIFO, there's no client to receive the data.
                while self.sio.fifo_valid() {
                    let _discarded_data = self.sio.read_fifo();
                }
            }
        }
    }

    /// Clear the ROE and WOF error bits.
    pub fn clear_error(&self) {
        self.sio.fifo_clear_error()
    }
}

impl hil::fifo::FIFO for FIFO {
    type Data = u32;

    fn ready(&self) -> bool {
        self.sio.fifo_ready()
    }

    fn valid(&self) -> bool {
        self.sio.fifo_valid()
    }

    fn write(&self, data: Self::Data) {
        self.sio.write_fifo(data)
    }

    fn read(&self) -> Option<Self::Data> {
        if self.ready() {
            Some(self.sio.read_fifo())
        } else {
            None
        }
    }

    fn set_client(&self, client: &'static dyn hil::fifo::FIFOClient<Publisher = Self>) {
        self.client.set(client)
    }
}
