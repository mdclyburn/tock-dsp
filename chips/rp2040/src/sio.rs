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
        if let Some(fifo_client) = self.client.extract() {
            while self.sio.fifo_valid() {
                let data = self.sio.read_fifo();
                fifo_client.data_received(data);
            }
        } else {
            // Clear out FIFO, there's no client to receive the data.
            while self.sio.fifo_valid() {
                let discarded_data = self.sio.read_fifo();
                debug!("core{} discarding data from FIFO.",
                       self.sio.get_processor() as u8);
            }
        }
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
