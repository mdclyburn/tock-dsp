//! Single-cycle I/O  hardware.

use cortexm0p::nvic::Nvic;
use kernel::debug;
use kernel::hil;
use kernel::utilities::cells::OptionalCell;

use crate::chip::Processor;
use crate::gpio::SIO;
use crate::interrupts;

/// RP2040 interprocessor FIFO.
pub struct FIFO {
    sio: SIO,
    client: (OptionalCell<&'static dyn hil::fifo::FIFOClient<Publisher = Self>>,
             OptionalCell<&'static dyn hil::fifo::FIFOClient<Publisher = Self>>),
}

impl FIFO {
    pub const fn new() -> FIFO {
        FIFO {
            sio: SIO::new(),
            client: (OptionalCell::empty(), OptionalCell::empty()),
        }
    }

    pub fn enable_interrupt(&self) {
        let index = match self.sio.get_processor() {
            Processor::Processor0 => interrupts::SIO_IRQ_PROC0,
            Processor::Processor1 => interrupts::SIO_IRQ_PROC1,
        };

        unsafe { Nvic::new(index) }.enable();
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
            let client = match self.sio.get_processor() {
                Processor::Processor0 => self.client.0.extract(),
                Processor::Processor1 => self.client.1.extract(),
            };

            if let Some(fifo_client) = client {
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
        match self.sio.get_processor() {
            Processor::Processor0 => &self.client.0.set(client),
            Processor::Processor1 => &self.client.1.set(client),
        };
    }
}
