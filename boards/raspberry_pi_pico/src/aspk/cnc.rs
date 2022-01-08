//! DSP command and control implementations.

use dsp::control::{Command, CommandReceiver, Controller, Response};
use kernel::static_init;
use kernel::hil;
use kernel::utilities::cells::OptionalCell;
use rp2040::sio;

pub struct FIFOController {
    fifo: &'static sio::FIFO,
}

impl FIFOController {
    pub(crate) unsafe fn new(fifo: &'static sio::FIFO) -> &'static FIFOController {
        let ctrl = static_init!(FIFOController, FIFOController {
            fifo,
        });
        hil::fifo::FIFO::set_client(fifo, ctrl);

        ctrl
    }
}

impl hil::fifo::FIFOClient for FIFOController {
    type Publisher = sio::FIFO;

    fn data_received(&self, data: u32) {  }
}

impl Controller for FIFOController {
    fn suspend(&self) {  }

    fn resume(&self) {  }

    fn current_state(&self) {  }

    fn response_received(&self, _response: Response) {  }
}

pub struct FIFOCommandReceiver {
    fifo: &'static sio::FIFO,
    latest_command: OptionalCell<Command>,
}

impl FIFOCommandReceiver {
    /// Create a static [`FIFOCommandReceiver`].
    pub(crate) unsafe fn new(fifo: &'static sio::FIFO) -> &'static FIFOCommandReceiver {
        let recv = static_init!(FIFOCommandReceiver, FIFOCommandReceiver {
            fifo,
            latest_command: OptionalCell::empty(),
        });
        hil::fifo::FIFO::set_client(fifo, recv);

        recv
    }
}

impl hil::fifo::FIFOClient for FIFOCommandReceiver {
    type Publisher = sio::FIFO;

    fn data_received(&self, data: u32) {  }
}

impl CommandReceiver for FIFOCommandReceiver {
    fn next_pending(&self) -> Option<Command> {
        self.latest_command.take()
    }
}
