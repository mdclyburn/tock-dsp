//! DSP command and control implementations.

use core::cell::Cell;

use dsp::control::{
    Command,
    CommandReceiver,
    Controller,
    Response,
    ResponseReceiver,
    State,
};
use kernel::static_init;
use kernel::hil;
use kernel::utilities::cells::{MapCell, NumericCellExt, OptionalCell};
use rp2040::sio;

/// Controller messaging through the FIFO.
///
/// Uses the RP2040 interprocessor FIFO to implement controller-engine messaging.
/// This [`Controller`] requires a [`ResponseReceiver`] to accep the final message
/// since all data associated with a single message may not appear in the FIFO immediately.
pub struct FIFOController {
    fifo: &'static sio::FIFO,
    client: OptionalCell<&'static dyn ResponseReceiver>,
    recv_buf: MapCell<[u32; 5]>,
    recv_idx: Cell<usize>,
}

impl FIFOController {
    /// Create a static `FIFOController`.
    pub(crate) unsafe fn new(fifo: &'static sio::FIFO) -> &'static FIFOController {
        // Create the ring buffer to hold the unfinished responses.
        let ctrl = static_init!(FIFOController, FIFOController {
            fifo,
            client: OptionalCell::empty(),
            recv_buf: MapCell::new([0; 5]),
            recv_idx: Cell::new(0),
        });
        hil::fifo::FIFO::set_client(fifo, ctrl);

        ctrl
    }
}

impl hil::fifo::FIFOClient for FIFOController {
    type Publisher = sio::FIFO;

    /// New data has arrived from the FIFO.
    ///
    /// Add the word to the FIFO.
    /// Because this is the controller side and not the engine,
    /// this function makes a call to attempt to parse through the data.
    /// When the data matches a known response, this function additionally passes the response to the assigned client.
    fn data_received(&self, data: u32) {
        let maybe_response = self.recv_buf.map(|buf| {
            let idx = self.recv_idx.get_and_increment();
            buf[idx as usize] = data;
            deserialize_response(&buf[0..idx+1])
        }).unwrap(); // No code in any impl takes the buffer.

        if let Some(response) = maybe_response {
            self.client.extract()
                .unwrap()
                .response_received(response);
            self.recv_idx.set(0);
        }
    }
}

impl Controller for FIFOController {
    fn set_receiver(&self, receiver: &'static dyn ResponseReceiver) {
        self.client.set(receiver);
    }

    fn suspend(&self) {  }

    fn resume(&self) {  }

    fn current_state(&self) {  }
}

/// Command receiver through the FIFO.
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

fn serialize_command(command: Command, out: &mut [u32]) {
    use Command::*;
    match command {
        GetState => {
            out[0] = 1;
        },

        Suspend => {
            out[1] = 2;
        },

        Resume => {
            out[2] = 3;
        },
    };
}

fn deserialize_command(data: &[u32]) -> Option<Command> {
    use Command::*;
    match data[0] {
        1 => Some(GetState),
        2 => Some(Suspend),
        3 => Some(Resume),
        _ => unimplemented!(),
    }
}

fn serialize_response(response: Response, out: &mut [u32]) {
    use Response::*;
    match response {
        Acknowledge => {
            out[0] = 1;
        },

        CurrentState(state) => {
            out[0] = 2;
            use State::*;
            out[1] = match state {
                Running => 1,
                Suspended => 2,
            };
        },
    }
}

fn deserialize_response(data: &[u32]) -> Option<Response> {
    use Response::*;
    match data[0] {
        1 => Some(Acknowledge),
        2 => {
            if data.len() < 2 {
                None
            } else {
                match data[1] {
                    1 => Some(CurrentState(State::Running)),
                    2 => Some(CurrentState(State::Suspended)),
                    // Bad state serialization.
                    _ => panic!(),
                }
            }
        },
        x => {
            kernel::debug!("discarding unknown engine response: {}", x);
            None
        },
    }
}
