//! Interprocessor communication for ASPK.

use kernel::hil::fifo::FIFOClient;
use kernel::platform::interprocessor::{
    InterprocessorMessenger,
    MessageDispatcher,
};
use kernel::utilities::cells::MapCell;
use rp2040::chip::Processor;
use rp2040::gpio::SIO;

/// Messages processors can pass and receive.
pub enum Message {
    /// Sending core has panicked.
    Panicked,
    /// Sending core has faulted.
    Faulted,
}

pub enum MessagingError {
    /// FIFO mailbox is full.
    Full,
    /// Received message is not valid.
    Uninterpretable,
}

/// Size of the receiving buffer, the length of the hardware FIFO.
const FIFO_BUFFER_LEN: usize = 32 * 4;

/// Data reception state.
#[derive(Copy, Clone)]
enum MessagingState {
    /// Awaiting the start of reception of new data.
    Idle,
    /// Accepting a message with a specific size.
    Expecting(usize),
}

/// Per-core messaging data.
#[derive(Copy, Clone)]
struct CoreData {
    state: MessagingState,
    rx_buffer: [u32; FIFO_BUFFER_LEN],
    write_next: usize,
}

/// Interprocessor messaging for ASPK.
pub struct ASPKMessaging {
    core0_data: MapCell<CoreData>,
    core1_data: MapCell<CoreData>,
}

impl ASPKMessaging {
    pub fn new() -> ASPKMessaging {
        let data = CoreData {
            state: MessagingState::Idle,
            rx_buffer: [0; FIFO_BUFFER_LEN],
            write_next: 0,
        };

        ASPKMessaging {
            core0_data: MapCell::new(data),
            core1_data: MapCell::new(data),
        }
    }

    fn core_data(&self) -> &MapCell<CoreData> {
        match SIO::new().get_processor() {
            Processor::Processor0 => &self.core0_data,
            Processor::Processor1 => &self.core1_data,
        }
    }
}

impl FIFOClient for ASPKMessaging {
    type Publisher = rp2040::sio::FIFO;

    fn data_received(&self, data: <Self::Publisher as kernel::hil::fifo::FIFO>::Data) {
        let core_data = self.core_data();
        core_data.map(|d| {
            d.rx_buffer[d.write_next] = data;
            d.write_next += 1;

            match d.state {
                MessagingState::Idle => {
                    let expected_length = data & 0b0111_1111;
                    // Length should always be greater than two because we use one word for the header.
                    assert!(expected_length > 1);
                    d.state = MessagingState::Expecting(expected_length as usize);
                },

                MessagingState::Expecting(len) => {
                    // Try parsing it if the message is complete.
                    if d.write_next == len {
                        todo!("parse message");
                        d.state = MessagingState::Idle;
                        d.write_next = 0;
                    }
                },
            }
        });
    }
}

impl InterprocessorMessenger for ASPKMessaging {
    type Identifier = Processor;
    type Message = Message;
    type SendError = MessagingError;
    type ReceiveError = MessagingError;

    fn id(&self) -> Self::Identifier { SIO::new().get_processor() }

    fn send(&self,
            message: &Self::Message,
            recipient: Self::Identifier)
            -> Result<(), Self::SendError>
    {
        let sio = SIO::new();
        unimplemented!()
    }
}

impl MessageDispatcher for ASPKMessaging {
    type Messenger = Self;

    fn on_message_received(&self,
                           ipm: &Self::Messenger,
                           from: <Self::Messenger as InterprocessorMessenger>::Identifier,
                           message: &<Self::Messenger as InterprocessorMessenger>::Message)
                           -> Result<(), <Self::Messenger as InterprocessorMessenger>::ReceiveError>
    {
        Err(MessagingError::Uninterpretable)
    }
}
