//! Interprocessor communication for ASPK.

use kernel::debug;
use kernel::hil::fifo::FIFOClient;
use kernel::platform::interprocessor::InterprocessorMessenger;
use kernel::utilities::cells::MapCell;
use rp2040::chip::Processor;
use rp2040::gpio::SIO;

/// Messages processors can pass and receive.
#[derive(Copy, Clone)]
pub enum Message {
    /// Sending core has panicked.
    Panicked,
    /// Sending core has faulted.
    Faulted,
    /// DSP core is online.
    DSPRunning,
}

pub enum MessagingError {
    /// FIFO mailbox is full.
    Full,
    /// Received message is not valid.
    Uninterpretable,
}

/// Size of the receiving buffer.
const FIFO_BUFFER_LEN: usize = 2;

/// Per-core messaging data.
#[derive(Copy, Clone)]
struct CoreData {
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

impl InterprocessorMessenger for ASPKMessaging {
    type Identifier = Processor;
    type Message = Message;
    type SendError = MessagingError;
    type ReceiveError = MessagingError;

    fn id(&self) -> Self::Identifier { SIO::new().get_processor() }

    fn send(&self,
            message: Self::Message,
            recipient: Self::Identifier)
            -> Result<(), Self::SendError>
    {
        use Message::*;
        match message {
            _ => unimplemented!(),
        };
    }

    fn message_received(&self,
                        from: Self::Identifier,
                        message: Self::Message)
    {
        use Message::*;
        match message {
            Panicked => debug!("core{} panicked.", from as u8),
            Faulted => debug!("core{} faulted.", from as u8),
            DSPRunning => debug!("core{} is online.", from as u8),
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

            if d.write_next == 2 {
                d.write_next = 0;

                let (head, data) = (d.rx_buffer[0], d.rx_buffer[1]);
                let other_core = match self.id() {
                    Processor::Processor0 => Processor::Processor1,
                    Processor::Processor1 => Processor::Processor0
                };

                match head {
                    0 => self.message_received(other_core, Message::Faulted),
                    1 => self.message_received(other_core, Message::Panicked),
                    // 2 => self.message_received(other_core, Message::DSPRunning),
                    _ => debug!("Message dropped (core{} → core{}): {:#010X} {:#010X}",
                                other_core as u8, self.id() as u8, head, data),
                };
            }
        });
    }
}
