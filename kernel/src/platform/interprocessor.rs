//! Interprocessor communication.

/// Communicates data between cores in a multicore system using specialized hardware structures.
pub trait InterprocessorMessenger {
    /// Unique identifier for each core participating in messaging.
    type Identifier: Copy + Clone;
    /// Structure of messages the implementation passes between cores.
    type Message;
    /// Error cases for message transmission failures.
    type SendError;
    /// Error cases for message reception failures.
    type ReceiveError;

    /// Executing core's unique identifier.
    fn id(&self) -> Self::Identifier;

    /// Send a message to another core.
    fn send(&self,
            message: &Self::Message,
            recipient: Self::Identifier) -> Result<(), Self::SendError>;
}

impl InterprocessorMessenger for () {
    type Identifier = ();
    type Message = ();
    type SendError = ();
    type ReceiveError = ();

    fn id(&self) -> () { () }

    fn send(&self,
            _message: &Self::Message,
            _recipient: Self::Identifier) -> Result<(), Self::SendError>
    {
        empty_implementation_panic();
    }
}

/// Handler for incoming messages from other cores.
pub trait MessageDispatcher {
    /// Interprocessor messaging implementation in use.
    type Messenger: InterprocessorMessenger;

    /// Hook called when the core receives a message.
    #[allow(unused_variables)]
    fn on_message_received(&self,
                           ipm: &Self::Messenger,
                           from: <Self::Messenger as InterprocessorMessenger>::Identifier,
                           message: &<Self::Messenger as InterprocessorMessenger>::Message)
                           -> Result<(), <Self::Messenger as InterprocessorMessenger>::ReceiveError>;
}

impl MessageDispatcher for () {
    type Messenger = ();

    fn on_message_received(&self,
                           _ipm: &Self::Messenger,
                           _from: <Self::Messenger as InterprocessorMessenger>::Identifier,
                           _message: &<Self::Messenger as InterprocessorMessenger>::Message)
                           -> Result<(), <Self::Messenger as InterprocessorMessenger>::ReceiveError>
    {
        empty_implementation_panic();
    }
}

#[inline(always)]
fn empty_implementation_panic() -> ! {
    unimplemented!("Interprocessor comm. impossible with empty implementation.");
}
