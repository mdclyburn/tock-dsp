//! Interprocessor communication.

/// Communicates data between cores in a multicore system using specialized hardware structures.
pub trait InterprocessorMessenger {
    /// Unique identifier for each core participating in messaging.
    type Identifier: Copy + Clone;
    /// Structure of messages the implementation passes between cores.
    type Message: Copy + Clone;
    /// Error cases for message transmission failures.
    type SendError;
    /// Error cases for message reception failures.
    type ReceiveError;

    /// Executing core's unique identifier.
    fn id(&self) -> Self::Identifier;

    /// Send a message to another core.
    fn send(&self,
            message: Self::Message,
            recipient: Self::Identifier)
            -> Result<(), Self::SendError>;

    fn message_received(&self,
                        from: Self::Identifier,
                        message: Self::Message);
}

impl InterprocessorMessenger for () {
    type Identifier = ();
    type Message = ();
    type SendError = ();
    type ReceiveError = ();

    fn id(&self) -> () { () }

    fn send(&self,
            _message: Self::Message,
            _recipient: Self::Identifier) -> Result<(), Self::SendError>
    {
        empty_implementation_panic();
    }

    fn message_received(&self,
                        _from: Self::Identifier,
                        _message: Self::Message)
    {
        empty_implementation_panic();
    }
}

#[inline(always)]
fn empty_implementation_panic() -> ! {
    unimplemented!("Interprocessor comm. impossible with empty implementation.");
}
