//! Controller-DSP engine communication.

/// Current state of the DSP engine.
#[derive(Copy, Clone)]
pub enum State {
    /// Currently running.
    Running,
    /// Not currently running.
    Suspended,
}

/// Commands to manipulate DSP engine operation.
///
/// The engine will send a [`Response`] in response to receiving a `Command`.
#[derive(Copy, Clone)]
pub enum Command {
    /// Request the engine's current state.
    ///
    /// The engine will send `Response::CurrentState(State)` in response.
    GetState,
    /// Pause engine processing.
    ///
    /// The engine will send `Response::Acknowledge` in response.
    Suspend,
    /// Continue engine processing.
    ///
    /// The engine will send `Response::Acknowledge` in response.
    Resume,
}

/// Messages sent from the DSP engine.
#[derive(Copy, Clone)]
pub enum Response {
    /// The engine completed enacting the command.
    ///
    /// If there is no more specific response to send back,
    /// then the engine will respond with this `Response`.
    Acknowledge,
    /// Response containing the engine's current state.
    CurrentState(State),
}

/// Client that accepts complete responses from the DSP engine.
///
/// An implementor of this trait acts as the recipient of the response from the DSP engine.
/// This trait allows separating the responsibility of handling the raw response format from dealing with the message itself.
pub trait ResponseReceiver {
    /// Callback function the `Controller` calls after processing a `Response`.
    fn response_received(&self, response: Response);
}

/// Manipulate DSP engine operation.
pub trait Controller {
    /// Set the endpoint receiving responses from the DSP engine.
    fn set_receiver(&self, receiver: &'static dyn ResponseReceiver);

    /// Command the DSP engine to pause operation.
    fn suspend(&self);

    /// Command the DSP engine to resume operation.
    fn resume(&self);

    /// Command the DSP engine to report its current state.
    fn current_state(&self);
}

/// Receive and enact `Controller` commands.
pub trait CommandReceiver {
    /// Retrieve a pending [`Command`], or `None` if there are none pending.
    fn next_pending(&self) -> Option<Command>;
}
