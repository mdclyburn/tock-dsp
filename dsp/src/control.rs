//! Controller-DSP engine communication.

#[derive(Copy, Clone)]
pub enum State {
    Running,
    Suspended,
    Faulted,
}

#[derive(Copy, Clone)]
pub enum Command {
    GetState,
    Suspend,
    Resume,
}

pub enum Response {
    Acknowledge,
    CurrentState(State),
}

pub trait Controller {
    fn suspend(&self);

    fn resume(&self);

    fn current_state(&self);

    fn response_received(&self, response: Response);
}

pub trait CommandReceiver {
    fn next_pending(&self) -> Option<Command>;
}
