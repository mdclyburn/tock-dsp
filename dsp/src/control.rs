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

pub trait ControllerMailbox {
    fn suspend(&self) -> Result<(), ()>;

    fn suspend_complete(&self);

    fn resume(&self) -> Result<(), ()>;

    fn resume_complete(&self);

    fn current_state(&self) -> Result<(), ()>;

    fn current_state_complete(&self, state: State);
}

pub trait CommandMailbox {
    fn next_pending(&self) -> Option<Command>;
}
