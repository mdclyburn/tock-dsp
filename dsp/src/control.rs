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

    fn resume(&self) -> Result<(), ()>;

    fn current_state(&self) -> Result<State, ()>;
}

pub trait CommandMailbox {
    fn next_pending(&self) -> Option<Command>;
}
