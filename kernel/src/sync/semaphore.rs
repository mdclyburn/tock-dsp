//! Semaphore synchronization primitive.

use core::cell::Cell;

use super::Lockable;

/// Synchronization primitive for limiting access to critical sections of code.
pub struct Semaphore {
    lock: &'static dyn Lockable,
    count: Cell<i8>,
}

impl Semaphore {
    /// Create a new semaphore.
    ///
    /// Use `max_concurrent` to specify the maximum number of threads of execution allowed to access the same section of code.
    /// Using a non-positive number for this parameter will cause all code calling [`Semaphore::wait`] to hang.
    pub fn new(lock: &'static dyn Lockable, max_concurrent: i8) -> Semaphore {
        Semaphore {
            lock,
            count: Cell::new(max_concurrent),
        }
    }

    /// Await access to a critical code section.
    pub fn wait(&self) {
        loop {
            self.lock.lock();
            if self.count.get() > 0 {
                self.count.set(self.count.get() - 1);
                self.lock.release();
                break;
            }
            self.lock.release();
        }
    }

    /// Signal departure from a critical code section.
    pub fn signal(&self) {
        self.lock.lock();
        self.count.set(self.count.get() + 1);
        self.lock.release();
    }
}
