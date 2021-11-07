//! Hardware FIFO.

/// FIFO operations.
pub trait FIFO {
    type Data: Copy + Clone;

    /// Returns true if the FIFO has space for data.
    fn ready(&self) -> bool;

    /// Returns true if the FIFO has data available for a read.
    fn valid(&self) -> bool;

    /// Write data to the FIFO.
    fn write(&self, data: Self::Data);

    /// Read data from the FIFO.
    fn read(&self) -> Option<Self::Data>;

    /// Assign a client to receive data from the FIFO.
    fn set_client(&self, client: &'static dyn FIFOClient<Publisher = Self>);
}

/// Subscriber for FIFO events.
pub trait FIFOClient {
    /// FIFO the client is for.
    type Publisher: FIFO;

    fn data_received(&self, data: <Self::Publisher as FIFO>::Data);
}
