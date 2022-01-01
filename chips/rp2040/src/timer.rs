use core::cell::Cell;

use cortexm0p;
use cortexm0p::support::atomic;
use kernel::hil;
use kernel::hil::time::{Ticks, Ticks32, Time};
use kernel::utilities::cells::OptionalCell;
use kernel::utilities::registers::interfaces::{Readable, Writeable};
use kernel::utilities::registers::{
    register_bitfields, register_structs, ReadOnly, ReadWrite, WriteOnly,
};
use kernel::utilities::StaticRef;
use kernel::ErrorCode;

use crate::interrupts;

register_structs! {
    /// Controls time and alarms\n
    /// time is a 64 bit value indicating the time in usec since power-on\n
    /// timeh is the top 32 bits of time & timel is the bottom 32 bits\n
    /// to change time write to timelw before timehw\n
    /// to read time read from timelr before timehr\n
    /// An alarm is set by setting alarm_enable and writing to the corresponding
    /// When an alarm is pending, the corresponding alarm_running signal will be
    /// An alarm can be cancelled before it has finished by clearing the alarm_e
    /// When an alarm fires, the corresponding alarm_irq is set and alarm_runnin
    /// To clear the interrupt write a 1 to the corresponding alarm_irq
    TimerRegisters {
        /// Write to bits 63:32 of time\n
        /// always write timelw before timehw
        (0x000 => timehw: WriteOnly<u32, TIMEHW::Register>),
        /// Write to bits 31:0 of time\n
        /// writes do not get copied to time until timehw is written
        (0x004 => timelw: WriteOnly<u32, TIMELW::Register>),
        /// Read from bits 63:32 of time\n
        /// always read timelr before timehr
        (0x008 => timehr: ReadOnly<u32, TIMEHR::Register>),
        /// Read from bits 31:0 of time
        (0x00C => timelr: ReadOnly<u32, TIMELR::Register>),
        /// Arm the alarm and configure the time it will fire.
        /// Once armed, the alarm fires when TIMER_ALARMx == TIMELR.
        /// The alarm will disarm itself once it fires, and can
        /// be disarmed early using the ARMED status register.
        (0x010 => alarm: [ReadWrite<u32, ALARMx::Register>; 4]),
        /// Indicates the armed/disarmed status of each alarm.\n
        /// A write to the corresponding ALARMx register arms the alarm.\n
        /// Alarms automatically disarm upon firing, but writing ones here\n
        /// will disarm immediately without waiting to fire.
        (0x020 => armed: ReadWrite<u32>),
        /// Raw read from bits 63:32 of time (no side effects)
        (0x024 => timerawh: ReadOnly<u32, TIMERAWH::Register>),
        /// Raw read from bits 31:0 of time (no side effects)
        (0x028 => timerawl: ReadOnly<u32, TIMERAWL::Register>),
        /// Set bits high to enable pause when the corresponding debug ports are active
        (0x02C => dbgpause: ReadWrite<u32, DBGPAUSE::Register>),
        /// Set high to pause the timer
        (0x030 => pause: ReadWrite<u32>),
        /// Raw Interrupts
        (0x034 => intr: ReadWrite<u32, INTR::Register>),
        /// Interrupt Enable
        (0x038 => inte: ReadWrite<u32, INTE::Register>),
        /// Interrupt Force
        (0x03C => intf: ReadWrite<u32, INTF::Register>),
        /// Interrupt status after masking & forcing
        (0x040 => ints: ReadWrite<u32, INTS::Register>),

        (0x044 => @END),
    }
}

register_bitfields![
    u32,
    TIMEHW [
        VALUE OFFSET (0) NUMBITS (32) []
    ],
    TIMELW [
        VALUE OFFSET (0) NUMBITS (32) []
    ],
    TIMEHR [
        VALUE OFFSET (0) NUMBITS (32) []
    ],
    TIMELR [
        VALUE OFFSET (0) NUMBITS (32) []
    ],
    ALARMx [
        VALUE OFFSET (0) NUMBITS (32) []
    ],
    ARMED [
        ARMED OFFSET(0) NUMBITS(4) []
    ],
    TIMERAWH [
        VALUE OFFSET (0) NUMBITS (32) []
    ],
    TIMERAWL [
        VALUE OFFSET (0) NUMBITS (32) []
    ],
    DBGPAUSE [
        /// Pause when processor 1 is in debug mode
        DBG1 OFFSET(2) NUMBITS(1) [],
        /// Pause when processor 0 is in debug mode
        DBG0 OFFSET(1) NUMBITS(1) []
    ],
    PAUSE [

        PAUSE OFFSET(0) NUMBITS(1) []
    ],
    INTR [

        ALARM_3 OFFSET(3) NUMBITS(1) [],

        ALARM_2 OFFSET(2) NUMBITS(1) [],

        ALARM_1 OFFSET(1) NUMBITS(1) [],

        ALARM_0 OFFSET(0) NUMBITS(1) []
    ],
    INTE [

        ALARM_3 OFFSET(3) NUMBITS(1) [],

        ALARM_2 OFFSET(2) NUMBITS(1) [],

        ALARM_1 OFFSET(1) NUMBITS(1) [],

        ALARM_0 OFFSET(0) NUMBITS(1) []
    ],
    INTF [

        ALARM_3 OFFSET(3) NUMBITS(1) [],

        ALARM_2 OFFSET(2) NUMBITS(1) [],

        ALARM_1 OFFSET(1) NUMBITS(1) [],

        ALARM_0 OFFSET(0) NUMBITS(1) []
    ],
    INTS [

        ALARM_3 OFFSET(3) NUMBITS(1) [],

        ALARM_2 OFFSET(2) NUMBITS(1) [],

        ALARM_1 OFFSET(1) NUMBITS(1) [],

        ALARM_0 OFFSET(0) NUMBITS(1) []
    ]
];

const IRQ_NOS: [u32; 4] = [
    interrupts::TIMER_IRQ_0,
    interrupts::TIMER_IRQ_1,
    interrupts::TIMER_IRQ_2,
    interrupts::TIMER_IRQ_3,
];

const REGISTERS: StaticRef<TimerRegisters> =
    unsafe { StaticRef::new(0x40054000 as *const TimerRegisters) };

pub struct Alarm<'a> {
    no: u8,
    client: OptionalCell<&'a dyn hil::time::AlarmClient>,
}

impl<'a> Alarm<'a> {
    const fn new(alarm_no: u8) -> Alarm<'a> {
        Alarm {
            no: alarm_no,
            client: OptionalCell::empty(),
        }
    }

    pub fn interrupt_no(&self) -> u32 {
        IRQ_NOS[self.no as usize]
    }

    pub fn handle_interrupt(&self) {
        self.client.map(|c| c.alarm());
        let r = REGISTERS.intr.get();
        REGISTERS.intr.set(r ^ (1 << self.no));
    }

    fn enable_interrupt(&self) {
        let r = REGISTERS.inte.get();
        REGISTERS.inte.set(r | (1 << self.no));
        unsafe { cortexm0p::nvic::Nvic::new(IRQ_NOS[self.no as usize]) }.enable();
    }

    fn disable_interrupt(&self) {
        let r = REGISTERS.inte.get();
        REGISTERS.inte.set(r ^ (1 << self.no));
        unsafe { cortexm0p::nvic::Nvic::new(IRQ_NOS[self.no as usize]) }.disable();
    }
}

impl <'a> hil::time::Time for Alarm<'a> {
    type Frequency = hil::time::Freq1MHz;
    type Ticks = hil::time::Ticks32;

    fn now(&self) -> Self::Ticks {
        Self::Ticks::from(REGISTERS.timerawl.get())
    }
}

impl<'a> hil::time::Alarm<'a> for Alarm<'a> {
    fn set_alarm_client(&self, client: &'a dyn hil::time::AlarmClient) {
        self.client.set(client)
    }

    fn set_alarm(&self, reference: Self::Ticks, dt: Self::Ticks) {
        let mut expire = reference.wrapping_add(dt);
        let now = Self::Ticks::from(REGISTERS.timerawl.get());
        if !now.within_range(reference, expire) {
            expire = now;
        }

        if expire.wrapping_sub(now) < self.minimum_dt() {
            expire = now.wrapping_add(self.minimum_dt());
        }

        REGISTERS.alarm[self.no as usize].set(expire.into_u32());
        self.enable_interrupt();
    }

    fn get_alarm(&self) -> Self::Ticks {
        Self::Ticks::from(REGISTERS.alarm[self.no as usize].get())
    }

    fn disarm(&self) -> Result<(), ErrorCode> {
        unsafe {
            atomic(|| {
                cortexm0p::nvic::Nvic::new(IRQ_NOS[self.no as usize])
                    .clear_pending();
                let r = REGISTERS.armed.get();
                REGISTERS.armed.set(r ^ (1 << self.no));
            })
        }
        self.disable_interrupt();

        Ok(())
    }

    fn is_armed(&self) -> bool {
        REGISTERS.armed.get() & (1 << self.no) != 0
    }

    fn minimum_dt(&self) -> Self::Ticks {
        Self::Ticks::from(50)
    }
}

pub struct RPTimer<'a> {
    alarms: [Alarm<'a>; 4],
    allocated: Cell<u8>,
}

impl<'a> RPTimer<'a> {
    pub const fn new() -> RPTimer<'a> {
        RPTimer {
            alarms: [
                Alarm::new(0),
                Alarm::new(1),
                Alarm::new(2),
                Alarm::new(3),
            ],
            allocated: Cell::new(0),
        }
    }

    pub fn handle_interrupt(&self) {
        let ints = REGISTERS.ints.get();
        for idx in 0..4 {
            if ints & (1 << idx) != 0 {
                self.alarms[idx].handle_interrupt();
            }
        }
    }

    pub fn allocate_alarm(&self) -> Result<&Alarm<'a>, ErrorCode> {
        for i in 0..self.alarms.len() {
            if self.allocated.get() & (1 << i) == 0 {
                self.allocated.set(self.allocated.get() | (1 << i));
                return Ok(&self.alarms[i]);
            }
        }

        Err(ErrorCode::BUSY)
    }
}

impl Time for RPTimer<'_> {
    type Frequency = hil::time::Freq1MHz;
    type Ticks = Ticks32;

    fn now(&self) -> Self::Ticks {
        Self::Ticks::from(REGISTERS.timerawl.get())
    }
}
