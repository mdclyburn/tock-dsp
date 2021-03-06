//! RTC instantiation.

use kernel::utilities::StaticRef;
use sifive::rtc::RtcRegisters;

pub const RTC_BASE: StaticRef<RtcRegisters> =
    unsafe { StaticRef::new(0x1000_0040 as *const RtcRegisters) };
