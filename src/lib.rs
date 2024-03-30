#![no_std]

mod bus;
mod control_pipe;
mod driver;
mod endpoint;
mod interrupt;
mod phy;
pub mod regs;
mod state;

pub use crate::bus::Bus;
pub use crate::control_pipe::ControlPipe;
pub use crate::driver::Driver;

#[derive(Copy, Clone, Debug, Eq, PartialEq)]
pub enum PhyType {
    Internal,
    External,
}

#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub struct Config {
    vbus_detection_pin: Option<()>, // TODO: should be an optional GPIO
    phy_type: PhyType,
}

impl Default for Config {
    fn default() -> Self {
        Self {
            vbus_detection_pin: None,
            phy_type: PhyType::Internal,
        }
    }
}

pub enum In {}
pub enum Out {}
