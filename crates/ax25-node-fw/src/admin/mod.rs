//! Administration interfaces (the web panel lives in `crate::ota` / `crate::webui`):
//! the telnet console, and the relay that lets a telnet user connect onward
//! over the air through the node task.

pub mod relay;
pub mod telnet;
