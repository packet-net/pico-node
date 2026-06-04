//! The transport-agnostic connection abstraction. Analogue of
//! `Packet.Node.Core.Console.INodeConnection`.
//!
//! A bidirectional byte stream to a connected user plus the metadata the console
//! needs, independent of how the user reached us. The prompt loop runs over this
//! trait only, so the command logic never depends on AX.25 vs telnet.
//!
//! This uses async fns in traits (stable since Rust 1.75), so it is usable from
//! the Embassy executor on the firmware without an extra async-trait crate. The
//! firmware provides two implementations: a telnet TCP adapter and an AX.25
//! session adapter — exactly mirroring the two C# implementations.

use core::future::Future;

/// The transport a [`NodeConnection`] arrived on. Mirrors `NodeTransportKind`.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum TransportKind {
    /// An over-the-air AX.25 connected-mode session.
    Ax25,
    /// A local telnet dial-in over TCP.
    Telnet,
}

impl TransportKind {
    /// The line terminator convention for this transport: CR-LF for telnet, a
    /// bare CR for AX.25 (the packet-radio convention). Mirrors
    /// `NodeCommandService.NewLine`.
    pub const fn newline(self) -> &'static [u8] {
        match self {
            TransportKind::Telnet => b"\r\n",
            TransportKind::Ax25 => b"\r",
        }
    }
}

/// A bidirectional byte stream to a connected user.
///
/// `read` returns an empty slice on EOF (peer gone). Errors are the
/// implementation's associated [`NodeConnection::Error`]; a normal close is *not*
/// an error (it's an empty read).
pub trait NodeConnection {
    /// Transport-specific error type for read/write failures.
    type Error;

    /// Which transport this connection arrived on.
    fn transport_kind(&self) -> TransportKind;

    /// Read the next chunk of inbound bytes into `buf`; returns the number of
    /// bytes read (0 == EOF / peer gone).
    fn read<'a>(
        &'a mut self,
        buf: &'a mut [u8],
    ) -> impl Future<Output = Result<usize, Self::Error>> + 'a;

    /// Send all of `bytes` to the peer. Framing/segmentation is the
    /// implementation's concern (the AX.25 adapter routes through the
    /// segmentation-aware send).
    fn write<'a>(
        &'a mut self,
        bytes: &'a [u8],
    ) -> impl Future<Output = Result<(), Self::Error>> + 'a;
}
