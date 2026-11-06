//! UDP endpoints that mirror RflySim3D: window inbox 20010+N and multicast 224.0.0.10:20009.

use rfly_proto::*;
use socket2::{Domain, Protocol, Socket, Type};
use std::net::{Ipv4Addr, SocketAddr, SocketAddrV4, UdpSocket};
use std::sync::mpsc::Sender;

pub type Inbox = (Ue4In, SocketAddr);

fn bind(port: u16, reuse: bool) -> std::io::Result<UdpSocket> {
    let s = Socket::new(Domain::IPV4, Type::DGRAM, Some(Protocol::UDP))?;
    if reuse {
        s.set_reuse_address(true)?;
        s.set_reuse_port(true)?;
    }
    let _ = s.set_recv_buffer_size(4 << 20);
    s.bind(&SocketAddrV4::new(Ipv4Addr::UNSPECIFIED, port).into())?;
    Ok(s.into())
}

/// Binds the window inbox. With `window = None`, takes the first free of 20010..20029,
/// like RflySim3D does when several windows are open. Returns (window id, socket).
pub fn bind_window(window: Option<u16>) -> std::io::Result<(u16, UdpSocket)> {
    if let Some(w) = window {
        return Ok((w, bind(UE4_BASE_PORT + w, false)?));
    }
    let mut last = None;
    for w in 0..20 {
        match bind(UE4_BASE_PORT + w, false) {
            Ok(s) => return Ok((w, s)),
            Err(e) => last = Some(e),
        }
    }
    Err(last.unwrap())
}

fn pump(sock: UdpSocket, tx: Sender<Inbox>) {
    let mut buf = vec![0u8; 65536];
    loop {
        match sock.recv_from(&mut buf) {
            Ok((n, from)) => {
                if let Some(m) = decode_ue4(&buf[..n]) {
                    if tx.send((m, from)).is_err() {
                        return;
                    }
                }
            }
            Err(_) => std::thread::sleep(std::time::Duration::from_millis(5)),
        }
    }
}

pub fn spawn(window_sock: UdpSocket, tx: Sender<Inbox>) {
    let tx2 = tx.clone();
    std::thread::spawn(move || pump(window_sock, tx));
    match bind(UE4_MCAST_PORT, true).and_then(|s| {
        s.join_multicast_v4(&Ipv4Addr::from(UE4_MCAST_GROUP), &Ipv4Addr::UNSPECIFIED)?;
        Ok(s)
    }) {
        Ok(s) => {
            std::thread::spawn(move || pump(s, tx2));
        }
        Err(e) => eprintln!("rflysim3d: multicast {UE4_MCAST_PORT} unavailable: {e}"),
    }
}

/// Socket for everything RflySim3D sends (replies, crash notices, images).
///
/// macOS refuses datagrams larger than the socket send buffer (9216 B by default), and RflySim
/// image packets are up to 60 KB, so the buffer is raised here.
pub fn sender() -> UdpSocket {
    let s = Socket::new(Domain::IPV4, Type::DGRAM, Some(Protocol::UDP)).expect("udp send socket");
    let _ = s.set_send_buffer_size(4 << 20);
    s.bind(&SocketAddrV4::new(Ipv4Addr::UNSPECIFIED, 0).into()).expect("udp bind");
    let s: UdpSocket = s.into();
    let _ = s.set_multicast_loop_v4(true);
    s
}
