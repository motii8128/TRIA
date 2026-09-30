use tria_message_rs::tria::{CommandPacket, SensorPacket};

use zenoh::{Config, Result};

use std::net::UdpSocket;

fn main() {
    let udp = UdpSocket::bind("addr").unwrap();
}
