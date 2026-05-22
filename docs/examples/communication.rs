//! Compilable examples for the Oort inter-ship Radio and Communication APIs.
//! Covers selecting radios, setting channels, float-based `Message` passing, and raw byte transfers.

use oort_api::prelude::*;

/// Example of selecting a radio and setting up channel routing.
pub fn show_radio_setup() {
    // Ships have 8 independent radio slots: 0 to 7
    let radio_slot = 0;
    select_radio(radio_slot);

    // Set the communication channel (e.g. channel 42)
    // Transmissions sent on a channel are received by any ship listening on that same channel
    let comm_channel = 42;
    set_radio_channel(comm_channel);

    // Retrieve the current channel for the selected radio slot
    let active_channel: usize = get_radio_channel();

    debug!("Radio slot {} configured on channel {}", radio_slot, active_channel);
}

/// Example of sending and receiving standard float messages (`Message` = `[f64; 4]`).
pub fn show_float_messages() {
    select_radio(0);
    set_radio_channel(1);

    // 1. Sending a standard message
    // A Message is a fixed-size array of four 64-bit floats: [f64; 4]
    // Here, we'll encode our ship position and heading
    let my_pos = position();
    let my_head = heading();
    
    let msg_to_send: Message = [my_pos.x, my_pos.y, my_head, 0.0];
    send(msg_to_send);
    debug!("Sent message: {:?}", msg_to_send);

    // 2. Receiving messages
    // receive() retrieves the message sent on the selected channel in the previous tick
    // It returns None if no message was transmitted on this channel
    let received: Option<Message> = receive();
    if let Some(msg) = received {
        let x = msg[0];
        let y = msg[1];
        let h = msg[2];
        debug!("Received coordinate: ({}, {}) with heading {}", x, y, h);
    }
}

/// Example of sending and receiving raw bytes for custom serialization.
pub fn show_byte_messages() {
    select_radio(1);
    set_radio_channel(100);

    // 1. Sending raw byte packets
    // send_bytes() takes a byte slice and pads/truncates it to a fixed length of 32 bytes
    let raw_data = b"OORT PROTOCOL V1";
    send_bytes(raw_data);
    debug!("Sent byte message");

    // 2. Receiving raw byte packets
    // receive_bytes() returns a fixed [u8; 32] array from the previous tick if a message exists
    let received: Option<[u8; 32]> = receive_bytes();
    if let Some(bytes) = received {
        // Find the length of the string up to the first null byte, or just use all bytes
        let valid_len = bytes.iter().position(|&b| b == 0).unwrap_or(bytes.len());
        if let Ok(text) = std::str::from_utf8(&bytes[..valid_len]) {
            debug!("Received string: {}", text);
        } else {
            debug!("Received raw bytes: {:?}", bytes);
        }
    }
}
