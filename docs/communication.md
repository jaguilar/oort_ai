# Chapter 4: Inter-Ship Communications

Coordinating multi-ship fleets, sharing target vectors, and broadcasting status updates is handled by Oort's inter-ship radio communication networks. This chapter covers channel configurations, passing structured float arrays (`Message`), and serializing raw byte buffers.

For the corresponding checkable source code, see [communication.rs](file:///home/jaguilar/projects/oort_ai/docs/examples/communication.rs).

---

## 📡 Radio Infrastructure

Ships are equipped with **8 independent radio slots** (indexed from `0` to `7`). Each radio can be separately assigned to a specific channel (frequency).

### Radio Selection and Channels

Before sending or receiving data on a specific radio, you must select it:
* **`select_radio(index: usize)`**: Selects radio slot `0` to `7` for subsequent operations.
* **`set_radio_channel(channel: usize)`**: Routes the selected radio to a specific integer channel frequency.
* **`get_radio_channel() -> usize`**: Queries which channel the currently selected radio is tuned to.

> [!NOTE]
> Transmissions are **global and public within a channel**. Any entity (including enemies!) tuned to the same channel can receive messages broadcasted on that frequency. 

---

## 📝 Structured Float Messaging (`Message`)

The simplest way to communicate is by broadcasting fixed-size float arrays. 
* **`Message`** is a type alias for **`[f64; 4]`**.

### APIs

* **`send(msg: Message)`**
  * Broadcasts the four 64-bit floats on the selected radio's active channel.
  * **Timing:** Messages are processed at the end of the tick. They become available for receivers on the **next simulation tick**.
* **`receive() -> Option<Message>`**
  * Retrieves the standard float message received on the selected radio's channel from the previous tick.
  * Returns `None` if no entity transmitted on that channel during the previous frame.

---

## 💾 Binary Packet Messaging

If you need to serialize complex structs, strings, or pack multiple integers/compact data, you can use raw byte buffers.

### APIs

* **`send_bytes(msg: &[u8])`**
  * Broadcasts a raw byte packet on the active channel.
  * **Sizing:** The payload is automatically **zero-padded or truncated** to be exactly **`32` bytes** long.
* **`receive_bytes() -> Option<[u8; 32]>`**
  * Retrieves the raw `[u8; 32]` packet broadcasted on this channel during the previous frame.
  * Returns `None` if no byte packet was sent.

---

## 💻 Code Examples

Below is a compilable example showing how to tune radios, broadcast positional vectors as floats, and exchange serialized byte string headers.

```rust
use oort_api::prelude::*;

// Standard Float Message Coordination
pub fn show_float_messages() {
    // Select radio 0 and tune to channel 10
    select_radio(0);
    set_radio_channel(10);

    // Broadcast our position and heading to the fleet
    let pos = position();
    let head = heading();
    let fleet_msg: Message = [pos.x, pos.y, head, 0.0];
    
    send(fleet_msg);

    // Read incoming coordinates from allies
    if let Some(msg) = receive() {
        let x = msg[0];
        let y = msg[1];
        let h = msg[2];
        debug!("Fleet member reported at ({}, {}) heading {}", x, y, h);
    }
}

// Binary Packet Coordination
pub fn show_byte_messages() {
    // Select radio 1 and tune to channel 99
    select_radio(1);
    set_radio_channel(99);

    // Broadcast a raw ASCII packet
    let payload = b"SECTOR_4_CLEAR";
    send_bytes(payload);

    // Receive raw packets
    if let Some(bytes) = receive_bytes() {
        // Parse up to the first null byte (0)
        let len = bytes.iter().position(|&b| b == 0).unwrap_or(bytes.len());
        if let Ok(msg_str) = std::str::from_utf8(&bytes[..len]) {
            debug!("Received state code: {}", msg_str);
        }
    }
}
```
