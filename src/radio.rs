use oort_api::prelude::*;

/// A radio setup that provides deterministic frequency hopping and HMAC-based
/// message authentication for [u8; 32] packets (with [u8; 30] payload + u16 HMAC).
#[derive(Clone, Copy, Debug)]
pub struct SecureRadio {
    /// The secret key shared between ships.
    pub secret: u32,
    /// The channel offset used to segment communications into separate tiers.
    pub channel_offset: usize,
}

impl SecureRadio {
    /// Creates a new `SecureRadio` configuration with the given secret and channel offset.
    pub fn new(secret: u32, channel_offset: usize) -> Self {
        SecureRadio {
            secret,
            channel_offset,
        }
    }

    /// Picks the specified radio slot, authenticates the message, and transmits it.
    pub fn transmit(&self, radio_num: usize, payload: [u8; 30]) {
        select_radio(radio_num);
        let tick = current_tick();
        set_radio_channel(self.channel_for_tick(tick));
        let message = self.format_message(payload, tick);
        send_bytes(&message);
    }

    /// Tunes the radio slot to the correct channel for messages that will be sent this turn.
    pub fn prepare_receive(&self, radio_num: usize) {
        select_radio(radio_num);
        let tick = current_tick();
        set_radio_channel(self.channel_for_tick(tick));
    }

    /// Receives a message from the currently selected radio channel, authenticates it,
    /// and returns the 30-byte payload if valid.
    pub fn receive(&self) -> Option<[u8; 30]> {
        if let Some(message) = receive_bytes() {
            let tick = current_tick();
            self.parse_message(message, tick.saturating_sub(1))
        } else {
            None
        }
    }

    /// Determines the channel frequency (0 to 9) for a given turn/tick.
    fn channel_for_tick(&self, tick: u32) -> usize {
        let mut hash = 0xcbf29ce484222325u64;
        const FNV_PRIME: u64 = 0x00000100000001B3;

        for byte in self.secret.to_le_bytes() {
            hash ^= byte as u64;
            hash = hash.wrapping_mul(FNV_PRIME);
        }

        for byte in tick.to_le_bytes() {
            hash ^= byte as u64;
            hash = hash.wrapping_mul(FNV_PRIME);
        }

        let base_channel = (hash % 10) as usize;
        (base_channel + self.channel_offset) % 10
    }

    /// Computes a 16-bit HMAC based on the secret, the time sent (tick), and the message payload itself.
    fn compute_hmac(secret: u32, time_sent: u32, payload: &[u8; 30]) -> u16 {
        let mut hash = 0xcbf29ce484222325u64;
        const FNV_PRIME: u64 = 0x00000100000001B3;

        // Hash secret
        for byte in secret.to_le_bytes() {
            hash ^= byte as u64;
            hash = hash.wrapping_mul(FNV_PRIME);
        }

        // Hash time_sent
        for byte in time_sent.to_le_bytes() {
            hash ^= byte as u64;
            hash = hash.wrapping_mul(FNV_PRIME);
        }

        // Hash payload
        for &byte in payload {
            hash ^= byte as u64;
            hash = hash.wrapping_mul(FNV_PRIME);
        }

        // Fold 64-bit hash into a 16-bit value
        (hash ^ (hash >> 16) ^ (hash >> 32) ^ (hash >> 48)) as u16
    }

    /// Formats a 30-byte payload into a signed 32-byte message containing the HMAC in the last 2 bytes.
    fn format_message(&self, payload: [u8; 30], time_sent: u32) -> [u8; 32] {
        let hmac = Self::compute_hmac(self.secret, time_sent, &payload);
        let mut message = [0u8; 32];
        message[..30].copy_from_slice(&payload);
        message[30..32].copy_from_slice(&hmac.to_le_bytes());
        message
    }

    /// Verifies the HMAC of a received message and returns the 30-byte payload if authentic.
    fn parse_message(&self, message: [u8; 32], time_sent: u32) -> Option<[u8; 30]> {
        let mut payload = [0u8; 30];
        payload.copy_from_slice(&message[..30]);

        let mut hmac_bytes = [0u8; 2];
        hmac_bytes.copy_from_slice(&message[30..32]);
        let received_hmac = u16::from_le_bytes(hmac_bytes);

        let expected_hmac = Self::compute_hmac(self.secret, time_sent, &payload);
        if received_hmac == expected_hmac {
            Some(payload)
        } else {
            None
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_channel_for_tick_in_range() {
        let radio = SecureRadio::new(12345, 0);
        for tick in 0..1000 {
            let channel = radio.channel_for_tick(tick);
            assert!(channel < 10, "Channel {} should be in 0..10", channel);
        }
    }

    #[test]
    fn test_frequency_hopping_determinism() {
        let secret = 424242;
        let radio1 = SecureRadio::new(secret, 0);
        let radio2 = SecureRadio::new(secret, 0);

        for tick in 0..100 {
            assert_eq!(
                radio1.channel_for_tick(tick),
                radio2.channel_for_tick(tick),
                "Same secret and tick must produce same channel"
            );
        }
    }

    #[test]
    fn test_frequency_hopping_variance() {
        let radio = SecureRadio::new(1337, 0);
        let mut different_channels = false;
        let first_channel = radio.channel_for_tick(0);
        for tick in 1..20 {
            if radio.channel_for_tick(tick) != first_channel {
                different_channels = true;
                break;
            }
        }
        assert!(different_channels, "Channels should vary over ticks");
    }

    #[test]
    fn test_channel_offset() {
        let secret = 99999;
        let radio0 = SecureRadio::new(secret, 0);
        let radio3 = SecureRadio::new(secret, 3);

        for tick in 0..100 {
            let ch0 = radio0.channel_for_tick(tick);
            let ch3 = radio3.channel_for_tick(tick);
            assert_eq!(ch3, (ch0 + 3) % 10);
        }
    }

    #[test]
    fn test_hmac_sign_and_verify_success() {
        let radio = SecureRadio::new(9876543, 0);
        let payload = [7u8; 30];
        let tick = 120;

        let message = radio.format_message(payload, tick);
        let parsed = radio.parse_message(message, tick);

        assert_eq!(parsed, Some(payload));
    }

    #[test]
    fn test_hmac_verify_fails_with_wrong_secret() {
        let radio_sender = SecureRadio::new(9876543, 0);
        let radio_receiver = SecureRadio::new(1111111, 0); // wrong secret
        let payload = [7u8; 30];
        let tick = 120;

        let message = radio_sender.format_message(payload, tick);
        let parsed = radio_receiver.parse_message(message, tick);

        assert_eq!(parsed, None);
    }

    #[test]
    fn test_hmac_verify_fails_with_wrong_tick() {
        let radio = SecureRadio::new(9876543, 0);
        let payload = [7u8; 30];
        let tick_sent = 120;
        let tick_received_wrong = 121;

        let message = radio.format_message(payload, tick_sent);
        let parsed = radio.parse_message(message, tick_received_wrong);

        assert_eq!(parsed, None);
    }

    #[test]
    fn test_hmac_verify_fails_with_modified_payload() {
        let radio = SecureRadio::new(9876543, 0);
        let payload = [7u8; 30];
        let tick = 120;

        let mut message = radio.format_message(payload, tick);
        // Modify one byte in the payload
        message[5] ^= 0xFF;

        let parsed = radio.parse_message(message, tick);
        assert_eq!(parsed, None);
    }

    #[test]
    fn test_hmac_verify_fails_with_modified_hmac() {
        let radio = SecureRadio::new(9876543, 0);
        let payload = [7u8; 30];
        let tick = 120;

        let mut message = radio.format_message(payload, tick);
        // Modify the HMAC portion (last two bytes)
        message[31] ^= 0x01;

        let parsed = radio.parse_message(message, tick);
        assert_eq!(parsed, None);
    }
}
