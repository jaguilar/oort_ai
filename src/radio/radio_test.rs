use super::*;

fn create_test_manager() -> Rc<RefCell<RadioManager>> {
    Rc::new(RefCell::new(RadioManager::new()))
}

#[test]
fn test_channel_for_tick_in_range() {
    let radio = SecureRadio::new(12345, 0, create_test_manager());
    for tick in 0..1000 {
        let channel = radio.channel_for_tick(tick);
        assert!(channel < 10, "Channel {} should be in 0..10", channel);
    }
}

#[test]
fn test_frequency_hopping_determinism() {
    let secret = 424242;
    let radio1 = SecureRadio::new(secret, 0, create_test_manager());
    let radio2 = SecureRadio::new(secret, 0, create_test_manager());

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
    let radio = SecureRadio::new(1337, 0, create_test_manager());
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
    let radio0 = SecureRadio::new(secret, 0, create_test_manager());
    let radio3 = SecureRadio::new(secret, 3, create_test_manager());

    for tick in 0..100 {
        let ch0 = radio0.channel_for_tick(tick);
        let ch3 = radio3.channel_for_tick(tick);
        assert_eq!(ch3, (ch0 + 3) % 10);
    }
}

#[test]
fn test_hmac_sign_and_verify_success() {
    let radio = SecureRadio::new(9876543, 0, create_test_manager());
    let payload = [7u8; 30];
    let tick = 120;

    let message = radio.format_message(payload, tick);
    let parsed = radio.parse_message(message, tick);

    assert_eq!(parsed, Some(payload));
}

#[test]
fn test_hmac_verify_fails_with_wrong_secret() {
    let radio_sender = SecureRadio::new(9876543, 0, create_test_manager());
    let radio_receiver = SecureRadio::new(1111111, 0, create_test_manager()); // wrong secret
    let payload = [7u8; 30];
    let tick = 120;

    let message = radio_sender.format_message(payload, tick);
    let parsed = radio_receiver.parse_message(message, tick);

    assert_eq!(parsed, None);
}

#[test]
fn test_hmac_verify_fails_with_wrong_tick() {
    let radio = SecureRadio::new(9876543, 0, create_test_manager());
    let payload = [7u8; 30];
    let tick_sent = 120;
    let tick_received_wrong = 121;

    let message = radio.format_message(payload, tick_sent);
    let parsed = radio.parse_message(message, tick_received_wrong);

    assert_eq!(parsed, None);
}

#[test]
fn test_hmac_verify_fails_with_modified_payload() {
    let radio = SecureRadio::new(9876543, 0, create_test_manager());
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
    let radio = SecureRadio::new(9876543, 0, create_test_manager());
    let payload = [7u8; 30];
    let tick = 120;

    let mut message = radio.format_message(payload, tick);
    // Modify the HMAC portion (last two bytes)
    message[31] ^= 0x01;

    let parsed = radio.parse_message(message, tick);
    assert_eq!(parsed, None);
}
