//! Integrity-protected framing for the binary profile-v2 UART training stream.
//!
//! Each protocol record is followed by a big-endian CRC-32/ISO-HDLC value
//! (polynomial `0x04C11DB7`, reflected implementation `0xEDB88320`, init and
//! xorout `0xffffffff`), then COBS encoded and terminated by `0x00`.

pub const MAX_PROTOCOL_FRAME_LEN: usize = 231;
const CRC_LEN: usize = 4;
/// 231 protocol bytes + CRC + worst-case COBS overhead + zero delimiter.
pub const MAX_COBS_FRAME_LEN: usize = 238;

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum Error {
    InputTooLarge,
    OutputTooSmall,
    #[cfg(test)]
    RecordTooShort,
    #[cfg(test)]
    CrcMismatch,
}

/// Encode one record and append a zero frame delimiter.
///
/// The returned length includes the trailing delimiter. The input may contain
/// arbitrary zero bytes and never allocates.
pub fn encode_delimited(input: &[u8], output: &mut [u8]) -> Result<usize, Error> {
    if input.len() > MAX_PROTOCOL_FRAME_LEN {
        return Err(Error::InputTooLarge);
    }
    let mut protected = [0_u8; MAX_PROTOCOL_FRAME_LEN + CRC_LEN];
    protected[..input.len()].copy_from_slice(input);
    protected[input.len()..input.len() + CRC_LEN].copy_from_slice(&crc32(input).to_be_bytes());
    cobs_encode_delimited(&protected[..input.len() + CRC_LEN], output)
}

/// Verify a decoded UART record and return only its protocol bytes.
///
/// COBS decoding and delimiter removal happen receiver-side before this call.
#[cfg(test)]
pub fn verify_decoded_record(record: &[u8]) -> Result<&[u8], Error> {
    let payload_len = record
        .len()
        .checked_sub(CRC_LEN)
        .ok_or(Error::RecordTooShort)?;
    let payload = &record[..payload_len];
    let expected = u32::from_be_bytes(
        record[payload_len..]
            .try_into()
            .map_err(|_| Error::RecordTooShort)?,
    );
    if crc32(payload) != expected {
        return Err(Error::CrcMismatch);
    }
    Ok(payload)
}

/// CRC-32/ISO-HDLC (`CRC-32`, Ethernet/ZIP) in a table-free implementation.
#[must_use]
pub fn crc32(bytes: &[u8]) -> u32 {
    let mut crc = 0xffff_ffff_u32;
    for byte in bytes {
        crc ^= u32::from(*byte);
        for _ in 0..8 {
            let mask = 0_u32.wrapping_sub(crc & 1);
            crc = (crc >> 1) ^ (0xedb8_8320 & mask);
        }
    }
    !crc
}

fn cobs_encode_delimited(input: &[u8], output: &mut [u8]) -> Result<usize, Error> {
    if output.is_empty() {
        return Err(Error::OutputTooSmall);
    }

    let mut read_index = 0;
    let mut write_index = 1;
    let mut code_index = 0;
    let mut code = 1_u8;

    while read_index < input.len() {
        let byte = input[read_index];
        read_index += 1;

        if byte == 0 {
            if code_index >= output.len() || write_index >= output.len() {
                return Err(Error::OutputTooSmall);
            }
            output[code_index] = code;
            code_index = write_index;
            write_index += 1;
            code = 1;
        } else {
            if write_index >= output.len() {
                return Err(Error::OutputTooSmall);
            }
            output[write_index] = byte;
            write_index += 1;
            code = code.saturating_add(1);

            if code == u8::MAX {
                if code_index >= output.len() || write_index >= output.len() {
                    return Err(Error::OutputTooSmall);
                }
                output[code_index] = code;
                code_index = write_index;
                write_index += 1;
                code = 1;
            }
        }
    }

    if code_index >= output.len() || write_index >= output.len() {
        return Err(Error::OutputTooSmall);
    }
    output[code_index] = code;
    output[write_index] = 0;
    Ok(write_index + 1)
}

#[cfg(test)]
mod tests {
    use super::*;

    fn decode_cobs(encoded: &[u8], output: &mut [u8]) -> usize {
        assert_eq!(encoded.last(), Some(&0));
        let mut read = 0;
        let mut write = 0;
        let end = encoded.len() - 1;
        while read < end {
            let code = usize::from(encoded[read]);
            assert_ne!(code, 0);
            read += 1;
            let data_len = code - 1;
            output[write..write + data_len].copy_from_slice(&encoded[read..read + data_len]);
            read += data_len;
            write += data_len;
            if code != 0xff && read < end {
                output[write] = 0;
                write += 1;
            }
        }
        write
    }

    #[test]
    fn arbitrary_protocol_bytes_round_trip() {
        let input = [0, 1, 2, 0, 0, 3, 0xff, 4, 0];
        let mut encoded = [0_u8; 32];
        let len = encode_delimited(&input, &mut encoded).unwrap();
        let mut decoded = [0_u8; 32];
        let decoded_len = decode_cobs(&encoded[..len], &mut decoded);
        assert_eq!(
            verify_decoded_record(&decoded[..decoded_len]).unwrap(),
            &input
        );
    }

    #[test]
    fn maximum_protocol_frame_fits_fixed_buffer() {
        let input = [0x55_u8; MAX_PROTOCOL_FRAME_LEN];
        let mut encoded = [0_u8; MAX_COBS_FRAME_LEN];
        let len = encode_delimited(&input, &mut encoded).unwrap();
        let mut decoded = [0_u8; MAX_PROTOCOL_FRAME_LEN + CRC_LEN];
        let decoded_len = decode_cobs(&encoded[..len], &mut decoded);
        assert_eq!(decoded_len, input.len() + CRC_LEN);
        assert_eq!(
            verify_decoded_record(&decoded[..decoded_len]).unwrap(),
            input
        );
    }

    #[test]
    fn too_small_output_returns_error_without_panicking() {
        let mut output = [0_u8; 2];
        assert_eq!(
            encode_delimited(&[1, 2], &mut output),
            Err(Error::OutputTooSmall)
        );
    }

    #[test]
    fn crc_has_standard_golden_value_and_big_endian_wire_order() {
        let input = b"123456789";
        assert_eq!(crc32(input), 0xcbf4_3926);

        let mut encoded = [0_u8; 32];
        let len = encode_delimited(input, &mut encoded).unwrap();
        assert_eq!(
            &encoded[..len],
            &[
                0x0e, b'1', b'2', b'3', b'4', b'5', b'6', b'7', b'8', b'9', 0xcb, 0xf4, 0x39, 0x26,
                0x00
            ]
        );
    }

    #[test]
    fn one_corrupted_payload_byte_is_rejected() {
        let input = b"wildfire-training-record";
        let mut encoded = [0_u8; 64];
        let len = encode_delimited(input, &mut encoded).unwrap();
        let mut decoded = [0_u8; 64];
        let decoded_len = decode_cobs(&encoded[..len], &mut decoded);
        decoded[3] ^= 0x80;
        assert_eq!(
            verify_decoded_record(&decoded[..decoded_len]),
            Err(Error::CrcMismatch)
        );
    }
}
