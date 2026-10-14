use core::fmt::{self, Write as _};

/// Bytes per xxd row.
pub const WIDTH: usize = 16;

/// True for bytes xxd shows as themselves in its text column.
pub fn is_printable(byte: &u8) -> bool {
    (0x20..=0x7e).contains(byte)
}

/// One row in `xxd` layout: offset, hex pairs grouped by two bytes, text column.
pub struct XxdRow<'a> {
    offset: usize,
    bytes: &'a [u8],
}

impl<'a> XxdRow<'a> {
    /// Bytes beyond `WIDTH` are ignored.
    pub fn new(offset: usize, bytes: &'a [u8]) -> Self {
        Self { offset, bytes: &bytes[..bytes.len().min(WIDTH)] }
    }
}

impl fmt::Display for XxdRow<'_> {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(f, "{:08x}:", self.offset)?;
        for index in 0..WIDTH {
            if index % 2 == 0 {
                f.write_char(' ')?;
            }
            match self.bytes.get(index) {
                Some(byte) => write!(f, "{byte:02x}")?,
                None => f.write_str("  ")?,
            }
        }
        f.write_str("  ")?;
        for byte in self.bytes {
            f.write_char(if is_printable(byte) { char::from(*byte) } else { '.' })?;
        }
        Ok(())
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    struct Buffer {
        bytes: [u8; 128],
        len: usize,
    }

    impl fmt::Write for Buffer {
        fn write_str(&mut self, text: &str) -> fmt::Result {
            let end = self.len + text.len();
            self.bytes.get_mut(self.len..end).ok_or(fmt::Error)?.copy_from_slice(text.as_bytes());
            self.len = end;
            Ok(())
        }
    }

    fn render(row: XxdRow<'_>) -> Buffer {
        let mut buffer = Buffer { bytes: [0; 128], len: 0 };
        write!(buffer, "{row}").expect("row fits test buffer");
        buffer
    }

    #[test]
    fn row_cases() {
        let cases: &[(&str, usize, &[u8], &str)] = &[
            (
                "full row",
                0,
                b"?GROOVES 6\xe5\xff\x67\xa0\x00\x81",
                "00000000: 3f47 524f 4f56 4553 2036 e5ff 67a0 0081  ?GROOVES 6..g...",
            ),
            (
                "short row with offset",
                0x10,
                b"hello\n",
                "00000010: 6865 6c6c 6f0a                           hello.",
            ),
            (
                "odd length",
                0x20,
                b"abc",
                "00000020: 6162 63                                  abc",
            ),
            (
                "excess bytes ignored",
                0,
                b"0123456789abcdefXYZ",
                "00000000: 3031 3233 3435 3637 3839 6162 6364 6566  0123456789abcdef",
            ),
        ];
        for &(name, offset, bytes, expected) in cases {
            let buffer = render(XxdRow::new(offset, bytes));
            assert_eq!(core::str::from_utf8(&buffer.bytes[..buffer.len]), Ok(expected), "{name}");
        }
    }
}
