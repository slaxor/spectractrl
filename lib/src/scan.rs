use core::fmt;
use core::ops::RangeInclusive;

/// Wavelength in picometers; one nm is 1000 pm.
pub type Picometers = i32;

pub const RANGE_SCAN: &[u8] = b"RANGE-SCAN";
pub const MIN_STEP: Picometers = 100;
pub const MAX_STEP: Picometers = 100_000;
pub const MAX_READS: i32 = 100;
const FRACTION_DIGITS: usize = 1;
const PICOMETERS_PER_FRACTION_UNIT: Picometers = 100;
const ARGS: usize = 6;

/// Inclusive wavelength range a grating can reach.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct NmRange {
    pub low: Picometers,
    pub high: Picometers,
}

impl NmRange {
    pub const fn new(low: Picometers, high: Picometers) -> Self {
        Self { low, high }
    }
}

/// Hardware limits a scan is validated against.
pub struct Limits<'a> {
    pub gratings: &'a [NmRange],
    pub volts: RangeInclusive<i32>,
}

/// A validated RANGE-SCAN request.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Plan {
    pub grating: i32,
    pub start: Picometers,
    pub end: Picometers,
    pub step: Picometers,
    pub volts: i32,
    pub reads: i32,
}

impl Plan {
    /// Wavelengths from `start` up to and including `end`, `step` apart.
    pub fn steps(&self) -> impl Iterator<Item = Picometers> + use<> {
        (self.start..=self.end).step_by(self.step as usize)
    }
}

/// Scan parameter, used to name the offending one in errors.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Field {
    Grating,
    Start,
    End,
    Step,
    Volts,
    Reads,
}

impl Field {
    fn is_wavelength(self) -> bool {
        matches!(self, Field::Start | Field::End | Field::Step)
    }
}

/// Why a RANGE-SCAN request was refused.
#[derive(Debug, PartialEq, Eq)]
pub enum ScanError {
    NotAScan,
    Usage,
    BadNumber(Field),
    OutOfRange { field: Field, low: i32, high: i32 },
}

/// Displays picometers as nm with one decimal, the only form the devices accept.
pub struct Nm(pub Picometers);

/// Parses `<grating> <start-nm> <end-nm> <step-nm> <pmt-volts> <reads> RANGE-SCAN`
/// and validates it against `limits`. Commands not ending in RANGE-SCAN yield
/// `ScanError::NotAScan`.
pub fn parse(command: &[u8], limits: &Limits<'_>) -> Result<Plan, ScanError> {
    let is_scan = tokens(command)
        .last()
        .is_some_and(|word| word.eq_ignore_ascii_case(RANGE_SCAN));
    if !is_scan {
        return Err(ScanError::NotAScan);
    }
    if tokens(command).count() != ARGS + 1 {
        return Err(ScanError::Usage);
    }
    let mut args: [&[u8]; ARGS] = [&[][..]; ARGS];
    for (slot, token) in args.iter_mut().zip(tokens(command)) {
        *slot = token;
    }
    validate(args, limits)
}

/// Parses a non-negative decimal with at most one fraction digit into picometers.
pub fn parse_picometers(token: &[u8]) -> Option<Picometers> {
    let (whole, fraction) = match token.iter().position(|&byte| byte == b'.') {
        Some(dot) => (&token[..dot], &token[dot + 1..]),
        None => (token, &b""[..]),
    };
    let valid = !whole.is_empty()
        && fraction.len() <= FRACTION_DIGITS
        && whole.iter().all(u8::is_ascii_digit)
        && fraction.iter().all(u8::is_ascii_digit);
    if !valid {
        return None;
    }
    let whole: i32 = core::str::from_utf8(whole).ok()?.parse().ok()?;
    let tenths = fraction.first().map_or(0, |digit| i32::from(digit - b'0'));
    whole.checked_mul(1000)?.checked_add(tenths * PICOMETERS_PER_FRACTION_UNIT)
}

/// Returns the last whitespace-separated token of `output` as an integer.
pub fn parse_reading(output: &[u8]) -> Option<i32> {
    let last = tokens(output).last()?;
    core::str::from_utf8(last).ok()?.parse().ok()
}

fn tokens(text: &[u8]) -> impl Iterator<Item = &[u8]> {
    text.split(u8::is_ascii_whitespace).filter(|token| !token.is_empty())
}

fn validate(args: [&[u8]; ARGS], limits: &Limits<'_>) -> Result<Plan, ScanError> {
    let [grating, start, end, step, volts, reads] = args;
    let grating = integer(grating, Field::Grating)?;
    let range = grating_range(grating, limits)?;
    let start = within(wavelength(start, Field::Start)?, Field::Start, range.low, range.high)?;
    let end = within(wavelength(end, Field::End)?, Field::End, start, range.high)?;
    let step = within(wavelength(step, Field::Step)?, Field::Step, MIN_STEP, MAX_STEP)?;
    let volts = within(
        integer(volts, Field::Volts)?,
        Field::Volts,
        *limits.volts.start(),
        *limits.volts.end(),
    )?;
    let reads = within(integer(reads, Field::Reads)?, Field::Reads, 1, MAX_READS)?;
    Ok(Plan { grating, start, end, step, volts, reads })
}

fn integer(token: &[u8], field: Field) -> Result<i32, ScanError> {
    core::str::from_utf8(token)
        .ok()
        .and_then(|text| text.parse().ok())
        .ok_or(ScanError::BadNumber(field))
}

fn wavelength(token: &[u8], field: Field) -> Result<Picometers, ScanError> {
    parse_picometers(token).ok_or(ScanError::BadNumber(field))
}

fn within(value: i32, field: Field, low: i32, high: i32) -> Result<i32, ScanError> {
    match (low..=high).contains(&value) {
        true => Ok(value),
        false => Err(ScanError::OutOfRange { field, low, high }),
    }
}

fn grating_range(grating: i32, limits: &Limits<'_>) -> Result<NmRange, ScanError> {
    let count = i32::try_from(limits.gratings.len()).unwrap_or(i32::MAX);
    let number = within(grating, Field::Grating, 1, count)?;
    Ok(limits.gratings[(number - 1) as usize])
}

impl fmt::Display for Nm {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(f, "{}.{}", self.0 / 1000, self.0 % 1000 / PICOMETERS_PER_FRACTION_UNIT)
    }
}

impl fmt::Display for Field {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.write_str(match self {
            Field::Grating => "grating",
            Field::Start => "start-nm",
            Field::End => "end-nm",
            Field::Step => "step-nm",
            Field::Volts => "pmt-volts",
            Field::Reads => "reads",
        })
    }
}

impl fmt::Display for ScanError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            ScanError::NotAScan => f.write_str("not a scan"),
            ScanError::Usage => f.write_str(
                "usage: <grating> <start-nm> <end-nm> <step-nm> <pmt-volts> <reads> RANGE-SCAN",
            ),
            ScanError::BadNumber(field) if field.is_wavelength() => {
                write!(f, "{field} must be a number with at most one decimal")
            }
            ScanError::BadNumber(field) => write!(f, "{field} is not a valid number"),
            ScanError::OutOfRange { field, low, high } if field.is_wavelength() => {
                write!(f, "{field} must be {}..{} nm", Nm(*low), Nm(*high))
            }
            ScanError::OutOfRange { field, low, high } => write!(f, "{field} must be {low}..{high}"),
        }
    }
}

impl fmt::Display for Plan {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(
            f,
            "grating {}, {}..{} nm, step {} nm, {} V, {} reads per step",
            self.grating,
            Nm(self.start),
            Nm(self.end),
            Nm(self.step),
            self.volts,
            self.reads,
        )
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use core::fmt::Write as _;

    const LIMITS: Limits<'static> = Limits {
        gratings: &[NmRange::new(0, 1_000_000), NmRange::new(200_000, 1_400_000)],
        volts: 0..=1000,
    };

    fn plan(start: i32, end: i32, step: i32) -> Plan {
        Plan { grating: 1, start, end, step, volts: 500, reads: 3 }
    }

    #[test]
    fn parse_cases() {
        let cases: &[(&str, &[u8], Result<Plan, ScanError>)] = &[
            ("not a scan", b"300.0 NM", Err(ScanError::NotAScan)),
            ("valid", b"1 300.0 400.0 0.5 500 3 RANGE-SCAN", Ok(plan(300_000, 400_000, 500))),
            ("integers as nm", b"1 300 400 1 500 3 range-scan", Ok(plan(300_000, 400_000, 1_000))),
            ("missing argument", b"1 300.0 400.0 0.5 500 RANGE-SCAN", Err(ScanError::Usage)),
            (
                "grating out of range",
                b"3 300.0 400.0 0.5 500 3 RANGE-SCAN",
                Err(ScanError::OutOfRange { field: Field::Grating, low: 1, high: 2 }),
            ),
            (
                "start below grating",
                b"2 100.0 400.0 0.5 500 3 RANGE-SCAN",
                Err(ScanError::OutOfRange { field: Field::Start, low: 200_000, high: 1_400_000 }),
            ),
            (
                "end before start",
                b"1 400.0 300.0 0.5 500 3 RANGE-SCAN",
                Err(ScanError::OutOfRange { field: Field::End, low: 400_000, high: 1_000_000 }),
            ),
            (
                "step zero",
                b"1 300.0 400.0 0.0 500 3 RANGE-SCAN",
                Err(ScanError::OutOfRange { field: Field::Step, low: 100, high: 100_000 }),
            ),
            (
                "two decimals",
                b"1 300.25 400.0 0.5 500 3 RANGE-SCAN",
                Err(ScanError::BadNumber(Field::Start)),
            ),
            (
                "volts out of range",
                b"1 300.0 400.0 0.5 2000 3 RANGE-SCAN",
                Err(ScanError::OutOfRange { field: Field::Volts, low: 0, high: 1000 }),
            ),
            (
                "zero reads",
                b"1 300.0 400.0 0.5 500 0 RANGE-SCAN",
                Err(ScanError::OutOfRange { field: Field::Reads, low: 1, high: 100 }),
            ),
        ];
        for (name, command, expected) in cases {
            assert_eq!(&parse(command, &LIMITS), expected, "{name}");
        }
    }

    #[test]
    fn steps_cases() {
        let cases: &[(&str, Plan, &[i32])] = &[
            ("single point", plan(300_000, 300_000, 500), &[300_000]),
            ("exact end", plan(300_000, 301_000, 500), &[300_000, 300_500, 301_000]),
            ("end not on grid", plan(300_000, 301_000, 400), &[300_000, 300_400, 300_800]),
        ];
        for (name, plan, expected) in cases {
            assert!(plan.steps().eq(expected.iter().copied()), "{name}");
        }
    }

    #[test]
    fn parse_picometers_cases() {
        let cases: &[(&str, &[u8], Option<i32>)] = &[
            ("integer", b"300", Some(300_000)),
            ("one decimal", b"300.5", Some(300_500)),
            ("smallest step", b"0.1", Some(100)),
            ("two decimals", b"0.25", None),
            ("trailing dot", b"300.", Some(300_000)),
            ("leading dot", b".5", None),
            ("padded decimals", b"300.500", None),
            ("negative", b"-1.0", None),
            ("garbage", b"3x0", None),
        ];
        for (name, token, expected) in cases {
            assert_eq!(&parse_picometers(token), expected, "{name}");
        }
    }

    #[test]
    fn nm_display_cases() {
        let cases: &[(&str, Picometers, &str)] = &[
            ("half nm", 300_500, "300.5"),
            ("whole nm", 666_000, "666.0"),
            ("below one", 100, "0.1"),
        ];
        for (name, picometers, expected) in cases {
            let mut buffer = [0u8; 16];
            let mut cursor = Cursor { bytes: &mut buffer, len: 0 };
            write!(cursor, "{}", Nm(*picometers)).expect("nm fits test buffer");
            let len = cursor.len;
            assert_eq!(core::str::from_utf8(&buffer[..len]), Ok(*expected), "{name}");
        }
    }

    struct Cursor<'a> {
        bytes: &'a mut [u8],
        len: usize,
    }

    impl fmt::Write for Cursor<'_> {
        fn write_str(&mut self, text: &str) -> fmt::Result {
            let end = self.len + text.len();
            self.bytes.get_mut(self.len..end).ok_or(fmt::Error)?.copy_from_slice(text.as_bytes());
            self.len = end;
            Ok(())
        }
    }

    #[test]
    fn parse_reading_cases() {
        let cases: &[(&str, &[u8], Option<i32>)] = &[
            ("echo and value", b"READ 368", Some(368)),
            ("negative", b"READ -12", Some(-12)),
            ("no number", b"READ", None),
            ("empty", b"", None),
        ];
        for (name, output, expected) in cases {
            assert_eq!(&parse_reading(output), expected, "{name}");
        }
    }
}
