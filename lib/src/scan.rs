use core::fmt;
use core::ops::RangeInclusive;

/// Wavelength in picometers; one nm is 1000 pm.
pub type Picometers = i32;

pub const RANGE_SCAN: &[u8] = b"RANGE-SCAN";
pub const POINT_MEASURE: &[u8] = b"POINT-MEASURE";
pub const SCAN: &[u8] = b"SCAN";
pub const MIN_STEP: Picometers = 100;
pub const MAX_STEP: Picometers = 100_000;
pub const MAX_READS: i32 = 100;
pub const MAX_INTEGRATION_MS: i32 = 60_000;
const FRACTION_DIGITS: usize = 1;
const PICOMETERS_PER_FRACTION_UNIT: Picometers = 100;
const MAX_ARGS: usize = 6;
const MOVE_WORDS: [&[u8]; 2] = [b"GOTO", b"NM"];

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

/// The macros the console understands, named by their last word.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Macro {
    RangeScan,
    PointMeasure,
    Scan,
}

impl Macro {
    fn from_word(word: &[u8]) -> Option<Self> {
        match word {
            _ if word.eq_ignore_ascii_case(RANGE_SCAN) => Some(Macro::RangeScan),
            _ if word.eq_ignore_ascii_case(POINT_MEASURE) => Some(Macro::PointMeasure),
            _ if word.eq_ignore_ascii_case(SCAN) => Some(Macro::Scan),
            _ => None,
        }
    }

    fn arg_count(self) -> usize {
        match self {
            Macro::RangeScan => 6,
            Macro::PointMeasure => 3,
            Macro::Scan => 6,
        }
    }

    fn usage(self) -> &'static str {
        match self {
            Macro::RangeScan => {
                "usage: <grating> <start-nm> <end-nm> <step-nm> <pmt-volts> <reads> RANGE-SCAN"
            }
            Macro::PointMeasure => "usage: <grating> <nm> <reads> POINT-MEASURE",
            Macro::Scan => {
                "usage: <grating> <start-nm> <end-nm> <step-nm> <pmt-volts> <integration-ms> SCAN"
            }
        }
    }
}

/// How each step gets its value: several plain reads, or one read after an
/// integration of `ms` milliseconds.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Sampling {
    Reads(i32),
    Integrate { ms: i32 },
}

/// A validated measurement request; a point measurement has `start == end`.
/// `volts` is `None` when the PMT voltage is left as it is.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Plan {
    pub grating: i32,
    pub start: Picometers,
    pub end: Picometers,
    pub step: Picometers,
    pub volts: Option<i32>,
    pub sampling: Sampling,
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
    Wavelength,
    Start,
    End,
    Step,
    Volts,
    Reads,
    IntegrationTime,
}

impl Field {
    fn is_wavelength(self) -> bool {
        matches!(self, Field::Wavelength | Field::Start | Field::End | Field::Step)
    }
}

/// Why a macro request was refused.
#[derive(Debug, PartialEq, Eq)]
pub enum ScanError {
    NotAScan,
    Usage(Macro),
    BadNumber(Field),
    OutOfRange { field: Field, low: i32, high: i32 },
}

/// Displays picometers as nm with one decimal, the only form the devices accept.
pub struct Nm(pub Picometers);

/// Parses `<grating> <start-nm> <end-nm> <step-nm> <pmt-volts> <reads> RANGE-SCAN`,
/// `<grating> <start-nm> <end-nm> <step-nm> <pmt-volts> <integration-ms> SCAN`
/// or `<grating> <nm> <reads> POINT-MEASURE` and validates it against `limits`.
/// Commands ending in none of these words yield `ScanError::NotAScan`.
pub fn parse(command: &[u8], limits: &Limits<'_>) -> Result<Plan, ScanError> {
    let kind = tokens(command)
        .last()
        .and_then(Macro::from_word)
        .ok_or(ScanError::NotAScan)?;
    if tokens(command).count() != kind.arg_count() + 1 {
        return Err(ScanError::Usage(kind));
    }
    let mut args: [&[u8]; MAX_ARGS] = [&[][..]; MAX_ARGS];
    for (slot, token) in args.iter_mut().zip(tokens(command).take(kind.arg_count())) {
        *slot = token;
    }
    match kind {
        Macro::RangeScan => range_scan(args, limits),
        Macro::PointMeasure => point_measure(args, limits),
        Macro::Scan => integrated_scan(args, limits),
    }
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

/// The wavelength a SpectraDrive command moves to: the number in front of the
/// last GOTO or NM, if there is one.
pub fn target_wavelength(command: &[u8]) -> Option<Picometers> {
    let mut previous: Option<&[u8]> = None;
    let mut target = None;
    for token in tokens(command) {
        if MOVE_WORDS.iter().any(|word| token.eq_ignore_ascii_case(word)) {
            target = previous.and_then(parse_picometers).or(target);
        }
        previous = Some(token);
    }
    target
}

/// Returns the last whitespace-separated token of `output` as an integer.
pub fn parse_reading(output: &[u8]) -> Option<i32> {
    let last = tokens(output).last()?;
    core::str::from_utf8(last).ok()?.parse().ok()
}

fn tokens(text: &[u8]) -> impl Iterator<Item = &[u8]> {
    text.split(u8::is_ascii_whitespace).filter(|token| !token.is_empty())
}

fn range_scan(args: [&[u8]; MAX_ARGS], limits: &Limits<'_>) -> Result<Plan, ScanError> {
    let reads = within(integer(args[5], Field::Reads)?, Field::Reads, 1, MAX_READS)?;
    sweep(args, limits, Sampling::Reads(reads))
}

fn integrated_scan(args: [&[u8]; MAX_ARGS], limits: &Limits<'_>) -> Result<Plan, ScanError> {
    let ms = integer(args[5], Field::IntegrationTime)?;
    let ms = within(ms, Field::IntegrationTime, 1, MAX_INTEGRATION_MS)?;
    sweep(args, limits, Sampling::Integrate { ms })
}

fn sweep(args: [&[u8]; MAX_ARGS], limits: &Limits<'_>, sampling: Sampling) -> Result<Plan, ScanError> {
    let [grating, start, end, step, volts, _] = args;
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
    Ok(Plan { grating, start, end, step, volts: Some(volts), sampling })
}

fn point_measure(args: [&[u8]; MAX_ARGS], limits: &Limits<'_>) -> Result<Plan, ScanError> {
    let [grating, nm, reads, ..] = args;
    let grating = integer(grating, Field::Grating)?;
    let range = grating_range(grating, limits)?;
    let nm = within(wavelength(nm, Field::Wavelength)?, Field::Wavelength, range.low, range.high)?;
    let reads = within(integer(reads, Field::Reads)?, Field::Reads, 1, MAX_READS)?;
    let sampling = Sampling::Reads(reads);
    Ok(Plan { grating, start: nm, end: nm, step: MIN_STEP, volts: None, sampling })
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
            Field::Wavelength => "nm",
            Field::Start => "start-nm",
            Field::End => "end-nm",
            Field::Step => "step-nm",
            Field::Volts => "pmt-volts",
            Field::Reads => "reads",
            Field::IntegrationTime => "integration-ms",
        })
    }
}

impl fmt::Display for ScanError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            ScanError::NotAScan => f.write_str("not a scan"),
            ScanError::Usage(kind) => f.write_str(kind.usage()),
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
        write!(f, "grating {}, ", self.grating)?;
        match self.start == self.end {
            true => write!(f, "at {} nm", Nm(self.start))?,
            false => write!(f, "{}..{} nm, step {} nm", Nm(self.start), Nm(self.end), Nm(self.step))?,
        }
        match self.volts {
            Some(volts) => write!(f, ", {volts} V")?,
            None => f.write_str(", PMT voltage unchanged")?,
        }
        match self.sampling {
            Sampling::Reads(reads) => write!(f, ", {reads} reads per step"),
            Sampling::Integrate { ms } => write!(f, ", {ms} ms integration per step"),
        }
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
        Plan { grating: 1, start, end, step, volts: Some(500), sampling: Sampling::Reads(3) }
    }

    #[test]
    fn parse_cases() {
        let cases: &[(&str, &[u8], Result<Plan, ScanError>)] = &[
            ("not a scan", b"300.0 NM", Err(ScanError::NotAScan)),
            ("valid", b"1 300.0 400.0 0.5 500 3 RANGE-SCAN", Ok(plan(300_000, 400_000, 500))),
            ("integers as nm", b"1 300 400 1 500 3 range-scan", Ok(plan(300_000, 400_000, 1_000))),
            (
                "integrated scan",
                b"1 300.0 400.0 0.5 500 250 scan",
                Ok(Plan { sampling: Sampling::Integrate { ms: 250 }, ..plan(300_000, 400_000, 500) }),
            ),
            (
                "integration too long",
                b"1 300.0 400.0 0.5 500 90000 SCAN",
                Err(ScanError::OutOfRange { field: Field::IntegrationTime, low: 1, high: 60_000 }),
            ),
            (
                "integration as decimal",
                b"1 300.0 400.0 0.5 500 0.5 SCAN",
                Err(ScanError::BadNumber(Field::IntegrationTime)),
            ),
            (
                "scan missing argument",
                b"1 300.0 400.0 0.5 SCAN",
                Err(ScanError::Usage(Macro::Scan)),
            ),
            (
                "missing argument",
                b"1 300.0 400.0 0.5 500 RANGE-SCAN",
                Err(ScanError::Usage(Macro::RangeScan)),
            ),
            (
                "point measure",
                b"2 300.5 5 point-measure",
                Ok(Plan {
                    grating: 2,
                    start: 300_500,
                    end: 300_500,
                    step: 100,
                    volts: None,
                    sampling: Sampling::Reads(5),
                }),
            ),
            (
                "point measure with volts",
                b"1 300.5 500 5 POINT-MEASURE",
                Err(ScanError::Usage(Macro::PointMeasure)),
            ),
            (
                "point outside grating",
                b"2 150.0 5 POINT-MEASURE",
                Err(ScanError::OutOfRange { field: Field::Wavelength, low: 200_000, high: 1_400_000 }),
            ),
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
            ("point", Plan { volts: None, ..plan(300_500, 300_500, 100) }, &[300_500]),
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
    fn target_wavelength_cases() {
        let cases: &[(&str, &[u8], Option<Picometers>)] = &[
            ("goto", b"300.5 GOTO", Some(300_500)),
            ("nm lower case", b"666.0 nm", Some(666_000)),
            ("query", b"?NM", None),
            ("grating", b"1 GRATING", None),
            ("last move wins", b"300.0 GOTO 400.0 NM", Some(400_000)),
            ("word without number", b"GOTO", None),
            ("empty", b"", None),
        ];
        for (name, command, expected) in cases {
            assert_eq!(&target_wavelength(command), expected, "{name}");
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
