use core::fmt::{self, Write as _};

const MAX_ARGS: usize = 4;

/// Kind of numeric argument a device word consumes.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Arg {
    Integer,
    Decimal,
}

/// A device word and the arguments it takes, in stack order.
#[derive(Clone, Copy, Debug)]
pub struct Word {
    pub name: &'static [u8],
    pub args: &'static [Arg],
}

impl Word {
    pub const fn new(name: &'static [u8], args: &'static [Arg]) -> Self {
        Self { name, args }
    }
}

/// Why a command was refused.
#[derive(Debug, PartialEq, Eq)]
pub enum Rejection<'a> {
    UnknownWord(&'a [u8]),
    MalformedNumber(&'a [u8]),
    WrongArguments { word: &'a [u8], expected: &'static [Arg] },
    TooManyArguments,
    DanglingArguments,
}

/// Checks that every word in `command` is listed in `words` and is preceded by
/// exactly the numbers it expects. Numbers no word consumes are refused, and so
/// are decimals with anything but exactly one fraction digit.
pub fn check<'a>(command: &'a [u8], words: &[Word]) -> Result<(), Rejection<'a>> {
    let mut pending = Pending::new();
    for token in tokens(command) {
        match number_kind(token) {
            Some(arg) => pending.push(arg)?,
            None if looks_numeric(token) => return Err(Rejection::MalformedNumber(token)),
            None => consume(token, words, &mut pending)?,
        }
    }
    pending.ensure_empty()
}

fn tokens(command: &[u8]) -> impl Iterator<Item = &[u8]> {
    command.split(u8::is_ascii_whitespace).filter(|token| !token.is_empty())
}

fn number_kind(token: &[u8]) -> Option<Arg> {
    let unsigned = token.strip_prefix(b"-").unwrap_or(token);
    match unsigned.iter().position(|&byte| byte == b'.') {
        None => all_digits(unsigned).then_some(Arg::Integer),
        Some(dot) => (all_digits(&unsigned[..dot]) && is_single_digit(&unsigned[dot + 1..]))
            .then_some(Arg::Decimal),
    }
}

fn looks_numeric(token: &[u8]) -> bool {
    let unsigned = token.strip_prefix(b"-").unwrap_or(token);
    unsigned.first().is_some_and(u8::is_ascii_digit)
}

fn is_single_digit(text: &[u8]) -> bool {
    matches!(text, [digit] if digit.is_ascii_digit())
}

fn all_digits(text: &[u8]) -> bool {
    !text.is_empty() && text.iter().all(u8::is_ascii_digit)
}

fn consume<'a>(token: &'a [u8], words: &[Word], pending: &mut Pending) -> Result<(), Rejection<'a>> {
    let word = lookup(token, words).ok_or(Rejection::UnknownWord(token))?;
    if !pending.matches(word.args) {
        return Err(Rejection::WrongArguments { word: token, expected: word.args });
    }
    pending.clear();
    Ok(())
}

fn lookup<'w>(token: &[u8], words: &'w [Word]) -> Option<&'w Word> {
    words.iter().find(|word| word.name.eq_ignore_ascii_case(token))
}

struct Pending {
    args: [Arg; MAX_ARGS],
    len: usize,
}

impl Pending {
    fn new() -> Self {
        Self { args: [Arg::Integer; MAX_ARGS], len: 0 }
    }

    fn push<'a>(&mut self, arg: Arg) -> Result<(), Rejection<'a>> {
        if self.len == MAX_ARGS {
            return Err(Rejection::TooManyArguments);
        }
        self.args[self.len] = arg;
        self.len += 1;
        Ok(())
    }

    fn matches(&self, expected: &[Arg]) -> bool {
        &self.args[..self.len] == expected
    }

    fn clear(&mut self) {
        self.len = 0;
    }

    fn ensure_empty<'a>(&self) -> Result<(), Rejection<'a>> {
        match self.len {
            0 => Ok(()),
            _ => Err(Rejection::DanglingArguments),
        }
    }
}

struct Ascii<'a>(&'a [u8]);

impl fmt::Display for Ascii<'_> {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        for &byte in self.0 {
            f.write_char(if byte.is_ascii_graphic() { byte as char } else { '.' })?;
        }
        Ok(())
    }
}

impl fmt::Display for Arg {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Arg::Integer => f.write_str("<int>"),
            Arg::Decimal => f.write_str("<dec>"),
        }
    }
}

impl fmt::Display for Word {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        for arg in self.args {
            write!(f, "{arg} ")?;
        }
        write!(f, "{}", Ascii(self.name))
    }
}

impl fmt::Display for Rejection<'_> {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Rejection::UnknownWord(word) => write!(f, "unknown word {}", Ascii(word)),
            Rejection::MalformedNumber(number) => {
                write!(f, "malformed number {}, decimals take exactly one digit after the point", Ascii(number))
            }
            Rejection::WrongArguments { word, expected } => {
                let usage = Word { name: b"", args: expected };
                write!(f, "usage: {usage}{}", Ascii(word))
            }
            Rejection::TooManyArguments => write!(f, "more than {MAX_ARGS} numbers in a row"),
            Rejection::DanglingArguments => f.write_str("numbers without a word to consume them"),
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    const WORDS: &[Word] = &[
        Word::new(b"NM", &[Arg::Decimal]),
        Word::new(b"?NM", &[]),
        Word::new(b"VOLTS", &[Arg::Integer]),
    ];

    #[test]
    fn check_cases() {
        let cases: &[(&str, &[u8], Result<(), Rejection<'static>>)] = &[
            ("ping", b"", Ok(())),
            ("query", b"?NM", Ok(())),
            ("decimal argument", b"300.0 NM", Ok(())),
            ("extra whitespace", b"  300.0   NM ", Ok(())),
            ("ignores case", b"300.0 nm", Ok(())),
            ("two commands", b"300.0 NM ?NM", Ok(())),
            ("negative integer", b"-5 VOLTS", Ok(())),
            ("integer for decimal", b"300 NM",
                Err(Rejection::WrongArguments { word: &b"NM"[..], expected: &[Arg::Decimal] })),
            ("decimal for integer", b"400.0 VOLTS",
                Err(Rejection::WrongArguments { word: &b"VOLTS"[..], expected: &[Arg::Integer] })),
            ("missing argument", b"NM",
                Err(Rejection::WrongArguments { word: &b"NM"[..], expected: &[Arg::Decimal] })),
            ("argument to query", b"1 ?NM",
                Err(Rejection::WrongArguments { word: &b"?NM"[..], expected: &[] })),
            ("dangling number", b"300.0", Err(Rejection::DanglingArguments)),
            ("dangling after word", b"?NM 5", Err(Rejection::DanglingArguments)),
            ("unknown word", b"FOO", Err(Rejection::UnknownWord(&b"FOO"[..]))),
            ("trailing dot", b"300. NM", Err(Rejection::MalformedNumber(&b"300."[..]))),
            ("two decimals", b"300.55 NM", Err(Rejection::MalformedNumber(&b"300.55"[..]))),
            ("padded decimals", b"300.500 GOTO NM", Err(Rejection::MalformedNumber(&b"300.500"[..]))),
            ("digits then letters", b"3x0 NM", Err(Rejection::MalformedNumber(&b"3x0"[..]))),
            ("leading dot is not a decimal", b".5 NM", Err(Rejection::UnknownWord(&b".5"[..]))),
            ("too many numbers", b"1 2 3 4 5 VOLTS", Err(Rejection::TooManyArguments)),
        ];
        for (name, command, expected) in cases {
            assert_eq!(&check(command, WORDS), expected, "{name}");
        }
    }
}
