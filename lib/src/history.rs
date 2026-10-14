/// Console command history, newest entry first, walked like bash's up/down keys.
/// Lines longer than `WIDTH` are truncated.
pub struct History<const ENTRIES: usize, const WIDTH: usize> {
    entries: [[u8; WIDTH]; ENTRIES],
    lens: [usize; ENTRIES],
    count: usize,
    cursor: Option<usize>,
}

impl<const ENTRIES: usize, const WIDTH: usize> History<ENTRIES, WIDTH> {
    pub const fn new() -> Self {
        Self { entries: [[0; WIDTH]; ENTRIES], lens: [0; ENTRIES], count: 0, cursor: None }
    }

    /// Records a submitted line and resets the walk. Empty lines and repeats of
    /// the newest entry are not recorded.
    pub fn push(&mut self, line: &[u8]) {
        self.cursor = None;
        if line.is_empty() || self.last() == Some(line) {
            return;
        }
        self.entries.copy_within(..ENTRIES - 1, 1);
        self.lens.copy_within(..ENTRIES - 1, 1);
        let len = line.len().min(WIDTH);
        self.entries[0][..len].copy_from_slice(&line[..len]);
        self.lens[0] = len;
        self.count = (self.count + 1).min(ENTRIES);
    }

    /// The most recently recorded line.
    pub fn last(&self) -> Option<&[u8]> {
        (self.count > 0).then(|| self.entry(0))
    }

    /// Steps one entry back in time; `None` when there is nothing older.
    pub fn older(&mut self) -> Option<&[u8]> {
        let next = self.cursor.map_or(0, |cursor| cursor + 1);
        if next >= self.count {
            return None;
        }
        self.cursor = Some(next);
        Some(self.entry(next))
    }

    /// Steps one entry forward in time; past the newest entry it yields an
    /// empty line, and `None` when the walk has not started.
    pub fn newer(&mut self) -> Option<&[u8]> {
        match self.cursor {
            None => None,
            Some(0) => {
                self.cursor = None;
                Some(&[])
            }
            Some(cursor) => {
                self.cursor = Some(cursor - 1);
                Some(self.entry(cursor - 1))
            }
        }
    }

    fn entry(&self, index: usize) -> &[u8] {
        &self.entries[index][..self.lens[index]]
    }
}

impl<const ENTRIES: usize, const WIDTH: usize> Default for History<ENTRIES, WIDTH> {
    fn default() -> Self {
        Self::new()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[derive(Clone, Copy)]
    enum Step {
        Push(&'static [u8]),
        Older(Option<&'static [u8]>),
        Newer(Option<&'static [u8]>),
        Last(Option<&'static [u8]>),
    }

    fn run(steps: &[Step]) -> Result<(), usize> {
        let mut history: History<3, 8> = History::new();
        for (index, step) in steps.iter().enumerate() {
            let matches = match *step {
                Step::Push(line) => {
                    history.push(line);
                    true
                }
                Step::Older(expected) => history.older() == expected,
                Step::Newer(expected) => history.newer() == expected,
                Step::Last(expected) => history.last() == expected,
            };
            if !matches {
                return Err(index);
            }
        }
        Ok(())
    }

    #[test]
    fn walk_cases() {
        use Step::*;
        let cases: &[(&str, &[Step])] = &[
            ("empty", &[Older(None), Newer(None), Last(None)]),
            ("newest first", &[Push(b"a"), Push(b"b"), Older(Some(b"b")), Older(Some(b"a")), Older(None)]),
            ("back to fresh line", &[Push(b"a"), Older(Some(b"a")), Newer(Some(b"")), Newer(None)]),
            ("walk forward", &[Push(b"a"), Push(b"b"), Older(Some(b"b")), Older(Some(b"a")), Newer(Some(b"b"))]),
            ("push resets walk", &[Push(b"a"), Older(Some(b"a")), Push(b"b"), Older(Some(b"b"))]),
            ("skips empty", &[Push(b"a"), Push(b""), Last(Some(b"a"))]),
            ("skips repeat", &[Push(b"a"), Push(b"a"), Older(Some(b"a")), Older(None)]),
            ("drops oldest", &[Push(b"a"), Push(b"b"), Push(b"c"), Push(b"d"), Older(Some(b"d")), Older(Some(b"c")), Older(Some(b"b")), Older(None)]),
            ("truncates", &[Push(b"0123456789"), Last(Some(b"01234567"))]),
        ];
        for (name, steps) in cases {
            assert_eq!(run(steps), Ok(()), "{name}");
        }
    }
}
