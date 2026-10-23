#![no_std]
#![no_main]

use core::cell::{Cell, RefCell};
use core::fmt::{self, Write as _};
use core::future::pending;

use defmt::{info, warn, Debug2Format};
use embassy_executor::Spawner;
use embassy_futures::join::{join, join5};
use embassy_futures::select::{select, select3, select4, Either, Either3, Either4};
use embassy_rp::bind_interrupts;
use embassy_rp::gpio::{self, Pull};
use embassy_rp::peripherals::{UART0, UART1, USB};
use embassy_rp::pwm::{self, Pwm};
use embassy_rp::uart::{self, BufferedUart};
use embassy_rp::usb::{self, Driver};
use embassy_sync::blocking_mutex::raw::NoopRawMutex;
use embassy_sync::channel::Channel;
use embassy_sync::pipe::Pipe;
use embassy_sync::signal::Signal;
use embassy_time::{Duration, Ticker, Timer};
use embassy_usb::class::cdc_acm::{CdcAcmClass, State};
use embassy_usb::driver::EndpointError;
use embassy_usb::Builder;
use embedded_io_async::{Read, Write};
use heapless::{String, Vec};
use {defmt_rtt as _, panic_probe as _};

use spectractrl::colour::wavelength_rgb;
use spectractrl::command::{check, Arg, Rejection, Word};
use spectractrl::history::History;
use spectractrl::scan::{
    self, parse_reading, target_wavelength, Limits, Nm, NmRange, Picometers, Plan, Sampling, ScanError,
};
use spectractrl::xxd::{self, is_printable, XxdRow};

bind_interrupts!(struct Irqs {
    USBCTRL_IRQ => usb::InterruptHandler<USB>;
    UART0_IRQ => uart::BufferedInterruptHandler<UART0>;
    UART1_IRQ => uart::BufferedInterruptHandler<UART1>;
});

const SPECTRADRIVE_BAUD: u32 = 9600;
const PS445_BAUD: u32 = 9600;
const DEVICE_LINE_END: &[u8] = b"\r";
const DEVICE_LINE_LIMIT: usize = 79;
const DONE_MARKER: &[u8] = b"ok";
const FAILED_MARKER: &[u8] = b"?";
const SETTLE: Duration = Duration::from_millis(500);
const DEBOUNCE: Duration = Duration::from_millis(20);
const BLINK: Duration = Duration::from_millis(500);
const RELAY_POLL: Duration = Duration::from_millis(100);
const RELAY_EXIT: u8 = 0x1d;
const LAMP_BRIGHTNESS: u16 = 64;
const PACKET: usize = 64;
const PROMPT: &[u8] = b"> ";
const REPEAT: &[u8] = b".";
const LINE_CAPACITY: usize = 128;
const HISTORY_ENTRIES: usize = 16;
const DRY_READING: &[u8] = b"0";
const HELP: &str = concat!(
    "console\r\n",
    "  help                  this text\r\n",
    "  up / down             walk through the last 16 commands\r\n",
    "  .                     repeat the last command\r\n",
    "  nop                   answers ok\r\n",
    "  status                local state, then asks idle devices for theirs\r\n",
    "  allowed sd|ps         whitelisted words with their arguments\r\n",
    "  sd <forth>            queue a command for the SpectraDrive\r\n",
    "  ps <forth>            queue a command for the PS-445\r\n",
    "                        with the unlock jumper closed, sd/ps skip the whitelist\r\n",
    "  abort sd|ps|scan      stop waiting for a device, or stop the running macro\r\n",
    "  goto sd|ps            transparent link to a device, unlock jumper only; Ctrl-] leaves\r\n",
    "  dry-run <command>     show what an sd, ps or macro command would send, send nothing\r\n",
    "\r\n",
    "macros\r\n",
    "  <grating> <start-nm> <end-nm> <step-nm> <pmt-volts> <integration-ms> SCAN\r\n",
    "      GRATING, VOLTS, CURRENT, PMT-ON, ITIME, INTEGRATE-START, then per step\r\n",
    "      GOTO, wait two integration windows, .READ,\r\n",
    "      one row \"nm,reading\" per step, PMT-OFF at the end, also on failure or abort\r\n",
    "  <grating> <start-nm> <end-nm> <step-nm> <pmt-volts> <reads> RANGE-SCAN\r\n",
    "      like SCAN, but <reads> x .READ per step instead of an integration,\r\n",
    "      one row \"nm,reading,...\" per step\r\n",
    "  <grating> <nm> <reads> POINT-MEASURE\r\n",
    "      the same at a single wavelength, PMT voltage left unchanged\r\n",
    "\r\n",
    "numbers\r\n",
    "  <int> 500   <dec> 300.5, exactly one digit after the point\r\n",
    "  macros always pass the device whitelist, sd/ps unless the unlock jumper is closed\r\n",
    "  device lines are limited to 79 characters; split longer Forth over several commands\r\n",
);

const SPECTRADRIVE_WORDS: &[Word] = &[
    Word::new(b"?GRATING", &[]),
    Word::new(b"?NM", &[]),
    Word::new(b"GOTO", &[Arg::Decimal]),
    Word::new(b"GRATING", &[Arg::Integer]),
    Word::new(b"INPUT", &[]),
    Word::new(b"NM", &[Arg::Decimal]),
    Word::new(b"STATE", &[]),
    Word::new(b"WHO", &[]),
    Word::new(b"WORDS", &[]),
];

const PS445_WORDS: &[Word] = &[
    Word::new(b".READ", &[]),
    Word::new(b"?VOLTS", &[]),
    Word::new(b"CURRENT", &[]),
    Word::new(b"INTEGRATE-START", &[]),
    Word::new(b"ITIME", &[Arg::Integer]),
    Word::new(b"PMT-OFF", &[]),
    Word::new(b"PMT-ON", &[]),
    Word::new(b"PMT-RANGE", &[]),
    Word::new(b"STATE", &[]),
    Word::new(b"VOLTS", &[Arg::Integer]),
    Word::new(b"WHO", &[]),
    Word::new(b"WORDS", &[]),
];

const PMT_VOLTAGE_WORD: &str = "VOLTS";
const INTEGRATION_TIME_WORD: &str = "ITIME";
const INTEGRATE_WORD: &str = "INTEGRATE-START";
const CURRENT_MODE_WORD: &str = "CURRENT";
const SETTLE_WINDOWS: u64 = 2;
const READ_WORD: &str = ".READ";
const SPECTRADRIVE_STATUS_QUERIES: &[&[u8]] = &[b"?NM", b"?GRATING", b"STATE"];
const PS445_STATUS_QUERIES: &[&[u8]] = &[];
const PMT_ON_WORD: &str = "PMT-ON";
const PMT_OFF_WORD: &str = "PMT-OFF";

const GRATINGS: &[NmRange] = &[
    NmRange::new(0, 1_400_000),
    NmRange::new(0, 1_400_000),
    NmRange::new(0, 1_400_000),
];
const SCAN_LIMITS: Limits<'static> = Limits { gratings: GRATINGS, volts: 0..=1000 };

type Console<'d> = CdcAcmClass<'d, Driver<'d, USB>>;
type Line = Vec<u8, LINE_CAPACITY>;
type CommandHistory = History<HISTORY_ENTRIES, LINE_CAPACITY>;
type Text = Vec<u8, 512>;
type Requests = Channel<NoopRawMutex, Line, 4>;
type Events = Channel<NoopRawMutex, Event, 8>;
type RawPipe = Pipe<NoopRawMutex, 256>;

#[derive(Clone, Copy, defmt::Format)]
enum Device {
    SpectraDrive,
    Ps445,
    Scan,
}

impl Device {
    fn name(self) -> &'static str {
        match self {
            Device::SpectraDrive => "sd",
            Device::Ps445 => "ps",
            Device::Scan => "scan",
        }
    }
}

#[derive(Clone, Copy, PartialEq)]
enum Kind {
    Output,
    Done,
    Failed,
    Aborted,
}

impl Kind {
    fn label(self) -> &'static [u8] {
        match self {
            Kind::Output => b":out] ",
            Kind::Done => b":done] ",
            Kind::Failed => b":fail] ",
            Kind::Aborted => b":abort] ",
        }
    }
}

struct Event {
    device: Device,
    kind: Kind,
    text: Text,
}

struct Outcome {
    kind: Kind,
    output: Text,
}

struct Link {
    device: Device,
    allowed: &'static [Word],
    requests: Requests,
    abort: Signal<NoopRawMutex, ()>,
    finished: Signal<NoopRawMutex, Outcome>,
    current: RefCell<Option<Line>>,
    last_output: RefCell<Text>,
    wavelength: Cell<Option<Picometers>>,
    sent: Cell<u32>,
    lines: Cell<u32>,
    errors: Cell<u32>,
    passthrough: Cell<bool>,
    to_device: RawPipe,
    from_device: RawPipe,
}

impl Link {
    fn new(device: Device, allowed: &'static [Word]) -> Self {
        Self {
            device,
            allowed,
            requests: Channel::new(),
            abort: Signal::new(),
            finished: Signal::new(),
            current: RefCell::new(None),
            last_output: RefCell::new(Text::new()),
            wavelength: Cell::new(None),
            sent: Cell::new(0),
            lines: Cell::new(0),
            errors: Cell::new(0),
            passthrough: Cell::new(false),
            to_device: RawPipe::new(),
            from_device: RawPipe::new(),
        }
    }

    fn is_busy(&self) -> bool {
        self.current.borrow().is_some()
    }

    fn is_idle(&self) -> bool {
        !self.is_busy() && self.requests.is_empty()
    }
}

#[derive(Clone, Copy, PartialEq)]
enum Mode {
    Live,
    Dry,
}

struct Job {
    plan: Plan,
    mode: Mode,
}

struct Scan {
    requests: Channel<NoopRawMutex, Job, 1>,
    active: Cell<bool>,
    abort: Cell<bool>,
}

impl Scan {
    fn new() -> Self {
        Self { requests: Channel::new(), active: Cell::new(false), abort: Cell::new(false) }
    }
}

enum ScanFailure {
    Rejected(Device),
    Unfinished(Device),
    BadReading,
    Overflow,
    Aborted,
}

impl From<fmt::Error> for ScanFailure {
    fn from(_: fmt::Error) -> Self {
        ScanFailure::Overflow
    }
}

impl fmt::Display for ScanFailure {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            ScanFailure::Rejected(device) => write!(f, "command rejected by {} whitelist", device.name()),
            ScanFailure::Unfinished(device) => write!(f, "{} command did not end with ok", device.name()),
            ScanFailure::BadReading => f.write_str("no number in .READ output"),
            ScanFailure::Overflow => f.write_str("line too long"),
            ScanFailure::Aborted => f.write_str("aborted"),
        }
    }
}

struct Links<'a> {
    spectradrive: &'a Link,
    ps445: &'a Link,
    scan: &'a Scan,
    unlocked: &'a Cell<bool>,
}

impl Links<'_> {
    fn find(&self, name: &[u8]) -> Option<&Link> {
        match name {
            b"sd" => Some(self.spectradrive),
            b"ps" => Some(self.ps445),
            _ => None,
        }
    }

    fn all(&self) -> [&Link; 2] {
        [self.spectradrive, self.ps445]
    }

    fn relaying(&self) -> Option<&Link> {
        self.all().into_iter().find(|link| link.passthrough.get())
    }
}

enum Input {
    Request(Line),
    Abort,
    Byte(u8),
    Error(uart::Error),
    Settled,
}

#[derive(Clone, Copy)]
enum Escape {
    Idle,
    Started,
    Bracket,
}

struct Editor {
    line: Line,
    escape: Escape,
    history: CommandHistory,
}

struct Disconnected;

impl From<EndpointError> for Disconnected {
    fn from(error: EndpointError) -> Self {
        match error {
            EndpointError::BufferOverflow => panic!("usb packet exceeds buffer"),
            EndpointError::Disabled => Disconnected,
        }
    }
}

#[embassy_executor::main]
async fn main(_spawner: Spawner) {
    let p = embassy_rp::init(Default::default());

    let (mut sd_tx, mut sd_rx) = ([0u8; 64], [0u8; 256]);
    let (mut ps_tx, mut ps_rx) = ([0u8; 64], [0u8; 256]);
    let spectradrive = BufferedUart::new(
        p.UART0, p.PIN_0, p.PIN_1, Irqs,
        &mut sd_tx, &mut sd_rx, uart_config(SPECTRADRIVE_BAUD),
    );
    let ps445 = BufferedUart::new(
        p.UART1, p.PIN_4, p.PIN_5, Irqs,
        &mut ps_tx, &mut ps_rx, uart_config(PS445_BAUD),
    );

    let sd = Link::new(Device::SpectraDrive, SPECTRADRIVE_WORDS);
    let ps = Link::new(Device::Ps445, PS445_WORDS);
    let scan = Scan::new();
    let events = Events::new();
    let unlock_jumper = gpio::Input::new(p.PIN_2, Pull::Up);
    let red_green = Pwm::new_output_ab(p.PWM_SLICE1, p.PIN_18, p.PIN_19, lamp_config(0, 0));
    let blue = Pwm::new_output_a(p.PWM_SLICE2, p.PIN_20, lamp_config(0, 0));
    let unlocked = Cell::new(false);
    let links = Links { spectradrive: &sd, ps445: &ps, scan: &scan, unlocked: &unlocked };

    let mut config_descriptor = [0u8; 256];
    let mut bos_descriptor = [0u8; 256];
    let mut control_buf = [0u8; 64];
    let mut state = State::new();
    let mut builder = Builder::new(
        Driver::new(p.USB, Irqs),
        usb_config(),
        &mut config_descriptor,
        &mut bos_descriptor,
        &mut [],
        &mut control_buf,
    );
    let mut console = CdcAcmClass::new(&mut builder, &mut state, PACKET as u16);
    let mut usb = builder.build();

    info!("spectractrl up");
    join(
        join5(
            usb.run(),
            serve(&mut console, &links, &events),
            worker(&sd, spectradrive, &events),
            worker(&ps, ps445, &events),
            scanner(&links, &events),
        ),
        join(guard(unlock_jumper, &links), lamp(red_green, blue, &unlocked, &sd)),
    )
    .await;
}

async fn guard(mut jumper: gpio::Input<'_>, links: &Links<'_>) -> ! {
    loop {
        let closed = jumper.is_low();
        if closed != links.unlocked.get() {
            warn!("whitelist {}", if closed { "bypassed by unlock jumper" } else { "active" });
        }
        links.unlocked.set(closed);
        if !closed {
            end_relays(links);
        }
        jumper.wait_for_any_edge().await;
        Timer::after(DEBOUNCE).await;
    }
}

async fn lamp(mut red_green: Pwm<'_>, mut blue: Pwm<'_>, unlocked: &Cell<bool>, spectradrive: &Link) -> ! {
    let mut ticker = Ticker::every(BLINK);
    let mut blink_on = false;
    loop {
        blink_on = !blink_on;
        let [red, green, blue_level] = lamp_colour(unlocked.get(), blink_on, spectradrive.wavelength.get());
        red_green.set_config(&lamp_config(red, green));
        blue.set_config(&lamp_config(blue_level, 0));
        ticker.next().await;
    }
}

fn lamp_colour(unlocked: bool, blink_on: bool, wavelength: Option<Picometers>) -> [u8; 3] {
    match (unlocked, wavelength) {
        (true, _) if blink_on => [u8::MAX, 0, 0],
        (true, _) => [0, 0, 0],
        (false, Some(wavelength)) => wavelength_rgb(wavelength),
        (false, None) => [0, 0, 0],
    }
}

fn lamp_config(a: u8, b: u8) -> pwm::Config {
    let mut config = pwm::Config::default();
    config.top = u16::from(u8::MAX);
    config.compare_a = u16::from(a) * LAMP_BRIGHTNESS / u16::from(u8::MAX);
    config.compare_b = u16::from(b) * LAMP_BRIGHTNESS / u16::from(u8::MAX);
    config.invert_a = true;
    config.invert_b = true;
    config
}

fn uart_config(baudrate: u32) -> uart::Config {
    let mut config = uart::Config::default();
    config.baudrate = baudrate;
    config
}

fn usb_config() -> embassy_usb::Config<'static> {
    let mut config = embassy_usb::Config::new(0x16c0, 0x27dd);
    config.manufacturer = Some("Microprojects");
    config.product = Some("spectractrl");
    config.serial_number = Some("0001");
    config.max_power = 100;
    config.max_packet_size_0 = 64;
    config
}

async fn worker(link: &Link, mut port: BufferedUart, events: &Events) -> ! {
    resync(link, &mut port).await;
    let mut line = Text::new();
    let mut settling = false;
    loop {
        if link.passthrough.get() {
            relay_device(link, &mut port).await;
            line.clear();
            settling = false;
            continue;
        }
        match next_input(link, &mut port, settling).await {
            Input::Request(command) => start(link, &mut port, command, events).await,
            Input::Abort => finish(link, events, Kind::Aborted),
            Input::Byte(byte) => settling = on_device_byte(link, &mut line, byte, events),
            Input::Settled => {
                settling = false;
                finish(link, events, Kind::Failed);
            }
            Input::Error(error) => {
                warn!("{} rx: {}", link.device, Debug2Format(&error));
                bump(&link.errors);
            }
        }
    }
}

async fn relay_device(link: &Link, port: &mut BufferedUart) {
    info!("{} relay on", link.device);
    let mut inbound = [0u8; 64];
    let mut outbound = [0u8; 64];
    while link.passthrough.get() {
        let input = select3(port.read(&mut inbound), link.to_device.read(&mut outbound), link.abort.wait()).await;
        match input {
            Either3::First(Ok(len)) => forward_to_console(link, &inbound[..len]),
            Either3::First(Err(error)) => {
                warn!("{} relay rx: {}", link.device, Debug2Format(&error));
                bump(&link.errors);
            }
            Either3::Second(len) => {
                if let Err(error) = port.write_all(&outbound[..len]).await {
                    warn!("{} relay tx: {}", link.device, Debug2Format(&error));
                    bump(&link.errors);
                }
            }
            Either3::Third(()) => {}
        }
    }
    info!("{} relay off", link.device);
}

fn forward_to_console(link: &Link, bytes: &[u8]) {
    match link.from_device.try_write(bytes) {
        Ok(written) if written == bytes.len() => {}
        _ => warn!("{} relay output dropped, console not reading", link.device),
    }
}

fn end_relay(link: &Link) {
    link.passthrough.set(false);
    link.abort.signal(());
}

fn end_relays(links: &Links<'_>) {
    if let Some(link) = links.relaying() {
        end_relay(link);
    }
}

async fn next_input(link: &Link, port: &mut BufferedUart, settling: bool) -> Input {
    let mut byte = [0u8; 1];
    let request = async {
        match link.is_busy() {
            true => pending::<Line>().await,
            false => link.requests.receive().await,
        }
    };
    let settle = async {
        match settling {
            true => Timer::after(SETTLE).await,
            false => pending::<()>().await,
        }
    };
    let input = select4(request, link.abort.wait(), port.read(&mut byte), settle).await;
    match input {
        Either4::First(command) => Input::Request(command),
        Either4::Second(()) => Input::Abort,
        Either4::Third(Ok(_)) => Input::Byte(byte[0]),
        Either4::Third(Err(error)) => Input::Error(error),
        Either4::Fourth(()) => Input::Settled,
    }
}

async fn resync(link: &Link, port: &mut BufferedUart) {
    if let Err(error) = send(port, b"").await {
        warn!("{} resync: {}", link.device, Debug2Format(&error));
        bump(&link.errors);
    }
}

async fn start(link: &Link, port: &mut BufferedUart, command: Line, events: &Events) {
    info!("{} tx {=[u8]:a}", link.device, &command[..]);
    match send(port, &command).await {
        Ok(()) => {
            bump(&link.sent);
            *link.current.borrow_mut() = Some(command);
        }
        Err(error) => {
            warn!("{} tx: {}", link.device, Debug2Format(&error));
            bump(&link.errors);
            publish(events, link.device, Kind::Failed, b"write error");
        }
    }
}

async fn send<P: Write>(port: &mut P, command: &[u8]) -> Result<(), P::Error> {
    port.write_all(command).await?;
    port.write_all(DEVICE_LINE_END).await?;
    port.flush().await
}

fn finish(link: &Link, events: &Events, kind: Kind) {
    if let Some(command) = link.current.take() {
        publish(events, link.device, kind, &command);
        conclude(link, kind);
    }
}

fn conclude(link: &Link, kind: Kind) {
    let command = link.current.take();
    if kind == Kind::Done {
        track_wavelength(link, command.as_deref());
    }
    let output = link.last_output.take();
    link.finished.signal(Outcome { kind, output });
}

fn track_wavelength(link: &Link, command: Option<&[u8]>) {
    if let Some(wavelength) = command.and_then(target_wavelength) {
        link.wavelength.set(Some(wavelength));
    }
}

fn remember(link: &Link, text: &[u8]) {
    *link.last_output.borrow_mut() = Text::from_slice(text).expect("line fits text");
}

fn on_device_byte(link: &Link, line: &mut Text, byte: u8, events: &Events) -> bool {
    defmt::trace!("{} rx {=[u8]:a}", link.device, &[byte][..]);
    match byte {
        0 | b'\r' => false,
        b'\n' => complete_line(link, line, events) == Some(Kind::Output) && link.is_busy(),
        _ => {
            if line.is_full() {
                complete_line(link, line, events);
            }
            line.push(byte).expect("line has room after completion");
            false
        }
    }
}

fn complete_line(link: &Link, line: &mut Text, events: &Events) -> Option<Kind> {
    let text = line.trim_ascii();
    let kind = (!text.is_empty()).then(|| classify(link.is_busy(), text));
    if let Some(kind) = kind {
        bump(&link.lines);
        publish(events, link.device, kind, text);
        match kind {
            Kind::Output if link.is_busy() => remember(link, text),
            Kind::Output => {}
            _ => conclude(link, kind),
        }
    }
    line.clear();
    kind
}

fn classify(busy: bool, text: &[u8]) -> Kind {
    match busy {
        false => Kind::Output,
        true if text.ends_with(DONE_MARKER) => Kind::Done,
        true if text.ends_with(FAILED_MARKER) => Kind::Failed,
        true => Kind::Output,
    }
}

fn publish(events: &Events, device: Device, kind: Kind, text: &[u8]) {
    let text = Text::from_slice(text).expect("event text comes from a line, so it fits");
    if events.try_send(Event { device, kind, text }).is_err() {
        warn!("{} event dropped, queue full", device);
    }
}

fn bump(counter: &Cell<u32>) {
    counter.set(counter.get().wrapping_add(1));
}

async fn scanner(links: &Links<'_>, events: &Events) -> ! {
    let scan = links.scan;
    loop {
        let job = scan.requests.receive().await;
        scan.abort.set(false);
        let result = run_scan(&job, links, events).await;
        scan.active.set(false);
        report(events, job.mode, result).await;
    }
}

async fn run_scan(job: &Job, links: &Links<'_>, events: &Events) -> Result<u32, ScanFailure> {
    let plan = &job.plan;
    let mode = job.mode;
    emit(events, Kind::Output, format_args!("# {plan}")).await?;
    call(mode, links.spectradrive, events, format_args!("{} GRATING", plan.grating)).await?;
    if let Some(volts) = plan.volts {
        call(mode, links.ps445, events, format_args!("{volts} {PMT_VOLTAGE_WORD}")).await?;
    }
    if let Sampling::Integrate { .. } = plan.sampling {
        call(mode, links.ps445, events, format_args!("{CURRENT_MODE_WORD}")).await?;
    }
    let swept = match call(mode, links.ps445, events, format_args!("{PMT_ON_WORD}")).await {
        Ok(_) => start_sampling(job, links, events).await,
        Err(failure) => Err(failure),
    };
    let switched_off = call(mode, links.ps445, events, format_args!("{PMT_OFF_WORD}")).await;
    let steps = swept?;
    switched_off?;
    Ok(steps)
}

async fn start_sampling(job: &Job, links: &Links<'_>, events: &Events) -> Result<u32, ScanFailure> {
    if let Sampling::Integrate { ms } = job.plan.sampling {
        call(job.mode, links.ps445, events, format_args!("{ms} {INTEGRATION_TIME_WORD}")).await?;
        call(job.mode, links.ps445, events, format_args!("{INTEGRATE_WORD}")).await?;
    }
    sweep(job, links, events).await
}

async fn sweep(job: &Job, links: &Links<'_>, events: &Events) -> Result<u32, ScanFailure> {
    let mode = job.mode;

    let mut steps = 0;
    for nm in job.plan.steps() {
        if links.scan.abort.get() {
            return Err(ScanFailure::Aborted);
        }
        let row = measure(job, nm, links, events).await?;
        if mode == Mode::Live {
            emit(events, Kind::Output, format_args!("{row}")).await?;
        }
        steps += 1;
    }
    Ok(steps)
}

async fn measure(job: &Job, nm: i32, links: &Links<'_>, events: &Events) -> Result<String<512>, ScanFailure> {
    call(job.mode, links.spectradrive, events, format_args!("{} GOTO", Nm(nm))).await?;
    let mut row = String::new();
    write!(row, "{}", Nm(nm))?;
    match job.plan.sampling {
        Sampling::Reads(reads) => {
            for _ in 0..reads {
                let value = read_value(job.mode, links, events).await?;
                write!(row, ",{value}")?;
            }
        }
        Sampling::Integrate { ms } => {
            settle(job.mode, ms).await;
            let value = read_value(job.mode, links, events).await?;
            write!(row, ",{value}")?;
        }
    }
    Ok(row)
}

async fn read_value(mode: Mode, links: &Links<'_>, events: &Events) -> Result<i32, ScanFailure> {
    let output = call(mode, links.ps445, events, format_args!("{READ_WORD}")).await?;
    parse_reading(&output).ok_or(ScanFailure::BadReading)
}

async fn settle(mode: Mode, ms: i32) {
    if mode == Mode::Live {
        Timer::after(Duration::from_millis(u64::from(ms.unsigned_abs()) * SETTLE_WINDOWS)).await;
    }
}

async fn call(mode: Mode, link: &Link, events: &Events, args: fmt::Arguments<'_>) -> Result<Text, ScanFailure> {
    let command = scan_command(link, args)?;
    match mode {
        Mode::Live => transact(link, &command).await,
        Mode::Dry => preview_call(link, events, &command).await,
    }
}

fn scan_command(link: &Link, args: fmt::Arguments<'_>) -> Result<String<128>, ScanFailure> {
    let mut command: String<128> = String::new();
    command.write_fmt(args)?;
    if !fits_device_line(command.as_bytes()) {
        return Err(ScanFailure::Overflow);
    }
    if check(command.as_bytes(), link.allowed).is_err() {
        warn!("{} rejected scan command {=str}", link.device, command.as_str());
        return Err(ScanFailure::Rejected(link.device));
    }
    Ok(command)
}

async fn preview_call(link: &Link, events: &Events, command: &str) -> Result<Text, ScanFailure> {
    emit(events, Kind::Output, format_args!("{}> {command}", link.device.name())).await?;
    Ok(reply(DRY_READING))
}

async fn transact(link: &Link, command: &str) -> Result<Text, ScanFailure> {
    let line = Line::from_slice(command.as_bytes()).expect("string and line have equal capacity");
    link.finished.reset();
    link.requests.send(line).await;
    let outcome = link.finished.wait().await;
    match outcome.kind {
        Kind::Done => Ok(outcome.output),
        _ => Err(ScanFailure::Unfinished(link.device)),
    }
}

async fn emit(events: &Events, kind: Kind, args: fmt::Arguments<'_>) -> Result<(), ScanFailure> {
    let mut text: String<512> = String::new();
    text.write_fmt(args)?;
    let text = Text::from_slice(text.as_bytes()).expect("string and text have equal capacity");
    events.send(Event { device: Device::Scan, kind, text }).await;
    Ok(())
}

async fn report(events: &Events, mode: Mode, result: Result<u32, ScanFailure>) {
    let reported = match (result, mode) {
        (Ok(steps), Mode::Live) => emit(events, Kind::Done, format_args!("{steps} steps")).await,
        (Ok(steps), Mode::Dry) => emit(events, Kind::Done, format_args!("dry run, {steps} steps, nothing sent")).await,
        (Err(failure), _) => emit(events, Kind::Failed, format_args!("{failure}")).await,
    };
    if reported.is_err() {
        warn!("scan report too long");
    }
}

async fn serve(console: &mut Console<'_>, links: &Links<'_>, events: &Events) -> ! {
    let mut editor = Editor { line: Line::new(), escape: Escape::Idle, history: CommandHistory::new() };
    loop {
        console.wait_connection().await;
        info!("console connected");
        let Disconnected = session(console, links, events, &mut editor).await;
        end_relays(links);
        info!("console disconnected");
    }
}

async fn session(
    console: &mut Console<'_>,
    links: &Links<'_>,
    events: &Events,
    editor: &mut Editor,
) -> Disconnected {
    editor.line.clear();
    editor.escape = Escape::Idle;
    if let Err(disconnected) = write(console, PROMPT).await {
        return disconnected;
    }
    loop {
        if let Err(disconnected) = step(console, links, events, editor).await {
            return disconnected;
        }
    }
}

async fn step(
    console: &mut Console<'_>,
    links: &Links<'_>,
    events: &Events,
    editor: &mut Editor,
) -> Result<(), Disconnected> {
    if let Some(link) = links.relaying() {
        return relay_console(console, link).await;
    }
    let mut packet = [0u8; PACKET];
    let input = select(console.read_packet(&mut packet), events.receive()).await;
    match input {
        Either::First(received) => {
            let len = received?;
            for &byte in &packet[..len] {
                on_byte(console, links, editor, byte).await?;
            }
            Ok(())
        }
        Either::Second(event) if is_noise(links, &event) => Ok(()),
        Either::Second(event) => announce(console, &event, &editor.line).await,
    }
}

async fn relay_console(console: &mut Console<'_>, link: &Link) -> Result<(), Disconnected> {
    while link.passthrough.get() {
        relay_step(console, link).await?;
    }
    write(console, b"\r\n[").await?;
    write(console, link.device.name().as_bytes()).await?;
    write(console, b":relay] ended\r\n").await?;
    write(console, PROMPT).await
}

async fn relay_step(console: &mut Console<'_>, link: &Link) -> Result<(), Disconnected> {
    let mut packet = [0u8; PACKET];
    let mut inbound = [0u8; PACKET];
    let input = select3(
        console.read_packet(&mut packet),
        link.from_device.read(&mut inbound),
        Timer::after(RELAY_POLL),
    )
    .await;
    match input {
        Either3::First(received) => {
            let len = received?;
            relay_typed(link, &packet[..len]).await;
            Ok(())
        }
        Either3::Second(len) => write(console, &inbound[..len]).await,
        Either3::Third(()) => Ok(()),
    }
}

async fn relay_typed(link: &Link, typed: &[u8]) {
    match typed.iter().position(|&byte| byte == RELAY_EXIT) {
        Some(at) => {
            write_pipe(&link.to_device, &typed[..at]).await;
            end_relay(link);
        }
        None => write_pipe(&link.to_device, typed).await,
    }
}

async fn write_pipe(pipe: &RawPipe, mut bytes: &[u8]) {
    while !bytes.is_empty() {
        let written = pipe.write(bytes).await;
        bytes = &bytes[written..];
    }
}

fn is_noise(links: &Links<'_>, event: &Event) -> bool {
    links.scan.active.get()
        && !matches!(event.device, Device::Scan)
        && matches!(event.kind, Kind::Output | Kind::Done)
}

async fn announce(console: &mut Console<'_>, event: &Event, line: &Line) -> Result<(), Disconnected> {
    write(console, b"\r\x1b[K[").await?;
    write(console, event.device.name().as_bytes()).await?;
    write(console, event.kind.label()).await?;
    write(console, &printable(&event.text)).await?;
    write(console, b"\r\n").await?;
    if !event.text.iter().all(is_printable) {
        dump(console, &event.text).await?;
    }
    write(console, PROMPT).await?;
    write(console, line).await
}

fn printable(text: &[u8]) -> Text {
    text.iter()
        .map(|byte| if is_printable(byte) { *byte } else { b'.' })
        .collect()
}

async fn dump(console: &mut Console<'_>, bytes: &[u8]) -> Result<(), Disconnected> {
    for (index, chunk) in bytes.chunks(xxd::WIDTH).enumerate() {
        let mut row: String<80> = String::new();
        write!(row, "{}\r\n", XxdRow::new(index * xxd::WIDTH, chunk)).expect("xxd row fits 80 bytes");
        write(console, row.as_bytes()).await?;
    }
    Ok(())
}

async fn on_byte(
    console: &mut Console<'_>,
    links: &Links<'_>,
    editor: &mut Editor,
    byte: u8,
) -> Result<(), Disconnected> {
    match (editor.escape, byte) {
        (Escape::Idle, 0x1b) => {
            editor.escape = Escape::Started;
            Ok(())
        }
        (Escape::Started, b'[') => {
            editor.escape = Escape::Bracket;
            Ok(())
        }
        (Escape::Started, _) => {
            editor.escape = Escape::Idle;
            Ok(())
        }
        (Escape::Bracket, b'A') => {
            editor.escape = Escape::Idle;
            show_recalled(console, &mut editor.line, editor.history.older()).await
        }
        (Escape::Bracket, b'B') => {
            editor.escape = Escape::Idle;
            show_recalled(console, &mut editor.line, editor.history.newer()).await
        }
        (Escape::Bracket, 0x40..=0x7e) => {
            editor.escape = Escape::Idle;
            Ok(())
        }
        (Escape::Bracket, _) => Ok(()),
        (Escape::Idle, b'\r' | b'\n') => submit(console, links, editor).await,
        (Escape::Idle, 0x08 | 0x7f) => erase(console, &mut editor.line).await,
        (Escape::Idle, _) => append(console, &mut editor.line, byte).await,
    }
}

async fn show_recalled(
    console: &mut Console<'_>,
    line: &mut Line,
    recalled: Option<&[u8]>,
) -> Result<(), Disconnected> {
    let Some(entry) = recalled else {
        return write(console, b"\x07").await;
    };
    *line = Line::from_slice(entry).expect("history entries fit a line");
    write(console, b"\r\x1b[K").await?;
    write(console, PROMPT).await?;
    write(console, line).await
}

async fn expand_repeat(console: &mut Console<'_>, editor: &mut Editor) -> Result<(), Disconnected> {
    if editor.line.as_slice() != REPEAT {
        return Ok(());
    }
    let Some(last) = editor.history.last() else {
        return Ok(());
    };
    editor.line = Line::from_slice(last).expect("history entries fit a line");
    write(console, &editor.line).await?;
    write(console, b"\r\n").await
}

async fn submit(console: &mut Console<'_>, links: &Links<'_>, editor: &mut Editor) -> Result<(), Disconnected> {
    write(console, b"\r\n").await?;
    expand_repeat(console, editor).await?;
    editor.history.push(&editor.line);
    let line = &mut editor.line;
    if line.as_slice() == b"help" {
        line.clear();
        write(console, HELP.as_bytes()).await?;
        return write(console, PROMPT).await;
    }
    let reply = execute(links, line);
    line.clear();
    if !reply.is_empty() {
        write(console, &reply).await?;
        write(console, b"\r\n").await?;
    }
    write(console, PROMPT).await
}

async fn erase(console: &mut Console<'_>, line: &mut Line) -> Result<(), Disconnected> {
    match line.pop() {
        Some(_) => write(console, b"\x08 \x08").await,
        None => Ok(()),
    }
}

async fn append(console: &mut Console<'_>, line: &mut Line, byte: u8) -> Result<(), Disconnected> {
    match line.push(byte) {
        Ok(()) => write(console, &[byte]).await,
        Err(_) => write(console, b"\x07").await,
    }
}

async fn write(console: &mut Console<'_>, data: &[u8]) -> Result<(), Disconnected> {
    for chunk in data.chunks(PACKET) {
        console.write_packet(chunk).await?;
    }
    if !data.is_empty() && data.len() % PACKET == 0 {
        console.write_packet(&[]).await?;
    }
    Ok(())
}

fn execute(links: &Links<'_>, line: &[u8]) -> Text {
    match split_command(line) {
        (b"dry-run", rest) => dry_run(links, rest),
        _ => run_command(links, line),
    }
}

fn dry_run(links: &Links<'_>, line: &[u8]) -> Text {
    match split_command(line) {
        (b"sd", command) => preview(links.spectradrive, command, links.unlocked.get()),
        (b"ps", command) => preview(links.ps445, command, links.unlocked.get()),
        _ => start_macro(links, line, Mode::Dry),
    }
}

fn start_macro(links: &Links<'_>, line: &[u8], mode: Mode) -> Text {
    match (scan::parse(line, &SCAN_LIMITS), mode) {
        (Ok(plan), _) => start_scan(links, Job { plan, mode }),
        (Err(ScanError::NotAScan), Mode::Live) => reply(b"unknown command"),
        (Err(ScanError::NotAScan), Mode::Dry) => reply(b"usage: dry-run sd|ps <command> | dry-run <macro>"),
        (Err(error), _) => formatted(format_args!("scan: {error}")),
    }
}

fn preview(link: &Link, command: &[u8], unlocked: bool) -> Text {
    if !fits_device_line(command) {
        return too_long(link, command);
    }
    match (unlocked, check(command, link.allowed)) {
        (false, Err(rejection)) => rejected(link, &rejection),
        (true, Err(_)) => formatted(format_args!(
            "would send to {} unchecked: {}",
            link.device.name(),
            core::str::from_utf8(command).unwrap_or("<binary>"),
        )),
        (_, Ok(())) => formatted(format_args!(
            "would send to {}: {}",
            link.device.name(),
            core::str::from_utf8(command).unwrap_or("<binary>"),
        )),
    }
}

fn run_command(links: &Links<'_>, line: &[u8]) -> Text {
    match split_command(line) {
        (b"", _) => Text::new(),
        (b"nop", _) => reply(b"ok"),
        (b"status", _) => status(links),
        (b"allowed", target) => match links.find(target) {
            Some(link) => allowed_words(link),
            None => reply(b"usage: allowed sd|ps"),
        },
        (b"abort", b"scan") => abort_scan(links),
        (b"abort", target) => match links.find(target) {
            Some(link) => {
                link.abort.signal(());
                reply(b"abort requested")
            }
            None => reply(b"usage: abort sd|ps|scan"),
        },
        (b"goto", target) => start_relay(links, target),
        (b"sd" | b"ps", _) if links.scan.active.get() => reply(b"scan running, use abort scan"),
        (b"sd", command) => enqueue(links.spectradrive, command, links.unlocked.get()),
        (b"ps", command) => enqueue(links.ps445, command, links.unlocked.get()),
        _ => start_macro(links, line, Mode::Live),
    }
}

fn start_relay(links: &Links<'_>, target: &[u8]) -> Text {
    let Some(link) = links.find(target) else {
        return reply(b"usage: goto sd|ps");
    };
    if !links.unlocked.get() {
        return reply(b"goto needs the unlock jumper closed");
    }
    if links.scan.active.get() {
        return reply(b"scan running, use abort scan");
    }
    link.to_device.clear();
    link.from_device.clear();
    link.passthrough.set(true);
    link.abort.signal(());
    formatted(format_args!(
        "[{}:relay] transparent, Ctrl-] or opening the unlock jumper leaves",
        link.device.name(),
    ))
}

fn fits_device_line(command: &[u8]) -> bool {
    command.len() <= DEVICE_LINE_LIMIT
}

fn too_long(link: &Link, command: &[u8]) -> Text {
    formatted(format_args!(
        "rejected on {}: {} characters, the device reads at most {} per line",
        link.device.name(),
        command.len(),
        DEVICE_LINE_LIMIT,
    ))
}

fn enqueue(link: &Link, command: &[u8], unlocked: bool) -> Text {
    if !fits_device_line(command) {
        return too_long(link, command);
    }
    match (unlocked, check(command, link.allowed)) {
        (false, Err(rejection)) => rejected(link, &rejection),
        (true, Err(_)) => {
            warn!("{} unchecked {=[u8]:a}", link.device, command);
            push(link, command)
        }
        (_, Ok(())) => push(link, command),
    }
}

fn push(link: &Link, command: &[u8]) -> Text {
    let command = Line::from_slice(command).expect("command is part of a line, so it fits");
    match link.requests.try_send(command) {
        Ok(()) if link.is_busy() => reply(b"queued (device busy)"),
        Ok(()) => reply(b"queued"),
        Err(_) => reply(b"queue full"),
    }
}

fn start_scan(links: &Links<'_>, job: Job) -> Text {
    if links.scan.active.get() {
        return reply(b"scan already running");
    }
    if job.mode == Mode::Live && !links.all().iter().all(|link| link.is_idle()) {
        return reply(b"devices busy, wait or abort first");
    }
    let started = match job.mode {
        Mode::Live => &b"scan started"[..],
        Mode::Dry => &b"dry run started"[..],
    };
    match links.scan.requests.try_send(job) {
        Ok(()) => {
            links.scan.active.set(true);
            reply(started)
        }
        Err(_) => reply(b"scan already queued"),
    }
}

fn abort_scan(links: &Links<'_>) -> Text {
    if !links.scan.active.get() {
        return reply(b"no scan running");
    }
    links.scan.abort.set(true);
    for link in links.all() {
        link.abort.signal(());
    }
    reply(b"abort requested")
}

fn formatted(args: fmt::Arguments<'_>) -> Text {
    let mut text: String<512> = String::new();
    match text.write_fmt(args) {
        Ok(()) => reply(text.as_bytes()),
        Err(_) => reply(b"reply too long"),
    }
}

fn status(links: &Links<'_>) -> Text {
    let mut text: String<512> = String::new();
    for link in links.all() {
        if describe(link, &mut text).is_err() {
            return reply(b"status overflow");
        }
    }
    let scan_state = match links.scan.active.get() {
        true => "running",
        false => "idle",
    };
    if write!(text, "scan {scan_state}\r\n").is_err() {
        return reply(b"status overflow");
    }
    let wavelength = links.spectradrive.wavelength.get();
    let written = match wavelength {
        Some(wavelength) => write!(text, "wavelength {} nm (last move)\r\n", Nm(wavelength)),
        None => write!(text, "wavelength unknown\r\n"),
    };
    if written.is_err() {
        return reply(b"status overflow");
    }
    let whitelist_state = match links.unlocked.get() {
        true => "bypassed (unlock jumper closed)",
        false => "active",
    };
    if write!(text, "whitelist {whitelist_state}\r\n").is_err() {
        return reply(b"status overflow");
    }
    let remotes = [
        (links.spectradrive, SPECTRADRIVE_STATUS_QUERIES),
        (links.ps445, PS445_STATUS_QUERIES),
    ];
    for (link, queries) in remotes {
        let outcome = query_remote(links, link, queries);
        if write!(text, "remote {}: {outcome}\r\n", link.device.name()).is_err() {
            return reply(b"status overflow");
        }
    }
    reply(text.trim_end().as_bytes())
}

fn query_remote(links: &Links<'_>, link: &Link, queries: &[&[u8]]) -> &'static str {
    if links.scan.active.get() {
        return "scan running, not queried";
    }
    if !link.is_idle() {
        return "busy, not queried";
    }
    for query in queries {
        if check(query, link.allowed).is_err() {
            return "status query not whitelisted, stopped";
        }
        let line = Line::from_slice(query).expect("status queries fit a line");
        if link.requests.try_send(line).is_err() {
            return "queue full, partly queried";
        }
    }
    "queried, answers follow"
}

fn describe(link: &Link, text: &mut String<512>) -> core::fmt::Result {
    write!(text, "{} ", link.device.name())?;
    match link.current.borrow().as_ref() {
        Some(command) => write!(text, "busy \"{}\"", core::str::from_utf8(command).unwrap_or("<binary>"))?,
        None => write!(text, "idle")?,
    }
    write!(
        text,
        " queue {} sent {} lines {} errors {}\r\n",
        link.requests.len(),
        link.sent.get(),
        link.lines.get(),
        link.errors.get(),
    )
}

fn split_command(line: &[u8]) -> (&[u8], &[u8]) {
    match line.iter().position(|&byte| byte == b' ') {
        Some(at) => (&line[..at], &line[at + 1..]),
        None => (line, &[]),
    }
}

fn reply(text: &[u8]) -> Text {
    Text::from_slice(text).expect("reply fits buffer")
}

fn rejected(link: &Link, rejection: &Rejection<'_>) -> Text {
    let mut text: String<512> = String::new();
    match write!(text, "rejected on {}: {}", link.device.name(), rejection) {
        Ok(()) => reply(text.as_bytes()),
        Err(_) => reply(b"rejected (reason too long)"),
    }
}

fn allowed_words(link: &Link) -> Text {
    let mut text: String<512> = String::new();
    for word in link.allowed {
        if write!(text, "{word}\r\n").is_err() {
            return reply(b"word list overflow");
        }
    }
    reply(text.trim_end().as_bytes())
}
