#![no_std]
#![no_main]

use core::cell::{Cell, RefCell};
use core::fmt::Write as _;
use core::future::pending;

use defmt::{info, warn, Debug2Format};
use embassy_executor::Spawner;
use embassy_futures::join::join4;
use embassy_futures::select::{select, select3, Either, Either3};
use embassy_rp::bind_interrupts;
use embassy_rp::peripherals::{UART0, UART1, USB};
use embassy_rp::uart::{self, BufferedUart};
use embassy_rp::usb::{self, Driver};
use embassy_sync::blocking_mutex::raw::NoopRawMutex;
use embassy_sync::channel::Channel;
use embassy_sync::signal::Signal;
use embassy_usb::class::cdc_acm::{CdcAcmClass, State};
use embassy_usb::driver::EndpointError;
use embassy_usb::Builder;
use embedded_io_async::{Read, Write};
use heapless::{String, Vec};
use {defmt_rtt as _, panic_probe as _};

use spectractrl::command::{check, Arg, Rejection, Word};

bind_interrupts!(struct Irqs {
    USBCTRL_IRQ => usb::InterruptHandler<USB>;
    UART0_IRQ => uart::BufferedInterruptHandler<UART0>;
    UART1_IRQ => uart::BufferedInterruptHandler<UART1>;
});

const SPECTRADRIVE_BAUD: u32 = 9600;
const PS445_BAUD: u32 = 9600;
const DEVICE_LINE_END: &[u8] = b"\r";
const DONE_MARKER: &[u8] = b"ok";
const FAILED_MARKER: &[u8] = b"?";
const PACKET: usize = 64;
const PROMPT: &[u8] = b"> ";

const SPECTRADRIVE_WORDS: &[Word] = &[
    Word::new(b"WORDS", &[]),
    Word::new(b"WHO", &[]),
    Word::new(b"GOTO", &[Arg::Decimal]),
    Word::new(b"NM", &[Arg::Decimal]),
    Word::new(b"?NM", &[]),
    Word::new(b"GRATING", &[Arg::Integer]),
    Word::new(b"?GRATING", &[]),
];

const PS445_WORDS: &[Word] = &[
    Word::new(b"WORDS", &[]),
    Word::new(b"WHO", &[]),
    Word::new(b".READ", &[]),
    Word::new(b"PMT-ON", &[]),
    Word::new(b"PMT-OFF", &[]),
    Word::new(b"VOLTS", &[Arg::Integer]),
];

type Console<'d> = CdcAcmClass<'d, Driver<'d, USB>>;
type Line = Vec<u8, 128>;
type Text = Vec<u8, 512>;
type Requests = Channel<NoopRawMutex, Line, 4>;
type Events = Channel<NoopRawMutex, Event, 8>;

#[derive(Clone, Copy, defmt::Format)]
enum Device {
    SpectraDrive,
    Ps445,
}

impl Device {
    fn name(self) -> &'static str {
        match self {
            Device::SpectraDrive => "sd",
            Device::Ps445 => "ps",
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

struct Link {
    device: Device,
    allowed: &'static [Word],
    requests: Requests,
    abort: Signal<NoopRawMutex, ()>,
    current: RefCell<Option<Line>>,
    sent: Cell<u32>,
    lines: Cell<u32>,
    errors: Cell<u32>,
}

impl Link {
    fn new(device: Device, allowed: &'static [Word]) -> Self {
        Self {
            device,
            allowed,
            requests: Channel::new(),
            abort: Signal::new(),
            current: RefCell::new(None),
            sent: Cell::new(0),
            lines: Cell::new(0),
            errors: Cell::new(0),
        }
    }

    fn is_busy(&self) -> bool {
        self.current.borrow().is_some()
    }
}

struct Links<'a> {
    spectradrive: &'a Link,
    ps445: &'a Link,
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
}

enum Input {
    Request(Line),
    Abort,
    Byte(u8),
    Error(uart::Error),
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
    let events = Events::new();
    let links = Links { spectradrive: &sd, ps445: &ps };

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
    join4(
        usb.run(),
        serve(&mut console, &links, &events),
        worker(&sd, spectradrive, &events),
        worker(&ps, ps445, &events),
    )
    .await;
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
    loop {
        match next_input(link, &mut port).await {
            Input::Request(command) => start(link, &mut port, command, events).await,
            Input::Abort => abort(link, events),
            Input::Byte(byte) => on_device_byte(link, &mut line, byte, events),
            Input::Error(error) => {
                warn!("{} rx: {}", link.device, Debug2Format(&error));
                bump(&link.errors);
            }
        }
    }
}

async fn next_input(link: &Link, port: &mut BufferedUart) -> Input {
    let mut byte = [0u8; 1];
    let request = async {
        match link.is_busy() {
            true => pending::<Line>().await,
            false => link.requests.receive().await,
        }
    };
    let input = select3(request, link.abort.wait(), port.read(&mut byte)).await;
    match input {
        Either3::First(command) => Input::Request(command),
        Either3::Second(()) => Input::Abort,
        Either3::Third(Ok(_)) => Input::Byte(byte[0]),
        Either3::Third(Err(error)) => Input::Error(error),
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

fn abort(link: &Link, events: &Events) {
    if let Some(command) = link.current.take() {
        publish(events, link.device, Kind::Aborted, &command);
    }
}

fn on_device_byte(link: &Link, line: &mut Text, byte: u8, events: &Events) {
    defmt::trace!("{} rx {=[u8]:a}", link.device, &[byte][..]);
    match byte {
        0 | b'\r' => {}
        b'\n' => complete_line(link, line, events),
        _ => {
            if line.is_full() {
                complete_line(link, line, events);
            }
            line.push(byte).expect("line has room after completion");
        }
    }
}

fn complete_line(link: &Link, line: &mut Text, events: &Events) {
    let text = line.trim_ascii();
    if !text.is_empty() {
        bump(&link.lines);
        let kind = classify(link.is_busy(), text);
        if kind != Kind::Output {
            *link.current.borrow_mut() = None;
        }
        publish(events, link.device, kind, text);
    }
    line.clear();
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

async fn serve(console: &mut Console<'_>, links: &Links<'_>, events: &Events) -> ! {
    loop {
        console.wait_connection().await;
        info!("console connected");
        let Disconnected = session(console, links, events).await;
        info!("console disconnected");
    }
}

async fn session(console: &mut Console<'_>, links: &Links<'_>, events: &Events) -> Disconnected {
    let mut line = Line::new();
    if let Err(disconnected) = write(console, PROMPT).await {
        return disconnected;
    }
    loop {
        if let Err(disconnected) = step(console, links, events, &mut line).await {
            return disconnected;
        }
    }
}

async fn step(
    console: &mut Console<'_>,
    links: &Links<'_>,
    events: &Events,
    line: &mut Line,
) -> Result<(), Disconnected> {
    let mut packet = [0u8; PACKET];
    let input = select(console.read_packet(&mut packet), events.receive()).await;
    match input {
        Either::First(received) => {
            let len = received?;
            for &byte in &packet[..len] {
                on_byte(console, links, line, byte).await?;
            }
            Ok(())
        }
        Either::Second(event) => announce(console, &event, line).await,
    }
}

async fn announce(console: &mut Console<'_>, event: &Event, line: &Line) -> Result<(), Disconnected> {
    write(console, b"\r\x1b[K[").await?;
    write(console, event.device.name().as_bytes()).await?;
    write(console, event.kind.label()).await?;
    write(console, &event.text).await?;
    write(console, b"\r\n").await?;
    write(console, PROMPT).await?;
    write(console, line).await
}

async fn on_byte(
    console: &mut Console<'_>,
    links: &Links<'_>,
    line: &mut Line,
    byte: u8,
) -> Result<(), Disconnected> {
    match byte {
        b'\r' | b'\n' => submit(console, links, line).await,
        0x08 | 0x7f => erase(console, line).await,
        _ => append(console, line, byte).await,
    }
}

async fn submit(console: &mut Console<'_>, links: &Links<'_>, line: &mut Line) -> Result<(), Disconnected> {
    write(console, b"\r\n").await?;
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
        (b"", _) => Text::new(),
        (b"nop", _) => reply(b"ok"),
        (b"status", _) => status(links),
        (b"allowed", target) => match links.find(target) {
            Some(link) => allowed_words(link),
            None => reply(b"usage: allowed sd|ps"),
        },
        (b"abort", target) => match links.find(target) {
            Some(link) => {
                link.abort.signal(());
                reply(b"abort requested")
            }
            None => reply(b"usage: abort sd|ps"),
        },
        (b"sd", command) => enqueue(links.spectradrive, command),
        (b"ps", command) => enqueue(links.ps445, command),
        _ => reply(b"unknown command"),
    }
}

fn enqueue(link: &Link, command: &[u8]) -> Text {
    if let Err(rejection) = check(command, link.allowed) {
        return rejected(link, &rejection);
    }
    let command = Line::from_slice(command).expect("command is part of a line, so it fits");
    match link.requests.try_send(command) {
        Ok(()) if link.is_busy() => reply(b"queued (device busy)"),
        Ok(()) => reply(b"queued"),
        Err(_) => reply(b"queue full"),
    }
}

fn status(links: &Links<'_>) -> Text {
    let mut text: String<512> = String::new();
    for link in links.all() {
        if describe(link, &mut text).is_err() {
            return reply(b"status overflow");
        }
    }
    reply(text.trim_end().as_bytes())
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
