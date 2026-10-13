#![no_std]
#![no_main]

use defmt::{info, warn, Debug2Format};
use embassy_executor::Spawner;
use embassy_futures::join::join;
use embassy_rp::bind_interrupts;
use embassy_rp::peripherals::{UART0, UART1, USB};
use embassy_rp::uart::{self, BufferedUart};
use embassy_rp::usb::{self, Driver};
use embassy_time::{with_timeout, Duration};
use embassy_usb::class::cdc_acm::{CdcAcmClass, State};
use embassy_usb::driver::EndpointError;
use embassy_usb::Builder;
use embedded_io_async::{Read, Write};
use heapless::Vec;
use {defmt_rtt as _, panic_probe as _};

bind_interrupts!(struct Irqs {
    USBCTRL_IRQ => usb::InterruptHandler<USB>;
    UART0_IRQ => uart::BufferedInterruptHandler<UART0>;
    UART1_IRQ => uart::BufferedInterruptHandler<UART1>;
});

const SPECTRADRIVE_BAUD: u32 = 9600;
const PS445_BAUD: u32 = 9600;
const DEVICE_LINE_END: &[u8] = b"\r";
const REPLY_TIMEOUT: Duration = Duration::from_secs(2);
const PACKET: usize = 64;
const PROMPT: &[u8] = b"> ";

type Console<'d> = CdcAcmClass<'d, Driver<'d, USB>>;
type Line = Vec<u8, 128>;
type Reply = Vec<u8, 256>;

struct Devices {
    spectradrive: BufferedUart,
    ps445: BufferedUart,
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
    let mut devices = Devices {
        spectradrive: BufferedUart::new(
            p.UART0,
            p.PIN_0,
            p.PIN_1,
            Irqs,
            &mut sd_tx,
            &mut sd_rx,
            uart_config(SPECTRADRIVE_BAUD),
        ),
        ps445: BufferedUart::new(
            p.UART1,
            p.PIN_4,
            p.PIN_5,
            Irqs,
            &mut ps_tx,
            &mut ps_rx,
            uart_config(PS445_BAUD),
        ),
    };

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
    join(usb.run(), serve(&mut console, &mut devices)).await;
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

async fn serve(console: &mut Console<'_>, devices: &mut Devices) -> ! {
    loop {
        console.wait_connection().await;
        info!("console connected");
        let Disconnected = session(console, devices).await;
        info!("console disconnected");
    }
}

async fn session(console: &mut Console<'_>, devices: &mut Devices) -> Disconnected {
    let mut line = Line::new();
    if let Err(disconnected) = write(console, PROMPT).await {
        return disconnected;
    }
    loop {
        if let Err(disconnected) = step(console, devices, &mut line).await {
            return disconnected;
        }
    }
}

async fn step(
    console: &mut Console<'_>,
    devices: &mut Devices,
    line: &mut Line,
) -> Result<(), Disconnected> {
    let mut packet = [0u8; PACKET];
    let len = console.read_packet(&mut packet).await?;
    for &byte in &packet[..len] {
        on_byte(console, devices, line, byte).await?;
    }
    Ok(())
}

async fn on_byte(
    console: &mut Console<'_>,
    devices: &mut Devices,
    line: &mut Line,
    byte: u8,
) -> Result<(), Disconnected> {
    match byte {
        b'\r' | b'\n' => submit(console, devices, line).await,
        0x08 | 0x7f => erase(console, line).await,
        _ => append(console, line, byte).await,
    }
}

async fn submit(
    console: &mut Console<'_>,
    devices: &mut Devices,
    line: &mut Line,
) -> Result<(), Disconnected> {
    write(console, b"\r\n").await?;
    let reply = execute(devices, line).await;
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

async fn execute(devices: &mut Devices, line: &[u8]) -> Reply {
    match split_command(line) {
        (b"", _) => Reply::new(),
        (b"nop", _) => reply(b"ok"),
        (b"sd", command) => query(&mut devices.spectradrive, command).await,
        (b"ps", command) => query(&mut devices.ps445, command).await,
        _ => reply(b"unknown command"),
    }
}

fn split_command(line: &[u8]) -> (&[u8], &[u8]) {
    match line.iter().position(|&byte| byte == b' ') {
        Some(at) => (&line[..at], &line[at + 1..]),
        None => (line, &[]),
    }
}

fn reply(text: &[u8]) -> Reply {
    Reply::from_slice(text).expect("static reply fits buffer")
}

async fn query<P: Read + Write>(port: &mut P, command: &[u8]) -> Reply {
    if let Err(error) = send(port, command).await {
        warn!("write: {}", Debug2Format(&error));
        return reply(b"write error");
    }
    match with_timeout(REPLY_TIMEOUT, receive_line(port)).await {
        Ok(Ok(line)) => line,
        Ok(Err(error)) => {
            warn!("read: {}", Debug2Format(&error));
            reply(b"read error")
        }
        Err(_) => reply(b"timeout"),
    }
}

async fn send<P: Write>(port: &mut P, command: &[u8]) -> Result<(), P::Error> {
    port.write_all(command).await?;
    port.write_all(DEVICE_LINE_END).await?;
    port.flush().await
}

async fn receive_line<P: Read>(port: &mut P) -> Result<Reply, P::Error> {
    let mut line = Reply::new();
    let mut byte = [0u8; 1];
    loop {
        port.read(&mut byte).await?;
        match byte[0] {
            b'\n' => return Ok(line),
            b'\r' => {}
            other => {
                if line.push(other).is_err() {
                    return Ok(line);
                }
            }
        }
    }
}
