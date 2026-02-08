#![no_std]
#![no_main]

use embassy_executor::Spawner;
use embassy_time::{Duration, Timer};
use esp_alloc as _;
use esp_backtrace as _;
use esp_hal::{
    clock::CpuClock,
    dma::{DmaRxBuf, DmaTxBuf},
    dma_buffers,
    gpio::{Level, Output, OutputConfig, Pull},
    interrupt::software::SoftwareInterruptControl,
    ram,
    spi::{
        Mode,
        master::{self, SpiDmaBus},
    },
    time::Rate,
    timer::timg::TimerGroup,
};
use esp_println::println;

// for UsbSerialJtag
use embedded_io_async::Read;

use esp_hal::usb_serial_jtag::UsbSerialJtag;

esp_bootloader_esp_idf::esp_app_desc!();

const S_CMD_NOP: u8 = 0x00;
const S_CMD_Q_IFACE: u8 = 0x01;
const S_CMD_Q_CMDS: u8 = 0x02;
const S_CMD_Q_PGMNAME: u8 = 0x03;
const S_CMD_Q_SERBUF: u8 = 0x04;
const S_CMD_Q_BUSTYPE: u8 = 0x05;
const S_CMD_Q_WRNMAXLEN: u8 = 0x08;
const S_CMD_SYNCNOP: u8 = 0x10;
const S_CMD_Q_RDNMAXLEN: u8 = 0x11;
const S_CMD_S_BUSTYPE: u8 = 0x12;
const S_CMD_O_SPIOP: u8 = 0x13;
const S_CMD_S_SPIFREQ: u8 = 0x14;
const S_CMD_S_PINSTATE: u8 = 0x15;

const S_ACK: u8 = 0x06;
const S_NAK: u8 = 0x15;

const BUF_SIZE: usize = 4096;
const DEFAULT_SPI_CLOCK_HZ: u32 = 1_000_000;

struct EspSerProg<'a> {
    spi: SpiDmaBus<'a, esp_hal::Async>,
    usb_serial: UsbSerialJtag<'a, esp_hal::Async>,
    buffer_en: Output<'a>,
    power_en: Output<'a>,

    spi_clock_hz: u32,
}

impl<'a> EspSerProg<'a> {
    pub fn new(
        spi: SpiDmaBus<'a, esp_hal::Async>,
        usb_serial: UsbSerialJtag<'a, esp_hal::Async>,
        buffer_en: Output<'a>,
        power_en: Output<'a>,
    ) -> Self {
        Self {
            spi,
            usb_serial,
            spi_clock_hz: DEFAULT_SPI_CLOCK_HZ,
            buffer_en,
            power_en,
        }
    }

    pub async fn read_exact(&mut self, buf: &mut [u8]) -> Result<(), core::convert::Infallible> {
        let mut bytes_read = 0;

        while bytes_read != buf.len() {
            let res = self.usb_serial.read(&mut buf[bytes_read..]).await?;
            bytes_read += res;
        }

        Ok(())
    }

    pub fn write_all(&mut self, buf: &[u8]) -> Result<(), core::convert::Infallible> {
        self.usb_serial.write(buf)
    }

    #[ram]
    pub async fn run(&mut self) {
        loop {
            let mut cmd: [u8; 1] = [0];
            if self.read_exact(&mut cmd).await.is_err() {
                println!("read error on cmd. EOF?");
                continue;
            }
            // Double the buffer size to accommodate both write and read data
            let mut spi_buf: [u8; BUF_SIZE * 2] = [0; BUF_SIZE * 2];
            let mut resp: &[u8] = &[S_NAK]; // NAK by default

            // Echo back
            match cmd[0] {
                S_CMD_NOP => {
                    // NOP, Send back ACK
                    resp = &[S_ACK];
                }
                S_CMD_Q_IFACE => {
                    // Query programmer version. ACK + 16 bit version
                    resp = &[S_ACK, 0x01, 0x00]; // Version 1
                }
                S_CMD_Q_CMDS => {
                    // Query supported commands. ACK + 32 bytes (16 bits) of supported cmd
                    // flags
                    resp = &[
                        S_ACK, // ACK
                        0x3f, 0x01, 0xFF, 0x03, // CMDs 0x00-0x07, 0x08, 0x10 - 0x18
                        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
                        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
                        0x00, 0x00, 0x00, 0x00,
                    ];
                }
                S_CMD_Q_PGMNAME => {
                    // Query programmer name. ACK + 16 bytes, NULL-padded string
                    resp = &[
                        S_ACK, // ACK
                        b'e', b's', b'p', b'-', b's', b'e', b'r', b'p', b'r', b'o', b'g', 0x0, 0x0,
                        0x0, 0x0, 0x0,
                    ];
                }
                S_CMD_Q_SERBUF => {
                    // Query serial buffer size. ACK + 2 bytes size
                    resp = &[
                        S_ACK,
                        (BUF_SIZE & 0xff) as u8,
                        ((BUF_SIZE >> 8) & 0xff) as u8,
                    ];
                }
                S_CMD_Q_BUSTYPE => {
                    // Query supported bus types. ACK + 1 byte, (1 << 3) is SPI
                    resp = &[S_ACK, 1 << 3]; // Only SPI
                }
                S_CMD_Q_WRNMAXLEN => {
                    // Query max write-n lenght, ACK + 3 bytes length
                    resp = &[
                        S_ACK,
                        (BUF_SIZE & 0xff) as u8,
                        ((BUF_SIZE >> 8) & 0xff) as u8,
                        0x00,
                    ];
                }
                S_CMD_SYNCNOP => {
                    // Sync NOP. NAK + ACK
                    resp = &[S_NAK, S_ACK];
                }
                S_CMD_Q_RDNMAXLEN => {
                    // Query max read-n length
                    resp = &[
                        S_ACK,
                        (BUF_SIZE & 0xff) as u8,
                        ((BUF_SIZE >> 8) & 0xff) as u8,
                        0x00,
                    ];
                }
                S_CMD_S_BUSTYPE => {
                    // Set bus type
                    // Argument: 8 bit
                    // Response: ACK
                    let mut arg: [u8; 1] = [0];
                    if self.read_exact(&mut arg).await.is_err() {
                        println!("read error");
                        continue;
                    }

                    resp = &[S_ACK];
                }
                S_CMD_O_SPIOP => {
                    // Perform SPI operation
                    // Arguments: 3 bytes: write_len, 3 bytes: read_len, write_len bytes of data
                    // Response: ACK + read_len bytes of data
                    let mut arg: [u8; 6] = [0; 6];
                    if self.read_exact(&mut arg).await.is_err() {
                        println!("read error");
                        continue;
                    }
                    let write_len =
                        arg[0] as usize | ((arg[1] as usize) << 8) | ((arg[2] as usize) << 16);
                    let read_len =
                        arg[3] as usize | ((arg[4] as usize) << 8) | ((arg[5] as usize) << 16);

                    if write_len > BUF_SIZE || read_len > BUF_SIZE {
                        println!("argument is too large");
                        resp = &[S_NAK]; // NAK
                    } else {
                        if self.read_exact(&mut spi_buf[0..write_len]).await.is_err() {
                            println!("read error");
                            continue;
                        }
                        self.spi
                            .transfer_in_place_async(&mut spi_buf[0..(write_len + read_len)])
                            .await
                            .unwrap();
                        spi_buf[write_len - 1] = S_ACK;
                        resp = &spi_buf[write_len - 1..write_len + read_len];
                    }
                }
                S_CMD_S_SPIFREQ => {
                    // Set SPI clock speed
                    // Arguments: 4 bytes: requested speed in Hz (little endian)
                    let mut arg: [u8; 4] = [0; 4];
                    if self.read_exact(&mut arg).await.is_err() {
                        println!("read error");
                        continue;
                    }

                    let speed = arg[0] as u32
                        | ((arg[1] as u32) << 8)
                        | ((arg[2] as u32) << 16)
                        | ((arg[3] as u32) << 24);

                    self.spi_clock_hz = speed;

                    let config = master::Config::default()
                        .with_frequency(Rate::from_hz(speed))
                        .with_mode(Mode::_0);
                    self.spi.apply_config(&config).unwrap();

                    spi_buf[0] = S_ACK;
                    spi_buf[1] = arg[0];
                    spi_buf[2] = arg[1];
                    spi_buf[3] = arg[2];
                    spi_buf[4] = arg[3];
                    resp = &spi_buf[0..5];
                }
                S_CMD_S_PINSTATE => {
                    // Enable chip drivers
                    // Argument: 8 bit, 0 - disable, otherwise enable
                    // Response: ACK
                    let mut arg: [u8; 1] = [0];
                    if self.read_exact(&mut arg).await.is_err() {
                        println!("read error");
                        continue;
                    }
                    if arg[0] != 0 {
                        //println!("enable drivers");
                        self.power_en.set_low();
                        self.buffer_en.set_low();
                        Timer::after(Duration::from_millis(200)).await;
                    } else {
                        //println!("disable drivers");
                        self.buffer_en.set_high();
                        self.power_en.set_high();
                        Timer::after(Duration::from_millis(200)).await;
                    }

                    resp = &[S_ACK];
                }
                _ => {
                    println!("Unhandled command!");
                }
            }

            if self.write_all(resp).is_err() {
                println!("write error");
                continue;
            }
        }
    }
}

#[esp_rtos::main]
async fn main(_spawner: Spawner) -> ! {
    esp_println::logger::init_logger_from_env();
    let config = esp_hal::Config::default().with_cpu_clock(CpuClock::max());
    let peripherals = esp_hal::init(config);

    // for alloc
    esp_alloc::heap_allocator!(#[ram(reclaimed)] size: 64 * 1024);
    esp_alloc::heap_allocator!(size: 36 * 1024);

    let timg0 = TimerGroup::new(peripherals.TIMG0);
    let sw_int = SoftwareInterruptControl::new(peripherals.SW_INTERRUPT);
    esp_rtos::start(timg0.timer0, sw_int.software_interrupt0);

    let (dma_rx_buffer, rx_descriptors, dma_tx_buffer, tx_descriptors) = dma_buffers!(8192);
    let dma_rx_buf = DmaRxBuf::new(rx_descriptors, dma_rx_buffer).unwrap();
    let dma_tx_buf = DmaTxBuf::new(tx_descriptors, dma_tx_buffer).unwrap();

    let spi = master::Spi::new(
        peripherals.SPI2,
        master::Config::default()
            .with_frequency(Rate::from_hz(DEFAULT_SPI_CLOCK_HZ))
            .with_mode(Mode::_0),
    )
    .unwrap()
    .with_sck(peripherals.GPIO7)
    .with_miso(peripherals.GPIO6)
    .with_mosi(peripherals.GPIO5)
    .with_cs(peripherals.GPIO10)
    .with_dma(peripherals.DMA_CH0)
    .with_buffers(dma_rx_buf, dma_tx_buf)
    .into_async();

    let usb_serial = UsbSerialJtag::new(peripherals.USB_DEVICE).into_async();

    let config = OutputConfig::default().with_pull(Pull::Up);

    let power_en = Output::new(peripherals.GPIO0, Level::High, config);
    let buffer_en = Output::new(peripherals.GPIO1, Level::High, config);

    let mut prog = EspSerProg::new(spi, usb_serial, buffer_en, power_en);

    // would never return
    prog.run().await;

    panic!("Should never reach here");
}
