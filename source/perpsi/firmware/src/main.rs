#![no_std]
#![no_main]

use defmt::*;
use embassy_executor::Spawner;
use embassy_futures::join::join;
use embassy_rp::{
    adc::{self, Adc, Config as AdcConfig},
    bind_interrupts, clocks,
    dma::{self, AnyChannel},
    gpio::{Input, Level, Output, Pull},
    into_ref,
    peripherals::{PIN_16, PIN_17, PIO0, SPI0, UART0, USB},
    pio::{
        Common, Config as PioConfig, FifoJoin, Instance, Pio, PioPin, ShiftConfig, ShiftDirection,
        StateMachine,
    },
    spi::{self, Spi},
    uart::{self, BufferedUart},
    usb, Peripheral, PeripheralRef,
};
use embassy_time::{Delay, Duration, Ticker, Timer};
use embassy_usb::{
    class::cdc_acm::{CdcAcmClass, State},
    driver::EndpointError,
    Builder,
};
use embedded_hal_bus::spi::ExclusiveDevice;
use embedded_io_async::{Read, Write};
use fixed::types::U24F8;
use fixed_macro::fixed;
use lis3dh_async::Lis3dh;
use smart_leds::RGB8;
use {defmt_rtt as _, panic_probe as _};

bind_interrupts!(struct Irqs {
    PIO0_IRQ_0 => embassy_rp::pio::InterruptHandler<PIO0>;
    ADC_IRQ_FIFO => adc::InterruptHandler;
    USBCTRL_IRQ => usb::InterruptHandler<USB>;
    UART0_IRQ => uart::BufferedInterruptHandler<UART0>;
});

pub struct Ws2812<'d, P: Instance, const S: usize, const N: usize> {
    dma: PeripheralRef<'d, AnyChannel>,
    sm: StateMachine<'d, P, S>,
}

impl<'d, P: Instance, const S: usize, const N: usize> Ws2812<'d, P, S, N> {
    pub fn new(
        pio: &mut Common<'d, P>,
        mut sm: StateMachine<'d, P, S>,
        dma: impl Peripheral<P = impl dma::Channel> + 'd,
        pin: impl PioPin,
    ) -> Self {
        into_ref!(dma);

        // Setup sm0

        // prepare the PIO program
        let side_set = pio::SideSet::new(false, 1, false);
        let mut a: pio::Assembler<32> = pio::Assembler::new_with_side_set(side_set);

        const T1: u8 = 2; // start bit
        const T2: u8 = 5; // data bit
        const T3: u8 = 3; // stop bit
        const CYCLES_PER_BIT: u32 = (T1 + T2 + T3) as u32;

        let mut wrap_target = a.label();
        let mut wrap_source = a.label();
        let mut do_zero = a.label();
        a.set_with_side_set(pio::SetDestination::PINDIRS, 1, 0);
        a.bind(&mut wrap_target);
        // Do stop bit
        a.out_with_delay_and_side_set(pio::OutDestination::X, 1, T3 - 1, 0);
        // Do start bit
        a.jmp_with_delay_and_side_set(pio::JmpCondition::XIsZero, &mut do_zero, T1 - 1, 1);
        // Do data bit = 1
        a.jmp_with_delay_and_side_set(pio::JmpCondition::Always, &mut wrap_target, T2 - 1, 1);
        a.bind(&mut do_zero);
        // Do data bit = 0
        a.nop_with_delay_and_side_set(T2 - 1, 0);
        a.bind(&mut wrap_source);

        let prg = a.assemble_with_wrap(wrap_source, wrap_target);
        let mut cfg = PioConfig::default();

        // Pin config
        let out_pin = pio.make_pio_pin(pin);
        cfg.set_out_pins(&[&out_pin]);
        cfg.set_set_pins(&[&out_pin]);

        cfg.use_program(&pio.load_program(&prg), &[&out_pin]);

        // Clock config, measured in kHz to avoid overflows
        // TODO CLOCK_FREQ should come from embassy_rp
        let clock_freq = U24F8::from_num(clocks::clk_sys_freq() / 1000);
        let ws2812_freq = fixed!(800: U24F8);
        let bit_freq = ws2812_freq * CYCLES_PER_BIT;
        cfg.clock_divider = clock_freq / bit_freq;

        // FIFO config
        cfg.fifo_join = FifoJoin::TxOnly;
        cfg.shift_out = ShiftConfig {
            auto_fill: true,
            threshold: 24,
            direction: ShiftDirection::Left,
        };

        sm.set_config(&cfg);
        sm.set_enable(true);

        Self {
            dma: dma.map_into(),
            sm,
        }
    }

    pub async fn write(&mut self, colors: &[RGB8; N]) {
        // Precompute the word bytes from the colors
        let mut words = [0u32; N];
        for i in 0..N {
            let word = (u32::from(colors[i].g) << 24)
                | (u32::from(colors[i].r) << 16)
                | (u32::from(colors[i].b) << 8);
            words[i] = word;
        }

        // DMA transfer
        self.sm.tx().dma_push(self.dma.reborrow(), &words).await;

        Timer::after_micros(55).await;
    }
}

/// Input a value 0 to 255 to get a color value
/// The colours are a transition r - g - b - back to r.
fn wheel(mut wheel_pos: u8) -> RGB8 {
    wheel_pos = 255 - wheel_pos;
    if wheel_pos < 85 {
        return (255 - wheel_pos * 3, 0, wheel_pos * 3).into();
    }
    if wheel_pos < 170 {
        wheel_pos -= 85;
        return (0, wheel_pos * 3, 255 - wheel_pos * 3).into();
    }
    wheel_pos -= 170;
    (wheel_pos * 3, 255 - wheel_pos * 3, 0).into()
}

#[embassy_executor::main]
async fn main(spawner: Spawner) {
    info!("Start");
    let p = embassy_rp::init(Default::default());

    let Pio {
        mut common, sm0, ..
    } = Pio::new(p.PIO0, Irqs);

    let butts = [
        Input::new(p.PIN_0, Pull::Up),
        Input::new(p.PIN_1, Pull::Up),
        Input::new(p.PIN_2, Pull::Up),
        Input::new(p.PIN_3, Pull::Up),
        Input::new(p.PIN_18, Pull::Up),
        Input::new(p.PIN_19, Pull::Up),
        Input::new(p.PIN_20, Pull::Up),
        Input::new(p.PIN_21, Pull::Up),
    ];

    let adc = Adc::new(p.ADC, Irqs, AdcConfig::default());
    let p26 = adc::Channel::new_pin(p.PIN_26, Pull::None);

    // Common neopixel pins:
    // Thing plus: 8
    // Adafruit Feather: 16;  Adafruit Feather+RFM95: 4
    let ws2812 = Ws2812::new(&mut common, sm0, p.DMA_CH0, p.PIN_25);

    let spi = Spi::new(
        p.SPI0,
        p.PIN_6, // clk
        p.PIN_7, // mosi
        p.PIN_4, // miso
        p.DMA_CH1,
        p.DMA_CH2,
        spi::Config::default(),
    );
    // CS: GPIO5
    let bus = ExclusiveDevice::new(spi, Output::new(p.PIN_5, Level::High), Delay);

    // BRX: 17, BTX 16

    let driver = usb::Driver::new(p.USB, Irqs);

    // Create embassy-usb Config
    let mut config = embassy_usb::Config::new(0xc0de, 0xcafe);
    config.manufacturer = Some("Embassy");
    config.product = Some("USB-serial example");
    config.serial_number = Some("12345678");
    config.max_power = 100;
    config.max_packet_size_0 = 64;
    config.device_class = 0xEF;
    config.device_sub_class = 0x02;
    config.device_protocol = 0x01;
    config.composite_with_iads = true;

    // Create embassy-usb DeviceBuilder using the driver and config.
    // It needs some buffers for building the descriptors.
    let mut config_descriptor = [0; 256];
    let mut bos_descriptor = [0; 256];
    let mut control_buf = [0; 64];

    let mut state = State::new();

    let mut builder = Builder::new(
        driver,
        config,
        &mut config_descriptor,
        &mut bos_descriptor,
        &mut [], // no msos descriptors
        &mut control_buf,
    );

    // Create classes on the builder.
    let mut class = CdcAcmClass::new(&mut builder, &mut state, 64);

    // Build the builder.
    let mut usb = builder.build();

    // Run the USB device.
    let usb_fut = usb.run();

    // Do stuff with the class!
    let echo_fut = async {
        loop {
            class.wait_connection().await;
            info!("Connected");
            let _ = echo(&mut class).await;
            info!("Disconnected");
        }
    };

    spawner.must_spawn(rgb(ws2812));
    spawner.must_spawn(buttons(butts));
    spawner.must_spawn(adc_dial(adc, p26));
    spawner.must_spawn(acc_task(bus));
    spawner.must_spawn(uart_task(p.PIN_16, p.PIN_17, p.UART0));

    // Run everything concurrently.
    // If we had made everything `'static` above instead, we could do this using separate tasks instead.
    join(usb_fut, echo_fut).await;
}

#[embassy_executor::task]
async fn uart_task(txp: PIN_16, rxp: PIN_17, uart: UART0) -> ! {
    let mut tx_buf = [0u8; 16];
    let mut rx_buf = [0u8; 16];
    let uart = BufferedUart::new(
        uart,
        Irqs,
        txp,
        rxp,
        &mut tx_buf,
        &mut rx_buf,
        uart::Config::default(),
    );
    let (mut rx, mut tx) = uart.split();
    let mut scratch = [0u8; 32];

    loop {
        if let Ok(n) = rx.read(&mut scratch).await {
            defmt::println!("Uart: {=usize} bytes", n);
            let _ = tx.write_all(&scratch[..n]).await;
        }
    }
}

#[embassy_executor::task]
async fn acc_task(bus: ExclusiveDevice<Spi<'static, SPI0, spi::Async>, Output<'static>, Delay>) {
    let mut acc = Lis3dh::new_spi(bus).await.map_err(drop).unwrap();
    let mut ticker = Ticker::every(Duration::from_millis(100));
    acc.set_range(lis3dh_async::Range::G8).await.unwrap();
    loop {
        ticker.next().await;
        let acc = acc.accel_raw().await.unwrap();
        println!("ACC: {=i16},{=i16},{=i16}", acc.x, acc.y, acc.z);
    }
}

#[embassy_executor::task]
async fn adc_dial(mut adc: Adc<'static, adc::Async>, mut p26: adc::Channel<'static>) {
    let mut ticker = Ticker::every(Duration::from_millis(10));
    let mut last = 0u16;
    loop {
        ticker.next().await;
        let now = adc.read(&mut p26).await.unwrap();

        if last.abs_diff(now) > 8 {
            defmt::println!("ADC: {=u16}", now);
            last = now;
        }
    }
}

#[embassy_executor::task]
async fn buttons(butts: [Input<'static>; 8]) {
    let mut last = [true; 8];
    let mut ticker = Ticker::every(Duration::from_millis(10));
    loop {
        ticker.next().await;
        let mut now = [true; 8];
        now.iter_mut().zip(butts.iter()).for_each(|(o, i)| {
            *o = i.is_high();
        });
        if now != last {
            defmt::println!("{:?}", now);
            last = now;
        }
    }
}

#[embassy_executor::task]
async fn rgb(mut ws2812: Ws2812<'static, PIO0, 0, 24>) {
    // This is the number of leds in the string. Helpfully, the sparkfun thing plus and adafruit
    // feather boards for the 2040 both have one built in.
    const NUM_LEDS: usize = 24;
    let mut data = [RGB8::default(); NUM_LEDS];

    // Loop forever making RGB values and pushing them out to the WS2812.
    let mut ticker = Ticker::every(Duration::from_millis(10));
    loop {
        for j in 0..(256 * 5) {
            // debug!("New Colors:");
            for i in 0..NUM_LEDS {
                data[i] = wheel((((i * 256) as u16 / NUM_LEDS as u16 + j as u16) & 255) as u8);
                data[i].r >>= 4;
                data[i].g >>= 4;
                data[i].b >>= 4;
                // debug!("R: {} G: {} B: {}", data[i].r, data[i].g, data[i].b);
            }
            ws2812.write(&data).await;

            ticker.next().await;
        }
    }
}

struct Disconnected {}

impl From<EndpointError> for Disconnected {
    fn from(val: EndpointError) -> Self {
        match val {
            EndpointError::BufferOverflow => defmt::panic!("Buffer overflow"),
            EndpointError::Disabled => Disconnected {},
        }
    }
}

async fn echo<'d, T: usb::Instance + 'd>(
    class: &mut CdcAcmClass<'d, usb::Driver<'d, T>>,
) -> Result<(), Disconnected> {
    let mut buf = [0; 64];
    loop {
        let n = class.read_packet(&mut buf).await?;
        let data = &buf[..n];
        info!("data: {:x}", data);
        class.write_packet(data).await?;
    }
}
