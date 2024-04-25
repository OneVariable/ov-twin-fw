#![no_std]
#![no_main]

use defmt::info;
use embassy_executor::Spawner;
use embassy_rp::{
    bind_interrupts, flash::{Blocking, Flash}, peripherals::{FLASH, PIO0, USB}, pio::Pio, usb::{self, Driver, Endpoint, Out}
};
use embassy_sync::blocking_mutex::raw::ThreadModeRawMutex;
use embassy_usb::UsbDevice;
use perpsi_icd::{BadPositionError, GetUniqueIdEndpoint, PingEndpoint, Rgb8, SetAllLedEndpoint, SetSingleLedEndpoint, SingleLed};
use postcard_rpc::{
    define_dispatch,
    target_server::{buffers::AllBuffers, configure_usb, example_config, rpc_dispatch},
    WireHeader,
};
use smart_leds::{colors::BLACK, RGB8};
use static_cell::ConstInitCell;
use crate::ws2812::Ws2812;

use {defmt_rtt as _, panic_probe as _};

mod ws2812;

bind_interrupts!(struct Irqs {
    USBCTRL_IRQ => usb::InterruptHandler<USB>;
});

pub struct PerpsiContext {
    pub unique_id: u64,
    pub ws2812: Ws2812<'static, PIO0, 0, 24>,
    pub ws2812_state: [RGB8; 24],
}

define_dispatch! {
    dispatcher: Dispatcher<Mutex = ThreadModeRawMutex, Driver = usb::Driver<'static, USB>, Context = PerpsiContext>;
    PingEndpoint => blocking ping_handler,
    GetUniqueIdEndpoint => blocking unique_id_handler,
    SetSingleLedEndpoint => async set_led_handler,
    SetAllLedEndpoint => async set_all_led_handler,
}

static ALL_BUFFERS: ConstInitCell<AllBuffers<256, 256, 256>> =
    ConstInitCell::new(AllBuffers::new());

#[embassy_executor::main]
async fn main(spawner: Spawner) {
    // SYSTEM INIT
    info!("Start");
    let mut p = embassy_rp::init(Default::default());
    let unique_id = get_unique_id(&mut p.FLASH).unwrap();
    info!("id: {=u64:016X}", unique_id);

    // PIO/WS2812 INIT
    let Pio {
        mut common, sm0, ..
    } = Pio::new(p.PIO0, ws2812::Irqs);

    // Common neopixel pins:
    // Thing plus: 8
    // Adafruit Feather: 16;  Adafruit Feather+RFM95: 4
    let ws2812: Ws2812<'static, PIO0, 0, 24> = Ws2812::new(&mut common, sm0, p.DMA_CH0, p.PIN_25);

    // USB/RPC INIT
    let driver = usb::Driver::new(p.USB, Irqs);
    let mut config = example_config();
    config.manufacturer = Some("OneVariable");
    config.product = Some("ov-twin");
    let buffers = ALL_BUFFERS.take();
    let (device, ep_in, ep_out) = configure_usb(driver, &mut buffers.usb_device, config);
    let dispatch = Dispatcher::new(
        &mut buffers.tx_buf,
        ep_in,
        PerpsiContext { unique_id, ws2812, ws2812_state: [BLACK; 24] },
    );

    spawner.must_spawn(dispatch_task(ep_out, dispatch, &mut buffers.rx_buf));
    spawner.must_spawn(usb_task(device));
}

/// This actually runs the dispatcher
#[embassy_executor::task]
async fn dispatch_task(
    ep_out: Endpoint<'static, USB, Out>,
    dispatch: Dispatcher,
    rx_buf: &'static mut [u8],
) {
    rpc_dispatch(ep_out, dispatch, rx_buf).await;
}

/// This handles the low level USB management
#[embassy_executor::task]
pub async fn usb_task(mut usb: UsbDevice<'static, Driver<'static, USB>>) {
    usb.run().await;
}

// ---

fn ping_handler(_context: &mut PerpsiContext, header: WireHeader, rqst: u32) -> u32 {
    info!("ping: seq - {=u32}", header.seq_no);
    rqst
}

fn unique_id_handler(context: &mut PerpsiContext, header: WireHeader, _rqst: ()) -> u64 {
    info!("unique_id: seq - {=u32}", header.seq_no);
    context.unique_id
}

async fn set_led_handler(context: &mut PerpsiContext, header: WireHeader, rqst: SingleLed) -> Result<(), BadPositionError> {
    info!("set_led: seq - {=u32}", header.seq_no);
    if rqst.position >= 24 {
        return Err(BadPositionError);
    }
    let pos = rqst.position as usize;
    context.ws2812_state[pos] = RGB8 { r: rqst.rgb.r, g: rqst.rgb.g, b: rqst.rgb.b};
    context.ws2812.write(&context.ws2812_state).await;
    Ok(())
}

async fn set_all_led_handler(context: &mut PerpsiContext, header: WireHeader, rqst: [Rgb8; 24]) {
    info!("set_all_led: seq - {=u32}", header.seq_no);
    context.ws2812_state.iter_mut().zip(rqst.iter()).for_each(|(s, rgb)| {
        s.r = rgb.r;
        s.g = rgb.g;
        s.b = rgb.b;
    });
    context.ws2812.write(&context.ws2812_state).await;
}

pub fn get_unique_id(flash: &mut FLASH) -> Option<u64> {
    let mut flash: Flash<'_, FLASH, Blocking, { 2 * 1024 * 1024 }> = Flash::new_blocking(flash);

    // TODO: For different flash chips, we want to handle things
    // differently based on their jedec? That being said: I control
    // the hardware for this project, and both chips (Pico and XIAO)
    // support unique ID, so oh well.
    //
    // let jedec = flash.blocking_jedec_id().unwrap();

    let mut id = [0u8; core::mem::size_of::<u64>()];
    flash.blocking_unique_id(&mut id).unwrap();
    Some(u64::from_be_bytes(id))
}
