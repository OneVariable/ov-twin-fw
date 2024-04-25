#![no_std]
#![no_main]

use defmt::info;
use embassy_executor::Spawner;
use embassy_rp::{
    bind_interrupts,
    peripherals::USB,
    usb::{self, Driver, Endpoint, Out},
};
use embassy_sync::blocking_mutex::raw::ThreadModeRawMutex;
use embassy_usb::UsbDevice;
use perpsi_icd::PingEndpoint;
use postcard_rpc::{
    define_dispatch,
    target_server::{buffers::AllBuffers, configure_usb, example_config, rpc_dispatch},
    WireHeader,
};
use static_cell::ConstInitCell;
use {defmt_rtt as _, panic_probe as _};

bind_interrupts!(struct Irqs {
    USBCTRL_IRQ => usb::InterruptHandler<USB>;
});

define_dispatch! {
    dispatcher: Dispatcher<Mutex = ThreadModeRawMutex, Driver = usb::Driver<'static, USB>>;
    PingEndpoint => blocking ping_handler,
}

static ALL_BUFFERS: ConstInitCell<AllBuffers<256, 256, 256>> =
    ConstInitCell::new(AllBuffers::new());

#[embassy_executor::main]
async fn main(spawner: Spawner) {
    info!("Start");
    let p = embassy_rp::init(Default::default());
    let driver = usb::Driver::new(p.USB, Irqs);
    let mut config = example_config();
    config.manufacturer = Some("OneVariable");
    config.product = Some("ov-twin");
    let buffers = ALL_BUFFERS.take();
    let (device, ep_in, ep_out) = configure_usb(driver, &mut buffers.usb_device, config);
    let dispatch = Dispatcher::new(&mut buffers.tx_buf, ep_in);

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

fn ping_handler(header: WireHeader, rqst: u32) -> u32 {
    info!("ping: seq - {=u32}", header.seq_no);
    rqst
}
