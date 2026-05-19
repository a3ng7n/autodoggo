#![no_std]
#![no_main]
#![deny(
    clippy::mem_forget,
    reason = "mem::forget is generally not safe to do with esp_hal types, especially those \
    holding buffers for the duration of a data transfer."
)]
#![deny(clippy::large_stack_frames)]

use defmt::{error, info};
use embassy_executor::Spawner;
use embassy_net::{
    dns::DnsSocket,
    tcp::client::{TcpClient, TcpClientState},
    Runner, Stack, StackResources,
};
use embassy_time::{Duration, Timer};
use esp_println::println;
use esp_hal::{Blocking, analog::adc::AdcPin, clock::CpuClock, peripherals::{ADC1, GPIO34}};
use esp_hal::rng::Rng;
use esp_hal::timer::timg::TimerGroup;
use esp_radio::wifi::{
    scan::ScanConfig, sta::StationConfig, Config, ControllerConfig, Interface, WifiController,
};
use esp_println as _;
use reqwless::{
    client::HttpClient,
    request::{Method, RequestBuilder},
};

use esp_hal::analog::adc::{Adc, AdcConfig, Attenuation};


#[panic_handler]
fn panic(panic_info: &core::panic::PanicInfo) -> ! {
    error!("{}", panic_info);
    loop {}
}

const SSID: &str = env!("SSID");
const PASSWORD: &str = env!("PASSWORD");

extern crate alloc;

// When you are okay with using a nightly compiler it's better to use https://docs.rs/static_cell/2.1.0/static_cell/macro.make_static.html
macro_rules! mk_static {
    ($t:ty,$val:expr) => {{
        static STATIC_CELL: static_cell::StaticCell<$t> = static_cell::StaticCell::new();
        #[deny(unused_attributes)]
        let x = STATIC_CELL.uninit().write(($val));
        x
    }};
}

// This creates a default app-descriptor required by the esp-idf bootloader.
// For more information see: <https://docs.espressif.com/projects/esp-idf/en/stable/esp32/api-reference/system/app_image_format.html#application-description>
esp_bootloader_esp_idf::esp_app_desc!();

#[allow(
    clippy::large_stack_frames,
    reason = "it's not unusual to allocate larger buffers etc. in main"
)]
#[esp_rtos::main]
async fn main(spawner: Spawner) -> ! {
    // generator version: 1.3.0
    // generator parameters: --chip esp32 -o unstable-hal -o alloc -o wifi -o embassy -o defmt

    let config = esp_hal::Config::default().with_cpu_clock(CpuClock::max());
    let peripherals = esp_hal::init(config);

    esp_alloc::heap_allocator!(#[esp_hal::ram(reclaimed)] size: 98768);

    let timg0 = TimerGroup::new(peripherals.TIMG0);
    let sw_interrupt =
        esp_hal::interrupt::software::SoftwareInterruptControl::new(peripherals.SW_INTERRUPT);
    esp_rtos::start(timg0.timer0, sw_interrupt.software_interrupt0);

    let station_config = Config::Station(
        StationConfig::default()
            .with_ssid(SSID)
            .with_password(PASSWORD.into()),
    );

    info!("Embassy initialized!");

    let (mut wifi_controller, interfaces) =
        esp_radio::wifi::new(peripherals.WIFI, ControllerConfig::default().with_initial_config(station_config))
            .expect("Failed to initialize Wi-Fi controller");

    let wifi_interface = interfaces.station;
    let config = embassy_net::Config::dhcpv4(Default::default());

    let rng = Rng::new();
    let net_seed = rng.random() as u64 | ((rng.random() as u64) << 32);
    let tls_seed = rng.random() as u64 | ((rng.random() as u64) << 32);

    let (stack, runner) = embassy_net::new(
        wifi_interface,
        config,
        mk_static!(StackResources<3>, StackResources::<3>::new()),
        net_seed,
    );

    println!("Scanning for access points");
    let scan_config = ScanConfig::default().with_max(10);
    let result = wifi_controller.scan_async(&scan_config).await.unwrap();
    for ap in result {
        println!("{:?}", ap);
    }

    spawner.spawn(connection(wifi_controller).unwrap());
    spawner.spawn(net_task(runner).unwrap());

    wait_for_connection(stack).await;

    let tcp_client = TcpClient::new(
        stack,
        mk_static!(
            TcpClientState<1, 1500, 1500>,
            TcpClientState::<1, 1500, 1500>::new()
        ),
    );
    let dns_client = DnsSocket::new(stack);

    let adc_pin = peripherals.GPIO34;
    let mut adc1_config = AdcConfig::new();
    let mut pin = adc1_config.enable_pin(adc_pin, Attenuation::_11dB);
    let mut adc1 = Adc::new(peripherals.ADC1, adc1_config);

    spawner.spawn(adc_read(adc1, pin).unwrap());

    loop {
        println!("Making HTTP request");

        let mut client = HttpClient::new(&tcp_client, &dns_client);
        let mut rx_buf = [0u8; 4096];

        let request = client
            .request(Method::GET, "http://www.mobile-j.de/")
            .await
            .unwrap();
        let mut request = request.headers(&[("Connection", "close")]);

        let response = request.send(&mut rx_buf).await.unwrap();
        match response.body().read_to_end().await {
            Ok(data) => {
                if let Ok(body) = core::str::from_utf8(data) {
                    println!("Body: {}", body);
                }
            }
            Err(err) => println!("Body error: {:?}", err),
        }

        Timer::after(Duration::from_secs(5)).await;
    }

    // for inspiration have a look at the examples at https://github.com/esp-rs/esp-hal/tree/esp-hal-v1.1.0/examples
}

#[embassy_executor::task]
async fn adc_read(mut adc1: Adc<'static, ADC1<'static>, Blocking>, mut pin: AdcPin<GPIO34<'static>, ADC1<'static>>) {
    loop {
        if let Ok(value) = adc1.read_oneshot(&mut pin) {
            println!("ADC value: {}", value);
        }
        Timer::after(Duration::from_millis(100)).await;
    }
}

async fn wait_for_connection(stack: Stack<'_>) {
    stack.wait_config_up().await;
    if let Some(config) = stack.config_v4() {
        println!("Got IP: {}", config.address);
    }
}

#[embassy_executor::task]
async fn connection(mut controller: WifiController<'static>) {
    loop {
        println!("Connecting to Wi-Fi...");

        match controller.connect_async().await {
            Ok(info) => {
                println!("Wi-Fi connected to {:?}", info);
                let info = controller.wait_for_disconnect_async().await.ok();
                println!("Disconnected: {:?}", info);
            }
            Err(err) => println!("Failed to connect to Wi-Fi: {:?}", err),
        }

        Timer::after(Duration::from_secs(5)).await;
    }
}

#[embassy_executor::task]
async fn net_task(mut runner: Runner<'static, Interface<'static>>) {
    runner.run().await
}
