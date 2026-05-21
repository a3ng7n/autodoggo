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
use embassy_sync::{blocking_mutex::raw::CriticalSectionRawMutex, signal::Signal};
use embassy_time::{Duration, Timer};
use esp_println::println;
use esp_hal::{
    Blocking,
    analog::adc::AdcPin,
    clock::CpuClock,
    gpio::{DriveMode, Input, InputConfig, Pull},
    ledc::{
        LSGlobalClkSource, Ledc, LowSpeed,
        channel::{self, ChannelIFace},
        timer::{self, TimerIFace},
    },
    peripherals::{ADC1, GPIO34},
    time::Rate,
};
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

const DEBOUNCE_MS: u64 = 50;

// GPIO18 = PWM output A, GPIO27 = PWM output B
// GPIO19 = ramp button, GPIO21 = pin-select button
#[derive(Clone, Copy)]
enum RampDir { Up, Down }

static RAMP_CMD: Signal<CriticalSectionRawMutex, RampDir> = Signal::new();
static PIN_SEL:  Signal<CriticalSectionRawMutex, bool>    = Signal::new();

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
    
    let btn1 = Input::new(peripherals.GPIO19, InputConfig::default().with_pull(Pull::Up));
    let btn2 = Input::new(peripherals.GPIO21, InputConfig::default().with_pull(Pull::Up));
    spawner.spawn(button1_task(btn1).unwrap());
    spawner.spawn(button2_task(btn2).unwrap());

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

    // LEDC timer in static storage so Channel can hold a 'static reference to it
    let mut ledc = Ledc::new(peripherals.LEDC);
    ledc.set_global_slow_clock(LSGlobalClkSource::APBClk);
    let lstimer0 = mk_static!(timer::Timer<'static, LowSpeed>, {
        let mut t = ledc.timer::<LowSpeed>(timer::Number::Timer0);
        t.configure(timer::config::Config {
            duty: timer::config::Duty::Duty8Bit,
            clock_source: timer::LSClockSource::APBClk,
            frequency: Rate::from_khz(1),
        }).unwrap();
        t
    });
    let mut ch_a = ledc.channel::<LowSpeed>(channel::Number::Channel0, peripherals.GPIO18);
    ch_a.configure(channel::config::Config {
        timer: lstimer0,
        duty_pct: 0,
        drive_mode: DriveMode::PushPull,
    }).unwrap();
    let mut ch_b = ledc.channel::<LowSpeed>(channel::Number::Channel1, peripherals.GPIO27);
    ch_b.configure(channel::config::Config {
        timer: lstimer0,
        duty_pct: 0,
        drive_mode: DriveMode::PushPull,
    }).unwrap();

    let mut duty: u8 = 0;
    let mut use_a = true;

    enum RampState { Idle, Up, Down }
    let mut ramp_state = RampState::Idle;

    loop {
        Timer::after(Duration::from_millis(10)).await;

        if let Some(dir) = RAMP_CMD.try_take() {
            ramp_state = match dir {
                RampDir::Up   => RampState::Up,
                RampDir::Down => RampState::Down,
            };
        }

        if let Some(sel) = PIN_SEL.try_take() {
            let prev = use_a;
            use_a = sel;
            if prev != use_a {
                if use_a {
                    ch_a.set_duty(duty).unwrap();
                    ch_b.set_duty(0).unwrap();
                } else {
                    ch_a.set_duty(0).unwrap();
                    ch_b.set_duty(duty).unwrap();
                }
            }
        }

        match ramp_state {
            RampState::Idle => {}
            RampState::Up => {
                if duty < 100 {
                    duty += 1;
                    if use_a { ch_a.set_duty(duty).unwrap(); }
                    else      { ch_b.set_duty(duty).unwrap(); }
                } else {
                    ramp_state = RampState::Idle;
                }
            }
            RampState::Down => {
                if duty > 0 {
                    duty -= 1;
                    if use_a { ch_a.set_duty(duty).unwrap(); }
                    else      { ch_b.set_duty(duty).unwrap(); }
                } else {
                    ramp_state = RampState::Idle;
                }
            }
        }
    }
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

#[embassy_executor::task]
async fn button1_task(mut button: Input<'static>) {
    loop {
        button.wait_for_falling_edge().await;
        Timer::after(Duration::from_millis(DEBOUNCE_MS)).await;
        if button.is_low() {
            RAMP_CMD.signal(RampDir::Up);
        }
        button.wait_for_rising_edge().await;
        Timer::after(Duration::from_millis(DEBOUNCE_MS)).await;
        RAMP_CMD.signal(RampDir::Down);
    }
}

#[embassy_executor::task]
async fn button2_task(mut button: Input<'static>) {
    let mut use_a = true;
    loop {
        button.wait_for_falling_edge().await;
        Timer::after(Duration::from_millis(DEBOUNCE_MS)).await;
        if button.is_low() {
            use_a = !use_a;
            PIN_SEL.signal(use_a);
        }
        button.wait_for_rising_edge().await;
        Timer::after(Duration::from_millis(DEBOUNCE_MS)).await;
    }
}
