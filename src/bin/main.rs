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
use embassy_time::{Duration, Instant, Timer};
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
    peripherals::{ADC1, GPIO33},
    time::Rate,
};
use esp_hal::rng::Rng;
use esp_hal::timer::timg::TimerGroup;
use esp_radio::wifi::{
    scan::ScanConfig, sta::StationConfig, Config, ControllerConfig, Interface, WifiController,
};
use esp_println as _;
use esp_backtrace as _;
use reqwless::{
    client::HttpClient,
    request::{Method, RequestBuilder},
};

use esp_hal::analog::adc::{Adc, AdcConfig, Attenuation};

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
const DOOR_TIMEOUT_MS: u64 = 10_000;
const PULLEY_RAMP_STEP_MS: u64 = 10;
const OVERCURRENT_THRESHOLD: u16 = 1024;
const PULLEY_DUTY_MAX: u8 = 100;

#[derive(Clone, Copy, Debug, PartialEq)]
enum DoorCommand { Open, Close }

#[derive(Clone, Copy, Debug, PartialEq)]
enum DoorState { Idle, Opening, Opened, Closing, Closed }

#[derive(Clone, Copy, Debug, PartialEq)]
enum PulleyCommand { RampUpCCW, RampUpCW, Stop }

#[derive(Clone, Copy, Debug, PartialEq)]
enum PulleyState { Hold, RampingUpCCW, DrivingCCW, RampingDownCCW, RampingUpCW, DrivingCW, RampingDownCW }

static DOOR_CMD: Signal<CriticalSectionRawMutex, DoorCommand> = Signal::new();
static PULLEY_CMD: Signal<CriticalSectionRawMutex, PulleyCommand> = Signal::new();
static OPEN_LIMIT_HIT: Signal<CriticalSectionRawMutex, ()> = Signal::new();
static CLOSE_LIMIT_HIT: Signal<CriticalSectionRawMutex, ()> = Signal::new();
static OVERCURRENT_DETECTED: Signal<CriticalSectionRawMutex, ()> = Signal::new();

type AdcPinType = GPIO33<'static>;

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
    
    let adc_pin: AdcPinType = peripherals.GPIO33;
    let mut adc1_config = AdcConfig::new();
    let mut pin = adc1_config.enable_pin(adc_pin, Attenuation::_11dB);
    let mut adc1 = Adc::new(peripherals.ADC1, adc1_config);

    let btn1 = Input::new(peripherals.GPIO19, InputConfig::default().with_pull(Pull::Up));
    let btn2 = Input::new(peripherals.GPIO21, InputConfig::default().with_pull(Pull::Up));
    let open_limit = Input::new(peripherals.GPIO16, InputConfig::default().with_pull(Pull::Up));
    let close_limit = Input::new(peripherals.GPIO17, InputConfig::default().with_pull(Pull::Up));

    spawner.spawn(open_button_task(btn1).unwrap());
    spawner.spawn(close_button_task(btn2).unwrap());
    spawner.spawn(limit_switch_task(open_limit, &OPEN_LIMIT_HIT).unwrap());
    spawner.spawn(limit_switch_task(close_limit, &CLOSE_LIMIT_HIT).unwrap());
    spawner.spawn(door_state_machine_task().unwrap());

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

    // spawner.spawn(adc_read(adc1, pin).unwrap());

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
    let mut ch_b = ledc.channel::<LowSpeed>(channel::Number::Channel1, peripherals.GPIO5);
    ch_b.configure(channel::config::Config {
        timer: lstimer0,
        duty_pct: 0,
        drive_mode: DriveMode::PushPull,
    }).unwrap();

    spawner.spawn(overcurrent_monitor_task(adc1, pin).unwrap());
    spawner.spawn(pulley_driver_task(ch_a, ch_b).unwrap());

    loop {
        Timer::after(Duration::from_secs(1)).await;
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
async fn open_button_task(mut button: Input<'static>) {
    loop {
        button.wait_for_falling_edge().await;
        Timer::after(Duration::from_millis(DEBOUNCE_MS)).await;
        if button.is_low() {
            DOOR_CMD.signal(DoorCommand::Open);
        }
        button.wait_for_rising_edge().await;
        Timer::after(Duration::from_millis(DEBOUNCE_MS)).await;
    }
}

#[embassy_executor::task]
async fn close_button_task(mut button: Input<'static>) {
    loop {
        button.wait_for_falling_edge().await;
        Timer::after(Duration::from_millis(DEBOUNCE_MS)).await;
        if button.is_low() {
            DOOR_CMD.signal(DoorCommand::Close);
        }
        button.wait_for_rising_edge().await;
        Timer::after(Duration::from_millis(DEBOUNCE_MS)).await;
    }
}

#[embassy_executor::task(pool_size = 2)]
async fn limit_switch_task(mut limit_switch: Input<'static>, limit_signal: &'static Signal<CriticalSectionRawMutex, ()>) {
    info!("Starting limit switch task for {:?}", limit_switch);
    loop {
        limit_switch.wait_for_falling_edge().await; 
        Timer::after(Duration::from_millis(DEBOUNCE_MS)).await;
        if limit_switch.is_low(){
            limit_signal.signal(());
            info!("Limit switch hit: {:?}", limit_switch);
        }
        limit_switch.wait_for_rising_edge().await;
        Timer::after(Duration::from_millis(DEBOUNCE_MS)).await;
    }
}

#[embassy_executor::task]
async fn overcurrent_monitor_task(mut adc1: Adc<'static, ADC1<'static>, Blocking>, mut pin: AdcPin<AdcPinType, ADC1<'static>>) {
    loop {
        if let Ok(value) = adc1.read_oneshot(&mut pin) {
            if value >= OVERCURRENT_THRESHOLD {
                OVERCURRENT_DETECTED.signal(());
                info!("Overcurrent detected: {}", value);
            }
        }
        Timer::after(Duration::from_millis(100)).await;
    }
}

#[embassy_executor::task]
async fn door_state_machine_task() {
    let mut state = DoorState::Idle;
    let mut last_state = DoorState::Idle;
    let mut timeout = Instant::now();

    loop {
        Timer::after(Duration::from_millis(10)).await;

        match state {
            DoorState::Idle => {
                if last_state != state {
                    info!("Door: Entering Idle state");
                    last_state = state;
                    PULLEY_CMD.signal(PulleyCommand::Stop);
                }

                if let Some(cmd) = DOOR_CMD.try_take() {
                    match cmd {
                        DoorCommand::Open => {
                            state = DoorState::Opening;
                            info!("Door: Idle->Opening (command)");
                        }
                        DoorCommand::Close => {
                            state = DoorState::Closing;
                            info!("Door: Idle->Closing (command)");
                        }
                    }
                }
            }
            DoorState::Opening => {
                if last_state != state {
                    info!("Door: Entering Opening state");
                    last_state = state;
                    PULLEY_CMD.signal(PulleyCommand::RampUpCCW);
                    timeout = Instant::now() + Duration::from_millis(DOOR_TIMEOUT_MS);
                }

                if let Some(_) = OPEN_LIMIT_HIT.try_take() {
                    state = DoorState::Opened;
                    info!("Door: Opening->Opened (limit)");
                } else if let Some(cmd) = DOOR_CMD.try_take() {
                    if cmd == DoorCommand::Close {
                        state = DoorState::Closing;
                        info!("Door: Opening->Closing (command)");
                    }
                } else if let Some(_) = OVERCURRENT_DETECTED.try_take() {
                    state = DoorState::Idle;
                    info!("Door: Opening->Idle (overcurrent)");
                } else if Instant::now() >= timeout {
                    state = DoorState::Idle;
                    info!("Door: Opening->Idle (timeout)");
                }
            }
            DoorState::Opened => {
                if last_state != state {
                    info!("Door: Entering Opened state");
                    last_state = state;
                    PULLEY_CMD.signal(PulleyCommand::Stop);
                }

                if let Some(cmd) = DOOR_CMD.try_take() {
                    if cmd == DoorCommand::Close {
                        state = DoorState::Closing;
                        info!("Door: Opened->Closing (command)");
                    }
                }
            }
            DoorState::Closing => {
                if last_state != state {
                    info!("Door: Entering Closing state");
                    last_state = state;
                    PULLEY_CMD.signal(PulleyCommand::RampUpCW);
                    timeout = Instant::now() + Duration::from_millis(DOOR_TIMEOUT_MS);
                }

                if let Some(_) = CLOSE_LIMIT_HIT.try_take() {
                    state = DoorState::Closed;
                    info!("Door: Closing->Closed (limit)");
                } else if let Some(cmd) = DOOR_CMD.try_take() {
                    if cmd == DoorCommand::Open {
                        state = DoorState::Opening;
                        info!("Door: Closing->Opening (command)");
                    }
                } else if let Some(_) = OVERCURRENT_DETECTED.try_take() {
                    state = DoorState::Opening;
                    info!("Door: Closing->Opening (overcurrent)");
                } else if Instant::now() >= timeout {
                    state = DoorState::Idle;
                    info!("Door: Closing->Idle (timeout)");
                }
            }
            DoorState::Closed => {
                if last_state != state {
                    info!("Door: Entering Closed state");
                    last_state = state;
                    PULLEY_CMD.signal(PulleyCommand::Stop);
                }

                if let Some(cmd) = DOOR_CMD.try_take() {
                    if cmd == DoorCommand::Open {
                        state = DoorState::Opening;
                        info!("Door: Closed->Opening (command)");
                    }
                }
            }
        }
    }
}

#[embassy_executor::task]
async fn pulley_driver_task(ch_a: channel::Channel<'static, LowSpeed>, ch_b: channel::Channel<'static, LowSpeed>) {
    let mut state = PulleyState::Hold;
    let mut dest_state = PulleyState::Hold;
    let mut duty: u8 = 0;

    loop {
        Timer::after(Duration::from_millis(PULLEY_RAMP_STEP_MS)).await;

        match state {
            PulleyState::Hold => {
                ch_a.set_duty(0).unwrap();
                ch_b.set_duty(0).unwrap();

                if let Some(cmd) = PULLEY_CMD.try_take() {
                    match cmd {
                        PulleyCommand::RampUpCCW => {
                            dest_state = PulleyState::RampingUpCCW;
                            state = PulleyState::RampingUpCCW;
                            duty = 0;
                        }
                        PulleyCommand::RampUpCW => {
                            dest_state = PulleyState::RampingUpCW;
                            state = PulleyState::RampingUpCW;
                            duty = 0;
                        }
                        _ => {}
                    }
                } else if state != dest_state {
                    match dest_state {
                        PulleyState::RampingUpCCW => {
                            state = PulleyState::RampingUpCCW;
                            duty = 0;
                        }
                        PulleyState::RampingUpCW => {
                            state = PulleyState::RampingUpCW;
                            duty = 0;
                        }
                        _ => {},
                    };

                }
            }
            PulleyState::RampingUpCCW => {
                if let Some(cmd) = PULLEY_CMD.try_take() {
                    match cmd {
                        PulleyCommand::RampUpCW => {
                            dest_state = PulleyState::RampingUpCW;
                            state = PulleyState::RampingDownCCW;
                        }
                        PulleyCommand::Stop => {
                            dest_state = PulleyState::Hold;
                            state = PulleyState::RampingDownCCW;
                        }
                        _ => {}
                    }
                } else if duty < PULLEY_DUTY_MAX {
                    duty += 1;
                    ch_a.set_duty(duty).unwrap();
                    ch_b.set_duty(0).unwrap();
                } else {
                    dest_state = PulleyState::DrivingCCW;
                    state = PulleyState::DrivingCCW;
                }
            }
            PulleyState::DrivingCCW => {
                if let Some(cmd) = PULLEY_CMD.try_take() {
                    match cmd {
                        PulleyCommand::RampUpCW => {
                            dest_state = PulleyState::RampingUpCW;
                            state = PulleyState::RampingDownCCW;
                        }
                        PulleyCommand::Stop => {
                            dest_state = PulleyState::Hold;
                            state = PulleyState::RampingDownCCW;
                        }
                        _ => {}
                    }
                } else {
                    ch_a.set_duty(PULLEY_DUTY_MAX).unwrap();
                    ch_b.set_duty(0).unwrap();
                }
            }
            PulleyState::RampingDownCCW => {
                if let Some(cmd) = PULLEY_CMD.try_take() {
                    match cmd {
                        PulleyCommand::RampUpCCW => {
                            dest_state = PulleyState::RampingUpCCW;
                            state = PulleyState::RampingUpCCW;
                        }
                        _ => {}
                    }
                } else if duty > 0 {
                    duty -= 1;
                    ch_a.set_duty(duty).unwrap();
                    ch_b.set_duty(0).unwrap();
                } else {
                    state = PulleyState::Hold;
                }
            }
            PulleyState::RampingUpCW => {
                if let Some(cmd) = PULLEY_CMD.try_take() {
                    match cmd {
                        PulleyCommand::RampUpCCW => {
                            dest_state = PulleyState::RampingUpCCW;
                            state = PulleyState::RampingDownCW;
                        }
                        PulleyCommand::Stop => {
                            dest_state = PulleyState::Hold;
                            state = PulleyState::RampingDownCW;
                        }
                        _ => {}
                    }
                } else if duty < PULLEY_DUTY_MAX {
                    duty += 1;
                    ch_a.set_duty(0).unwrap();
                    ch_b.set_duty(duty).unwrap();
                } else {
                    dest_state = PulleyState::DrivingCW;
                    state = PulleyState::DrivingCW;
                }
            }
            PulleyState::DrivingCW => {
                if let Some(cmd) = PULLEY_CMD.try_take() {
                    match cmd {
                        PulleyCommand::RampUpCCW => {
                            dest_state = PulleyState::RampingUpCCW;
                            state = PulleyState::RampingDownCW;
                        }
                        PulleyCommand::Stop => {
                            dest_state = PulleyState::Hold;
                            state = PulleyState::RampingDownCW;
                        }
                        _ => {}
                    }
                } else {
                    ch_a.set_duty(0).unwrap();
                    ch_b.set_duty(PULLEY_DUTY_MAX).unwrap();
                }
            }
            PulleyState::RampingDownCW => {
                if let Some(cmd) = PULLEY_CMD.try_take() {
                    match cmd {
                        PulleyCommand::RampUpCW => {
                            dest_state = PulleyState::RampingUpCW;
                            state = PulleyState::RampingUpCW;
                        }
                        _ => {}
                    }
                } else if duty > 0 {
                    duty -= 1;
                    ch_a.set_duty(0).unwrap();
                    ch_b.set_duty(duty).unwrap();
                } else {
                    state = PulleyState::Hold;
                }
            }
        }
    }
}
