#![no_std]
#![no_main]
#![deny(unused_must_use)]

use alloc::{
    format,
    string::{String, ToString},
    vec::{self, Vec},
};
use as5600::asynch::As5600;
use bleps::{
    ad_structure::{
        create_advertising_data, AdStructure, BR_EDR_NOT_SUPPORTED, LE_GENERAL_DISCOVERABLE,
    },
    async_attribute_server::AttributeServer,
    asynch::Ble,
    att::Uuid,
    attribute_server::NotificationData,
    gatt,
};
use core::cell::{Cell, RefCell};
use embassy_executor::Spawner;
use embassy_futures::block_on;
use embassy_net::{dns::DnsQueryType, tcp::TcpSocket, Runner, StackResources};
use embassy_sync::{
    blocking_mutex::{raw::CriticalSectionRawMutex, CriticalSectionMutex},
    mutex::Mutex,
    signal::Signal,
    watch::Watch,
};
use embassy_time::{Duration, Ticker, Timer};
use embedded_graphics::Drawable;
use embedded_graphics::{
    mono_font::{ascii, MonoTextStyleBuilder},
    pixelcolor::BinaryColor,
    prelude::Point,
};
use esp_backtrace as _;
use esp_backtrace as _;
use esp_hal::{
    clock::CpuClock,
    delay::Delay,
    esp_riscv_rt::entry,
    gpio::{Input, InputConfig, Io, NoPin, Output, OutputConfig, Pull},
    i2c::master::I2c,
    peripherals::{BT, I2C0},
    rng::Rng,
    time::{self, Rate},
    timer::{systimer::SystemTimer, timg::TimerGroup},
    Async, Blocking,
};
use esp_println::println;
use esp_wifi::{
    ble::controller::BleConnector,
    init,
    wifi::{ClientConfiguration, Configuration, WifiController, WifiDevice, WifiEvent, WifiState},
    EspWifiController,
};
use futures::StreamExt;
use log::{debug, error, info};
use rust_mqtt::{
    client::{client::MqttClient, client_config::ClientConfig},
    packet::v5::reason_codes::ReasonCode,
    utils::rng_generator::CountingRng,
};
use ssd1306::{
    mode::{BufferedGraphicsMode, DisplayConfig},
    prelude::I2CInterface,
    I2CDisplayInterface,
};
use static_cell::StaticCell;
use uln2003::{StepperMotor, ULN2003};

use crate::{
    gy87::Gy87,
    stepper::{a4988::A4988, Stepper, StepperConfig},
};

mod gy87;
mod stepper;

#[macro_use]
extern crate alloc;

macro_rules! mk_static {
    ($t:ty,$val:expr) => {{
        static STATIC_CELL: static_cell::StaticCell<$t> = static_cell::StaticCell::new();
        #[deny(unused_attributes)]
        let x = STATIC_CELL.uninit().write(($val));
        x
    }};
}

type DisplaySize = ssd1306::size::DisplaySize128x64;
type Display = ssd1306::Ssd1306<
    I2CInterface<I2c<'static, Async>>,
    DisplaySize,
    BufferedGraphicsMode<DisplaySize>,
>;

// static POSITION: CriticalSectionMutex<Cell<i32>> = CriticalSectionMutex::new(Cell::new(0));

static POSITION: Watch<CriticalSectionRawMutex, i32, 2> = Watch::new();

pub async fn list_i2c_devices<'a>(i2c: &'a mut impl embedded_hal_async::i2c::I2c) -> Vec<String> {
    let mut addresses = vec![];

    for addr in 1..=127 {
        if i2c.read(addr, &mut [0]).await.is_ok() {
            addresses.push(addr);
        }
    }

    addresses
        .into_iter()
        .map(|addr| format!("{addr:#x}"))
        .collect()
}

// #[embassy_executor::task]
// async fn ui_task(mut display: Display) {
//     display
//         .init_with_addr_mode(ssd1306::command::AddrMode::Horizontal)
//         .unwrap();
//     display.clear_buffer();
//     display.set_display_on(true).unwrap();
//     loop {
//         display.clear_buffer();
//         let pos = critical_section::with(|cs| POSITION.borrow(cs).get());
//         embedded_graphics::text::Text::new(
//             &format!("POS: {}", pos),
//             Point::new(0, 0),
//             MonoTextStyleBuilder::new()
//                 .text_color(BinaryColor::On)
//                 .font(&ascii::FONT_5X8)
//                 .build(),
//         )
//         .draw(&mut display)
//         .unwrap();
//         display.flush().unwrap();
//         Timer::after_millis(10).await;
//     }
// }

#[embassy_executor::task]
async fn ble_task(esp_wifi_ctrl: &'static EspWifiController<'static>, bt: BT) {
    let connector = BleConnector::new(&esp_wifi_ctrl, bt);

    let now = || time::Instant::now().duration_since_epoch().as_millis();
    let mut ble = Ble::new(connector, now);
    println!("Connector created");

    let position_snd = POSITION.sender();
    let mut position_rcv = POSITION.receiver().unwrap();

    loop {
        println!("{:?}", ble.init().await);
        println!("{:?}", ble.cmd_set_le_advertising_parameters().await);
        println!(
            "{:?}",
            ble.cmd_set_le_advertising_data(
                create_advertising_data(&[
                    AdStructure::Flags(LE_GENERAL_DISCOVERABLE | BR_EDR_NOT_SUPPORTED),
                    AdStructure::ServiceUuids16(&[Uuid::Uuid16(0x1809)]),
                    AdStructure::CompleteLocalName(esp_hal::chip!()),
                ])
                .unwrap()
            )
            .await
        );
        println!("{:?}", ble.cmd_set_le_advertise_enable(true).await);

        println!("started advertising");

        let mut rf = |_offset: usize, data: &mut [u8]| {
            data[..20].copy_from_slice(&b"Hello Bare-Metal BLE"[..]);
            17
        };
        let mut wf = |offset: usize, data: &[u8]| {
            println!("RECEIVED: {} {:?}", offset, data);
        };
        let mut wf2 = |offset: usize, data: &[u8]| {
            println!("RECEIVED: {} {:?}", offset, data);
        };
        let mut rf3 = |_offset: usize, data: &mut [u8]| {
            // FIXME: Idk how but I need async attributes in bleps
            data[0..4].copy_from_slice(&position_rcv.try_changed().unwrap_or(0).to_be_bytes());
            4
        };

        let mut wf3 = |offset: usize, data: &[u8]| {
            if offset == 0 && data.len() == 4 {
                let new_pos = i32::from_be_bytes([data[0], data[1], data[2], data[3]]);
                println!("Set position to {new_pos} ({data:?})");
                position_snd.send(new_pos);
            } else {
                println!("Got wrong format for position write: offset={offset}, data={data:?}");
            }
        };

        gatt!([service {
            uuid: "937312e0-2354-11eb-9f10-fbc30a62cf38",
            characteristics: [
                characteristic {
                    uuid: "937312e0-2354-11eb-9f10-fbc30a62cf38",
                    read: rf,
                    write: wf,
                },
                characteristic {
                    uuid: "957312e0-2354-11eb-9f10-fbc30a62cf38",
                    write: wf2,
                },
                characteristic {
                    name: "position",
                    uuid: "987312e0-2354-11eb-9f10-fbc30a62cf38",
                    notify: true,
                    read: rf3,
                    write: wf3,
                },
            ],
        },]);

        let mut rng = bleps::no_rng::NoRng;
        let mut srv = AttributeServer::new(&mut ble, &mut gatt_attributes, &mut rng);

        let mut notifier = || {
            // TODO how to check if notifications are enabled for the characteristic?
            // maybe pass something into the closure which just can query the characteristic
            // value probably passing in the attribute server won't work?

            async {
                Timer::after_secs(100000000).await;
                NotificationData::new(position_handle, &[])
            }
        };

        srv.run(&mut notifier).await.unwrap();
    }
}

#[embassy_executor::task]
async fn imu_task(mut imu: Gy87<I2c<'static, Async>>) {
    loop {
        // println!(
        //     "IMU: temp: {}, accel: {}, gyro: {}, mag: {}",
        //     imu.get_imu_temp().await.unwrap(),
        //     imu.get_acc().await.unwrap(),
        //     imu.get_gyro().await.unwrap(),
        //     imu.read_mag().await.unwrap()
        // );
        println!("{}", imu.read_mag().await.unwrap());
        Timer::after_millis(1000).await;
    }
}

#[embassy_executor::task]
async fn stepper_task(mut stepper: Stepper<A4988<Output<'static>, Output<'static>, NoPin>>) {
    let mut position_rcv = POSITION.receiver().unwrap();
    loop {
        println!("Waiting for new position...");
        let position = position_rcv.changed().await;
        println!("Move stepper to {position}");
        stepper.move_to(position).await.unwrap();
    }
}

#[esp_hal_embassy::main]
async fn main(spawner: Spawner) -> ! {
    esp_println::logger::init_logger_from_env();
    let config = esp_hal::Config::default().with_cpu_clock(CpuClock::max());
    let peripherals = esp_hal::init(config);

    esp_alloc::heap_allocator!(size: 72 * 1024);

    let timg0 = TimerGroup::new(peripherals.TIMG0);
    let mut rng = Rng::new(peripherals.RNG);

    let systimer = SystemTimer::new(peripherals.SYSTIMER);
    esp_hal_embassy::init(systimer.alarm0);

    let esp_wifi_ctrl = mk_static!(
        EspWifiController<'static>,
        init(timg0.timer0, rng.clone(), peripherals.RADIO_CLK).unwrap()
    );

    let i2c = I2c::new(
        peripherals.I2C0,
        esp_hal::i2c::master::Config::default().with_frequency(Rate::from_khz(400)),
    )
    .unwrap()
    .into_async()
    .with_scl(peripherals.GPIO21)
    .with_sda(peripherals.GPIO20);

    // let i2c_bus = shared_bus::new_

    // let as5600 = As5600::new(bus)

    let a4988 = A4988::new(
        Output::new(
            peripherals.GPIO1,
            esp_hal::gpio::Level::Low,
            OutputConfig::default(),
        ),
        Output::new(
            peripherals.GPIO0,
            esp_hal::gpio::Level::Low,
            OutputConfig::default(),
        ),
        NoPin,
    );

    let mut stepper = Stepper::new(
        a4988,
        StepperConfig {
            steps_per_rev: 200,
            acceleration: 500.0,
            step_div: stepper::StepDiv::Div16,
            max_speed_rpm: 400.0,
        },
    );

    let mut imu = Gy87::new(i2c);

    match imu.init(&mut embassy_time::Delay).await {
        Ok(_) => {}
        Err(err) => match err {
            gy87::Error::I2c(err) => {
                println!(
                    "Available I2C devices: {}",
                    (1..=127)
                        .filter(|addr| imu.i2c().read(*addr, &mut [0]).is_ok())
                        .fold(String::new(), |list, addr| format!("{list} {addr:#x}"))
                );
                panic!("IMU init I2C error: {err:?}");
            }
            _ => panic!("IMU init error: {err:?}"),
        },
    }

    (1..=127).for_each(|addr| {
        if let Ok(_) = imu.i2c().read(addr, &mut [0]) {
            println!("Found I2C device at address {addr:#x}");
        }
    });

    // let di = I2CDisplayInterface::new(i2c);

    // let display = ssd1306::Ssd1306::new(
    //     di,
    //     ssd1306::size::DisplaySize128x64,
    //     ssd1306::rotation::DisplayRotation::Rotate0,
    // )
    // .into_buffered_graphics_mode();

    spawner.must_spawn(ble_task(esp_wifi_ctrl, peripherals.BT));
    // spawner.must_spawn(imu_task(imu));
    // spawner.must_spawn(ui_task(display));
    // spawner.must_spawn(stepper_task(stepper));

    loop {
        stepper.move_to(3200).await.unwrap();
        stepper.move_to(0).await.unwrap();
    }
}
