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
use core::{
    borrow::Borrow as _,
    cell::{Cell, RefCell},
    f32::consts::{FRAC_PI_2, PI, TAU},
};
use embassy_embedded_hal::shared_bus::asynch::i2c::I2cDevice;
use embassy_executor::Spawner;
use embassy_futures::block_on;
use embassy_net::{dns::DnsQueryType, tcp::TcpSocket, Runner, StackResources};
use embassy_sync::{
    blocking_mutex::raw::CriticalSectionRawMutex, mutex::Mutex, signal::Signal, watch::Watch,
};
use embassy_time::{Duration, Instant, Ticker, Timer};
use embedded_graphics::{
    mono_font::{ascii, MonoTextStyleBuilder},
    pixelcolor::BinaryColor,
    prelude::Point,
    primitives::Line,
    text::Text,
};
use embedded_graphics::{
    prelude::Angle,
    primitives::{Arc, Circle, PrimitiveStyleBuilder, StyledDrawable},
    Drawable,
};
use embedded_hal_async::i2c::I2c as _;
use esp_backtrace as _;
use esp_backtrace as _;
use esp_hal::{
    clock::CpuClock,
    gpio::{NoPin, Output, OutputConfig},
    i2c::master::I2c,
    peripherals::BT,
    rng::Rng,
    time::{self, Rate},
    timer::{systimer::SystemTimer, timg::TimerGroup},
    tsens::TemperatureSensor,
    Async,
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
use micromath::F32Ext;
use nalgebra::Vector3;
use rust_mqtt::{
    client::{client::MqttClient, client_config::ClientConfig},
    packet::v5::reason_codes::ReasonCode,
    utils::rng_generator::CountingRng,
};
use ssd1306::{
    mode::{
        BasicMode, BufferedGraphicsMode, BufferedGraphicsModeAsync, DisplayConfig,
        DisplayConfigAsync,
    },
    prelude::I2CInterface,
    I2CDisplayInterface,
};
use static_cell::StaticCell;
use uln2003::{StepperMotor, ULN2003};

use crate::{
    cube::Cube,
    gy87::{
        madgwick::{MadgwickFilter, Rotation},
        Gy87,
    },
    stepper::{a4988::A4988, Stepper, StepperConfig},
};

pub mod cube;
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

type I2cBus = esp_hal::i2c::master::I2c<'static, Async>;
type I2cDev = I2cDevice<'static, CriticalSectionRawMutex, I2cBus>;

type DisplaySize = ssd1306::size::DisplaySize128x64;
type Display = ssd1306::Ssd1306Async<I2CInterface<I2cDev>, DisplaySize, BasicMode>;

static POSITION: Watch<CriticalSectionRawMutex, i32, 3> = Watch::new();
static IMU: Watch<CriticalSectionRawMutex, ImuData, 2> = Watch::new();
static STEPPER_ANGLE: Watch<CriticalSectionRawMutex, f32, 2> = Watch::new();
static CHIP_TEMP: Watch<CriticalSectionRawMutex, f32, 2> = Watch::new();

// TODO: Should be fetched from phone's GPS
pub const DECLINATION: f32 = 0.20944;

#[derive(Clone, Copy)]
struct ImuData {
    mag: Vector3<f32>,
    accel: Vector3<f32>,
    gyro: Vector3<f32>,
    rotation: Rotation,
}

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

#[embassy_executor::task]
async fn ui_task(display: Display) {
    let mut position_rcv = POSITION.receiver().unwrap();
    let mut imu_rcv = IMU.receiver().unwrap();
    let mut stepper_angle_rcv = STEPPER_ANGLE.receiver().unwrap();

    let mut display = display.into_buffered_graphics_mode();
    display.init().await.unwrap();
    display.clear_buffer();
    display.flush().await.unwrap();
    display.set_display_on(true).await.unwrap();

    let mut redraw_blink = false;

    let stroke_style = PrimitiveStyleBuilder::new()
        .stroke_color(BinaryColor::On)
        .stroke_width(1)
        .build();
    let fill_style = PrimitiveStyleBuilder::new()
        .fill_color(BinaryColor::On)
        .build();
    let font_style = MonoTextStyleBuilder::new()
        .text_color(BinaryColor::On)
        .font(&ascii::FONT_6X9)
        .build();

    let compass_pos = Point::new(30, 30);
    let compass_r = 15;

    let stepper_pos = Point::new(60, 20);
    let stepper_dir_r = 20;

    loop {
        display.clear_buffer();

        display.set_pixel(0, 0, redraw_blink);
        redraw_blink = !redraw_blink;

        let stepper_angle = stepper_angle_rcv.changed().await;
        let ImuData { rotation, .. } = imu_rcv.changed().await;

        // let mag_zx = mag.z.atan2(mag.x);
        // let mag_zy = mag.y.atan2(mag.z);

        // let heading = Angle::from_degrees(heading);

        Circle::new(stepper_pos, stepper_dir_r * 2)
            .draw_styled(&stroke_style, &mut display)
            .unwrap();

        // Circle::new(compass_pos + Point::new_equal(compass_r as i32 - 2), 4)
        //     .draw_styled(&fill_style, &mut display)
        //     .unwrap();

        // let dir_point = Point::new(
        //     (stepper_dir_r as f32 * stepper_angle.cos()) as i32,
        //     (stepper_dir_r as f32 * stepper_angle.sin()) as i32,
        // );
        let stepper_center =
            stepper_pos + Point::new_equal(stepper_dir_r as i32 - stroke_style.stroke_width as i32);

        Line::new(
            stepper_center,
            stepper_center
                + Point::new(
                    (stepper_dir_r as f32 * (stepper_angle).cos()) as i32,
                    (stepper_dir_r as f32 * (stepper_angle).sin()) as i32,
                ),
        )
        .draw_styled(&stroke_style, &mut display)
        .unwrap();

        let compass_cube = Cube::new(compass_pos, compass_r).rotated_3d(
            // Angle::from_radians(mag_zy).normalize(),
            // Angle::from_radians(mag_zx).normalize(),
            Angle::from_radians(rotation.roll),
            Angle::from_radians(rotation.pitch),
            Angle::from_radians(rotation.yaw),
        );

        compass_cube
            .draw_styled(&stroke_style, &mut display)
            .unwrap();

        let north_edge = compass_cube.edge_n(0).unwrap();

        Text::new(
            "N",
            north_edge.midpoint()
                - Point::new(
                    font_style.font.character_size.width as i32 / 2,
                    font_style.font.character_size.height as i32 / 2,
                ),
            font_style,
        )
        .draw(&mut display)
        .unwrap();

        let text = format!("Heading: {}deg", rotation.yaw.to_degrees());

        embedded_graphics::text::Text::new(&text, Point::new(10, 10), font_style)
            .draw(&mut display)
            .unwrap();

        display.flush().await.unwrap();
    }
}

#[embassy_executor::task]
async fn chip_temp_task(sensor: TemperatureSensor<'static>) {
    let chip_temp_snd = CHIP_TEMP.sender();
    sensor.power_up();
    loop {
        let temp = sensor.get_temperature().to_celsius();
        chip_temp_snd.send(temp);
        println!("Chip temp: {temp}C");
        Timer::after_millis(5_000).await;
    }
}

#[embassy_executor::task]
async fn imu_task(mut imu: Gy87<I2cDev>) {
    imu.init().await.unwrap();
    println!(
        "Mag self test delta: {}",
        imu.mag_self_test().await.unwrap()
    );

    imu.set_accel_hpf(gy87::mpu6050::ACCEL_HPF::_5)
        .await
        .unwrap();
    imu.set_accel_range(gy87::mpu6050::AccelRange::G2)
        .await
        .unwrap();
    imu.set_gyro_range(gy87::mpu6050::GyroRange::D250)
        .await
        .unwrap();
    imu.set_master_interrupt_enabled(true).await.unwrap();
    let mut q = MadgwickFilter::new((3.0 / 4.0).sqrt() * PI * (40.0 / 180.0));

    let imu_sender = IMU.sender();
    loop {
        // println!(
        //     "IMU: temp: {}, accel: {}, gyro: {}, mag: {}",
        //     imu.get_imu_temp().await.unwrap(),
        //     imu.get_acc().await.unwrap(),
        //     imu.get_gyro().await.unwrap(),
        //     imu.read_mag().await.unwrap()
        // );
        // println!("{}", imu.read_mag().await.unwrap());
        let mag = match imu.read_mag_gauss().await {
            Ok(mut mag) => {
                let x = mag.x;
                let y = mag.y;
                mag.x = y;
                mag.y = -x;
                mag
            }
            Err(err) => {
                println!("Magnetometer read ERROR: {err:?}");
                continue;
            }
        };
        let accel = match imu.get_acc().await {
            Ok(accel) => accel,
            Err(err) => {
                println!("Accelerometer read ERROR: {err:?}");
                continue;
            }
        };
        let gyro = match imu.get_gyro().await {
            Ok(gyro) => gyro,
            Err(err) => {
                println!("Gyroscope read ERROR: {err:?}");
                continue;
            }
        };
        q.update(accel, gyro, mag);

        let mut rotation = q.rotation();
        rotation.yaw += DECLINATION;

        imu_sender.send(ImuData {
            mag,
            accel,
            gyro,
            rotation,
        });
        Timer::after_millis(100).await;
    }
}

#[embassy_executor::task]
async fn ble_task(esp_wifi_ctrl: &'static EspWifiController<'static>, bt: BT) {
    let connector = BleConnector::new(&esp_wifi_ctrl, bt);

    let now = || time::Instant::now().duration_since_epoch().as_millis();
    let mut ble = Ble::new(connector, now);
    println!("Connector created");

    let position_snd = POSITION.sender();
    let mut position_rcv = POSITION.receiver().unwrap();
    let mut chip_temp_rcv = CHIP_TEMP.receiver().unwrap();

    let mut chip_temp_read = |_offset: usize, data: &mut [u8]| -> usize {
        let temp = chip_temp_rcv.try_get().unwrap_or(0.0) as i16;
        data.copy_from_slice(&temp.to_le_bytes());
        2
    };

    loop {
        println!("{:?}", ble.init().await);
        println!("{:?}", ble.cmd_set_le_advertising_parameters().await);
        println!(
            "{:?}",
            ble.cmd_set_le_advertising_data(
                create_advertising_data(&[
                    AdStructure::Flags(LE_GENERAL_DISCOVERABLE | BR_EDR_NOT_SUPPORTED),
                    AdStructure::ServiceUuids16(&[Uuid::Uuid16(0x1809)]),
                    AdStructure::CompleteLocalName("telemount"),
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

        let temp_char_value = &[0u8; 8];

        gatt!([
            service {
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
            },
            service {
                uuid: "00001809-0000-1000-8000-00805f9b34fb",
                characteristics: [
                    characteristic {
                        uuid: "00001809-0000-1000-8000-00805f9b34fb",
                        notify: true,
                        value: temp_char_value,
                    },
                    characteristic {
                        name: "temp",
                        uuid: "00002A1C-0000-1000-8000-00805f9b34fb",
                        read: chip_temp_read,
                        description: "ESP32C3 chip internal temperature sensor value",
                        notify: true,
                    },
                ],
            },
        ]);

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
async fn stepper_task(mut stepper: Stepper<A4988<Output<'static>, Output<'static>, NoPin>>) {
    // let mut position_rcv = POSITION.receiver().unwrap();
    let mut imu_rcv = IMU.receiver().unwrap();
    loop {
        // println!("Waiting for new position...");
        // let position = position_rcv.changed().await;
        // println!("Move stepper to {position}");
        // stepper.move_to(position).await.unwrap();

        let ImuData { rotation, .. } = imu_rcv.changed().await;

        stepper
            .move_to((1600.0 * rotation.yaw / PI) as i32)
            .await
            .unwrap();
    }
}

#[embassy_executor::task]
async fn encoder_task(mut encoder: As5600<I2cDev>) {
    let stepper_angle_snd = STEPPER_ANGLE.sender();
    loop {
        let angle = encoder.angle().await.unwrap() as f32 / 2.0.powi(12) * 2.0 * PI;
        stepper_angle_snd.send(angle);
        // println!("Angle: {}", );
        Timer::after_millis(100).await;
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

    static I2C_BUS: StaticCell<Mutex<CriticalSectionRawMutex, I2c<'static, Async>>> =
        StaticCell::new();
    let i2c_bus = I2C_BUS.init(Mutex::new(i2c));

    let as5600 = As5600::new(I2cDevice::new(i2c_bus));

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

    let mut imu = Gy87::new(I2cDevice::new(i2c_bus));

    (1..=127).for_each(|addr| {
        if let Ok(_) = block_on(imu.i2c().read(addr, &mut [0])) {
            println!("Found I2C device at address {addr:#x}");
        }
    });

    let di = I2CDisplayInterface::new(I2cDevice::new(i2c_bus));

    let display = ssd1306::Ssd1306Async::new(
        di,
        ssd1306::size::DisplaySize128x64,
        ssd1306::rotation::DisplayRotation::Rotate0,
    );

    spawner.must_spawn(ble_task(esp_wifi_ctrl, peripherals.BT));
    // spawner.must_spawn(imu_task(imu));
    // spawner.must_spawn(ui_task(display));
    // spawner.must_spawn(stepper_task(stepper));
    // spawner.must_spawn(encoder_task(as5600));
    spawner.must_spawn(chip_temp_task(
        TemperatureSensor::new(peripherals.TSENS, esp_hal::tsens::Config::default())
            .expect("Failed to initialize an internal temperature sensor"),
    ));

    POSITION.sender().send(0);

    loop {
        // stepper.move_to(3200).await.unwrap();
        // stepper.move_to(0).await.unwrap();
        Timer::after_secs(10000).await;
    }
}
