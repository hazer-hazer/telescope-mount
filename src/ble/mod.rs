use crate::ble::temp::ChipTempService;
use embassy_futures::{
    join::join,
    select::{select, select_array, Either},
};
use embassy_time::Timer;
use esp_println::println;
use trouble_host::prelude::*;

pub mod temp;

/// Max number of connections
const CONNECTIONS_MAX: usize = 1;

/// Max number of L2CAP channels.
const L2CAP_CHANNELS_MAX: usize = 2; // Signal + att
const L2CAP_MTU: usize = 256;

const ADV_SETS: usize = 1;

const DEVICE_NAME: &str = "telemount";

// TODO: RSSI task to display connection

pub enum GattServiceError {}

pub trait GattService {
    #[allow(async_fn_in_trait)]
    async fn run<C: Controller>(
        &self,
        conn: &GattConnection<'_, '_>,
        stack: &Stack<'_, C>,
    ) -> Result<(), GattServiceError>;
}

#[gatt_server]
pub struct Server {
    pub chip_temp: ChipTempService,
}

impl<'a> Server<'a> {
    async fn rssi_task<C: Controller>(
        &self,
        conn: &GattConnection<'_, '_>,
        stack: &Stack<'_, C>,
    ) -> Result<(), GattServiceError> {
        loop {
            match conn.raw().rssi(stack).await {
                Ok(rssi) => {
                    println!("RSSI: {rssi}");
                }
                Err(err) => {
                    println!("Error reading RSSI: {err:?}");
                }
            }
            Timer::after_millis(1_000).await;
        }
    }

    async fn run_services<C: Controller>(
        &self,
        conn: &GattConnection<'_, '_>,
        stack: &Stack<'_, C>,
    ) -> Result<(), GattServiceError> {
        // TODO: Split error by services?
        match select(self.chip_temp.run(conn, stack), self.rssi_task(conn, stack)).await {
            Either::First(res) | Either::Second(res) => res,
        }
    }
}

async fn ble_task<C: Controller>(mut runner: Runner<'_, C>) {
    let backoff_factor = 2;
    let mut backoff_timeout_ms = 500;
    loop {
        if let Err(e) = runner.run().await {
            println!(
                "[ble_task] error: {:?}. Retrying after {backoff_timeout_ms}ms",
                e
            );
        }
        Timer::after_millis(backoff_timeout_ms).await;
        backoff_timeout_ms *= backoff_factor;
    }
}

/// Create an advertiser to use to connect to a BLE Central, and wait for it to connect.
async fn advertise<'values, 'server, C: Controller>(
    name: &'values str,
    peripheral: &mut Peripheral<'values, C>,
    server: &'server Server<'values>,
) -> Result<GattConnection<'values, 'server>, BleHostError<C::Error>> {
    let mut advertiser_data = [0; 31];
    let len = AdStructure::encode_slice(
        &[
            AdStructure::Flags(LE_GENERAL_DISCOVERABLE | BR_EDR_NOT_SUPPORTED),
            AdStructure::ServiceUuids16(&[[0x0f, 0x18]]),
            AdStructure::CompleteLocalName(name.as_bytes()),
            // AdStructure::ShortenedLocalName(name.as_bytes()),
        ],
        &mut advertiser_data[..],
    )?;
    let advertiser = peripheral
        .advertise(
            &Default::default(),
            Advertisement::ConnectableScannableUndirected {
                adv_data: &advertiser_data[..len],
                scan_data: &[],
            },
        )
        .await?;
    println!("[adv] advertising");
    let conn = advertiser.accept().await?.with_attribute_server(server)?;
    println!("[adv] connection established");
    Ok(conn)
}

pub async fn run<C: Controller>(controller: C) {
    // TODO: Use mac address from ESP32?
    let address = Address::random([0x42, 0x5A, 0xE3, 0x1E, 0x83, 0xE7]);

    let mut resources: HostResources<
        // DefaultPacketPool,
        CONNECTIONS_MAX,
        L2CAP_CHANNELS_MAX,
        L2CAP_MTU,
        ADV_SETS,
    > = HostResources::new();
    let stack = trouble_host::new(controller, &mut resources).set_random_address(address);
    let Host {
        // central,
        mut peripheral,
        runner,
        ..
    } = stack.build();

    // GAP (generic access profile) says how to interact with our device
    let server = Server::new_with_config(GapConfig::Peripheral(PeripheralConfig {
        name: DEVICE_NAME,
        appearance: &appearance::motorized_device::GENERIC_MOTORIZED_DEVICE,
    }))
    .unwrap();

    let _ = join(ble_task(runner), async {
        loop {
            let adv = advertise(DEVICE_NAME, &mut peripheral, &server).await;

            match adv {
                Ok(conn) => {
                    let gatt_events = gatt_events_task(&server, &conn);
                    let services_task = server.run_services(&conn, &stack);

                    select(gatt_events, services_task).await;
                }
                Err(_) => todo!(),
            }
        }
    })
    .await;
}

async fn gatt_events_task(server: &Server<'_>, conn: &GattConnection<'_, '_>) -> Result<(), Error> {
    // let level = server.battery_service.level;

    let reason = loop {
        match conn.next().await {
            GattConnectionEvent::Disconnected { reason } => break reason,
            GattConnectionEvent::Gatt { event } => {
                match event {
                    Ok(event) => {
                        match &event {
                            GattEvent::Read(event) => {
                                if event.handle() == server.chip_temp.value.handle {
                                    let value = server.get(&server.chip_temp.value);
                                    println!(
                                        "[gatt] Read Event to Level Characteristic: {:?}",
                                        value
                                    );
                                }
                            }
                            GattEvent::Write(event) => {
                                if event.handle() == server.chip_temp.value.handle {
                                    println!(
                                        "[gatt] Write Event to Level Characteristic: {:?}",
                                        event.data()
                                    );
                                }
                            }
                        }
                        // This step is also performed at drop(), but writing it explicitly is necessary
                        // in order to ensure reply is sent.
                        match event.accept() {
                            Ok(reply) => reply.send().await,
                            Err(e) => println!("[gatt] error sending response: {:?}", e),
                        }
                    }
                    Err(err) => {
                        println!("Event error {err:?}")
                    }
                }
            }
            _ => {} // ignore other Gatt Connection Events
        }
    };
    println!("[gatt] disconnected: {:?}", reason);
    Ok(())
}
