use crate::ble::{GattService, GattServiceError};
use embassy_sync::{blocking_mutex::raw::CriticalSectionRawMutex, watch::Watch};
use esp_println::println;
use trouble_host::prelude::*;

pub static CHIP_TEMP: Watch<CriticalSectionRawMutex, f32, 2> = Watch::new();

#[gatt_service(uuid = service::HEALTH_THERMOMETER)]
pub struct ChipTempService {
    // #[descriptor(uuid = descriptors::VALID_RANGE, read, value = [])]
    #[descriptor(uuid = descriptors::MEASUREMENT_DESCRIPTION, name = "Temperature", read, value = "Chip internal temperature")]
    #[characteristic(uuid = characteristic::TEMPERATURE_MEASUREMENT, read, notify, value = 0)]
    pub value: i16,
}

impl GattService for ChipTempService {
    async fn run<C: Controller>(
        &self,
        conn: &GattConnection<'_, '_>,
        _stack: &Stack<'_, C>,
    ) -> Result<(), GattServiceError> {
        let mut chip_temp_rcv = CHIP_TEMP.receiver().unwrap();
        let temp = self.value;
        loop {
            let value = chip_temp_rcv.changed().await;
            let value = (value * 100.0) as i16;

            if let Err(err) = temp.notify(conn, &value).await {
                println!("Error sending chip temp value: {err:?}");
            }
        }
    }
}
