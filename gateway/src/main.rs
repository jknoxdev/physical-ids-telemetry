use btleplug::api::{Manager as _, Central, Peripheral, ScanFilter};
use btleplug::platform::Manager;
use std::error::Error;
use tokio::time::{sleep, Duration};

#[tokio::main]
async fn main() -> Result<(), Box<dyn Error>> {
    let manager = Manager::new().await?;
    let adapters = manager.adapters().await?;
    let adapter = adapters.into_iter().next()
        .expect("No BLE adapter found");

    println!("Starting LIMA gateway scanner...");
    adapter.start_scan(ScanFilter::default()).await?;

    loop {
        let peripherals = adapter.peripherals().await?;
        for p in peripherals {
            if let Some(props) = p.properties().await? {
                if props.local_name.as_deref().map(|n| n.starts_with("LIMA")).unwrap_or(false) {
                    println!(
                        "LIMA | RSSI: {:?} | Addr: {} | Data: {:X?}",
                        props.rssi,
                        props.address,
                        props.manufacturer_data
                    );
                }
            }
        }
        sleep(Duration::from_millis(100)).await;
    }
}