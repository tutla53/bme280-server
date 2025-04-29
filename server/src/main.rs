#![no_std]
#![no_main]

mod gpio_list;

use {
    crate::gpio_list::{
        Irqs,
        AssignedResources,
        DisplayResources,
        BmeResources,
    },
    embassy_executor::Spawner,
    embassy_time::{Duration, Timer},
    embassy_sync::{
        mutex::Mutex,
        blocking_mutex::raw::CriticalSectionRawMutex,
    },
    embassy_rp::{
        peripherals::USB,
        usb::Driver,
        i2c::{I2c, Config as I2cConfig},
    },
    heapless::String,
    core::fmt::Write,
    bme280::i2c::BME280,
    ssd1306::{
        I2CDisplayInterface,
        Ssd1306Async,
        prelude::DisplayRotation,
        size::DisplaySize128x64,
        mode::DisplayConfigAsync,
    },
    {defmt_rtt as _, panic_probe as _},
};

#[derive(Clone, Copy)]
struct BmeData {
    temperature: f32,
    humidity: f32,
    pressure:f32,
}

struct BmeState {
    state : Mutex<CriticalSectionRawMutex, BmeData>,
}

impl BmeState {
    const fn new() -> Self {
        Self {
            state: Mutex::new(BmeData{temperature: 0.0, humidity: 0.0, pressure: 0.0}),
        }
    }

    async fn get_data(&self) -> BmeData {
        return *self.state.lock().await;
    }

    async fn set_data(&self, temperature:f32, humidity:f32, pressure:f32) {
        let new_state = BmeData {
            temperature: temperature, 
            humidity:humidity, 
            pressure:pressure
        };
        let mut current_state = self.state.lock().await;
        *current_state = new_state;
    }

}

static BME: BmeState = BmeState::new();

#[embassy_executor::task]
async fn usb_logger_task(driver: Driver<'static, USB>) {
    embassy_usb_logger::run!(1024, log::LevelFilter::Info, driver);
}

#[embassy_executor::task]
async fn display_task(p: DisplayResources) {
    let i2c = I2c::new_async(p.I2C_CH, p.SCL_PIN, p.SDA_PIN, Irqs, I2cConfig::default());

    let interface = I2CDisplayInterface::new(i2c);
    let mut display = Ssd1306Async::new(interface, DisplaySize128x64, DisplayRotation::Rotate0).into_terminal_mode();
    
    loop {
        match display.init().await{
            Ok(()) => {
                log::warn!("Display has been Initialized");
                break;
            }
            Err(e) => {
                log::warn!("Write Error: {:?}", e);
            }
        }
        Timer::after(Duration::from_millis(500)).await;
    }

    display.clear().await.unwrap();
    display.set_position(2, 0).await.unwrap();
    let _ = display.write_str("Air Monitor").await;

    loop {
        let data = BME.get_data().await;

        let mut ip_str = String::<32>::new();
        let mut temp_str = String::<32>::new();
        let mut humidity_str = String::<32>::new();
        let mut pressure_str = String::<32>::new();
        
        write!(&mut ip_str, "No WIFI").unwrap();
        write!(&mut temp_str, "Temp: {:.2} C", data.temperature).unwrap();
        write!(&mut humidity_str, "Humidity: {:.1}%", data.humidity).unwrap();
        write!(&mut pressure_str, "P: {:.3} atm", data.pressure).unwrap();

        display.set_position(0, 2).await.unwrap();
        let _ = display.write_str(&ip_str).await;
        display.set_position(0, 4).await.unwrap();
        let _ = display.write_str(&temp_str).await;
        display.set_position(0, 5).await.unwrap();
        let _ = display.write_str(&humidity_str).await;
        display.set_position(0, 6).await.unwrap();
        let _ = display.write_str(&pressure_str).await;
        Timer::after(Duration::from_secs(1)).await;
    }
}

#[embassy_executor::task]
async fn bme_task(p: BmeResources) {
    let i2c = I2c::new_async(p.I2C_CH, p.SCL_PIN, p.SDA_PIN, Irqs, I2cConfig::default());

    let mut delay = embassy_time::Delay;
    let mut bme280 = BME280::new_primary(i2c);

    loop {
        match bme280.init(&mut delay) {
            Ok(()) => {
                log::info!("BME280 initialized"); 
                break; 
            },
            Err(e) => {
                log::info!("{:?}", e);
            }
        }
        Timer::after(Duration::from_millis(500)).await;
    }

    loop {
        match bme280.measure(&mut delay) {
            Ok(data) => {
                BME.set_data(data.temperature, data.humidity, (data.pressure*0.9869233)/100000.0).await;
            },
            Err(_) => {
                BME.set_data(0.0, 0.0, 0.0).await;
            }
        }
        Timer::after(Duration::from_secs(1)).await;
    }
}

#[embassy_executor::main]
async fn main(spawner: Spawner) {
    let p = embassy_rp::init(Default::default());
    let usb_driver = Driver::new(p.USB, Irqs);
    spawner.must_spawn(usb_logger_task(usb_driver));

    let ph = split_resources!(p);
    spawner.must_spawn(display_task(ph.display_resources));
    spawner.must_spawn(bme_task(ph.bme_resources));

}
