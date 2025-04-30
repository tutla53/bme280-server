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
    embassy_time::{Duration, Timer, Instant, Ticker},
    embassy_sync::{
        channel::Channel,
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

struct TimeFormat {
    days: u64,
    hours: u64,
    minutes: u64,
    seconds: u64,
}

impl TimeFormat {
    fn get_format(duration: embassy_time::Duration) -> TimeFormat {
        let total_seconds = duration.as_secs();
        let days = total_seconds / 86400;  
        let remaining = total_seconds % 86400;
        let hours = remaining / 3600;
        let remaining = remaining % 3600;
        let minutes = remaining / 60;
        let seconds = remaining % 60;

        return TimeFormat {
            days: days,
            hours: hours, 
            minutes: minutes, 
            seconds: seconds
        };
    }
}

struct MessageFormat {
    message: String<32>,
    row: u8,
    col: u8,
}

struct DisplayMessage {
    state : Channel<CriticalSectionRawMutex, MessageFormat, 1024>,
}

impl DisplayMessage {
    const fn new() -> Self {
        Self {
            state: Channel::new(),
        }
    }

    async fn set_data(&self, message: String<32>, row: u8, col:u8) {
        
        let new_state = MessageFormat {
            message: message,
            row: row, 
            col: col,
        };

        self.state.send(new_state).await;
    }
}

static DISPLAY: DisplayMessage = DisplayMessage::new();

async fn set_title() {
    let mut title = String::<32>::new();
    let mut second = String::<32>::new();
    let mut third = String::<32>::new();

    write!(&mut title,  "  Air Monitor   ").unwrap();
    write!(&mut second, "                ").unwrap();
    write!(&mut third,  "                ").unwrap();

    DISPLAY.set_data(title, 0, 0).await;
    DISPLAY.set_data(second, 1, 0).await;
    DISPLAY.set_data(third, 2, 0).await;
}

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
    set_title().await;

    loop {
        let data = DISPLAY.state.receive().await;

        if let Err(e) = display.set_position(data.col, data.row).await {
            log::info!("{:?}", e);
            loop {
                match display.init().await {
                    Ok(()) => {
                        log::warn!("Display has been Initialized");
                        display.clear().await.unwrap();
                        set_title().await;
                        break;
                    }
                    Err(e) => {
                        log::warn!("Write Error: {:?}", e);
                    }
                }
                Timer::after(Duration::from_millis(500)).await;
            }
        };
        
        if let Err(e) = display.write_str(&data.message).await {
            log::info!("{:?}", e);
            loop {
                match display.init().await {
                    Ok(()) => {
                        log::warn!("Display has been Initialized");
                        display.clear().await.unwrap();
                        set_title().await;
                        break;
                    }
                    Err(e) => {
                        log::warn!("Write Error: {:?}", e);
                    }
                }
                Timer::after(Duration::from_millis(500)).await;
            }
        };
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

    let mut tick = Ticker::every(Duration::from_millis(500));

    loop {
        match bme280.measure(&mut delay) {
            Ok(data) => {
                let mut temp_str = String::<32>::new();
                let mut humidity_str = String::<32>::new();
                let mut pressure_str = String::<32>::new();

                write!(&mut temp_str,     "Temp:{:.2} C", data.temperature).unwrap();
                write!(&mut humidity_str, "RH  :{:.2} %", data.humidity).unwrap();
                write!(&mut pressure_str, "P   :{:.3} atm", (data.pressure*0.9869233)/100000.0).unwrap();

                DISPLAY.set_data(temp_str, 5, 0).await;
                DISPLAY.set_data(humidity_str, 6, 0).await;
                DISPLAY.set_data(pressure_str, 7, 0).await;

            },
            Err(_) => {
                let mut temp_str = String::<32>::new();
                let mut humidity_str = String::<32>::new();
                let mut pressure_str = String::<32>::new();

                write!(&mut temp_str,     "Temp: n/a C ").unwrap();
                write!(&mut humidity_str, "RH  : n/a % ").unwrap();
                write!(&mut pressure_str, "P   : n/a atm ").unwrap();

                DISPLAY.set_data(temp_str, 5, 0).await;
                DISPLAY.set_data(humidity_str, 6, 0).await;
                DISPLAY.set_data(pressure_str, 7, 0).await;


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
            }
        }

        tick.next().await;
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
    
    let mut tick = Ticker::every(Duration::from_secs(1));
    let uptime = Instant::now();
    
    loop {
        let uptime_format = TimeFormat::get_format(uptime.elapsed());
        let mut uptime_str = String::<32>::new();
        write!(&mut uptime_str,   "{}d:{:02}h:{:02}m:{:02}s", uptime_format.days, uptime_format.hours, uptime_format.minutes, uptime_format.seconds).unwrap();
        DISPLAY.set_data(uptime_str, 4, 0).await;

        tick.next().await;
    }
}
