#![no_std]
#![no_main]

mod gpio_list;

use crate::gpio_list::Irqs;
use crate::gpio_list::AssignedResources;
use crate::gpio_list::DisplayResources;
use crate::gpio_list::BmeResources;
use crate::gpio_list::WatchdogResources;

use cyw43::JoinOptions;
use cyw43::ScanOptions;
use cyw43_pio::PioSpi;
use cyw43_pio::DEFAULT_CLOCK_DIVIDER;

use embassy_executor::Spawner;

use embassy_time::Duration;
use embassy_time::Timer;
use embassy_time::Ticker; 
use embassy_time::with_timeout;

use embassy_sync::mutex::Mutex;
use embassy_sync::channel::Channel;
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;

use embassy_net::tcp::TcpSocket;
use embassy_net::Config;
use embassy_net::DhcpConfig;
use embassy_net::StackResources;
use embassy_net::Ipv4Address;

use embassy_rp::watchdog::Watchdog;
use embassy_rp::clocks::RoscRng;
use embassy_rp::gpio::Level;
use embassy_rp::gpio::Output;
use embassy_rp::peripherals::DMA_CH0;
use embassy_rp::peripherals:: PIO0;
use embassy_rp::peripherals:: USB;
use embassy_rp::usb::Driver;
use embassy_rp::pio::Pio;
use embassy_rp::i2c::I2c;
use embassy_rp::i2c:: Config as I2cConfig;

use embedded_hal::i2c::I2c as EmbeddedI2c;
use embedded_hal::i2c::ErrorType;

use heapless::String;
use core::fmt::Write;
use core::str::from_utf8;
use core::str::FromStr;

use rand_core::RngCore;
use bme280::i2c::BME280;
use embedded_io_async::Write as EmbeddedWriter;
use static_cell::StaticCell;

use defmt_rtt as _;
use panic_probe as _;

macro_rules! safe_write {
    ($target:expr, $($args:tt)*) => {
        {
            // Isolate the first mutable borrow
            let target = $target;
            match write!(target, $($args)*) {
                Ok(_) => (),
                Err(e) => {
                    log::warn!("Write error: {:?}", e);
                    
                    // Isolate truncate operation
                    {
                        let t = target;
                        t.truncate(t.capacity().saturating_sub(3));
                    }
                    
                    // Isolate fallback write
                    {
                        let t = $target;
                        let _ = write!(t, "...");
                    }
                }
            }
        }
    };
}

static BME: BmeState = BmeState::new();
static IP_ADDRESS: PicoAddress = PicoAddress::new();
static WIFI_WATCHDOG: Channel<CriticalSectionRawMutex, bool, 50> = Channel::new();

const WIFI_NETWORK: &str = env!("WIFI_NETWORK");
const WIFI_PASSWORD: &str = env!("WIFI_PASSWORD");
const CLIENT_NAME: &str = "Pico-W";
const TCP_PORT: u16 = 80;
const BUFF_SIZE: usize = 8192;
const HTML_BYTES: &[u8] = include_bytes!("html/index.html");
const ERROR_BYTES: &[u8] = include_bytes!("html/error_page.html");
const SSI_TEMP_TAG: &str = "<!--#TEMP-->";
const SSI_HUMID_TAG: &str = "<!--#HUMID-->";
const SSI_PRESSURE_TAG: &str = "<!--#PRESSURE-->";
const MAX_OLED_CHAR: usize = 16;
const CYW43_JOIN_ERROR: [&str; 16] = [
    "Success", 
    "Operation failed", 
    "Operation timed out",
    "Operation no matching network found",
    "Operation was aborted",
    "[Protocol Failure] Packet not acknowledged",
    "AUTH or ASSOC packet was unsolicited",
    "Attempt to ASSOC to an auto auth configuration",
    "Scan results are incomplete",
    "Scan aborted by another scan",
    "Scan aborted due to assoc in progress",
    "802.11h quiet period started",
    "User disabled scanning (WLC_SET_SCANSUPPRESS)",
    "No allowable channels to scat",
    "Scan aborted due to CCX fast roam",
    "Abort channel select"
];

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

    async fn set_sensor_data(&self, temperature:f32, humidity:f32, pressure:f32) {
        let new_state = BmeData {
            temperature: temperature, 
            humidity:humidity, 
            pressure:pressure
        };
        let mut current_state = self.state.lock().await;
        *current_state = new_state;
    }
}

struct PicoAddress {
    ip: Mutex<CriticalSectionRawMutex, Ipv4Address>,
}

impl PicoAddress {
    const fn new() -> Self {
        Self {
            ip: Mutex::new(Ipv4Address::new(0, 0, 0, 0)),
        }
    }

    async fn get_ip(&self) -> Ipv4Address {
        return *self.ip.lock().await;
    }

    async fn set_ip(&self, new_ip: Ipv4Address) {
        let mut current_ip = self.ip.lock().await;
        *current_ip = new_ip;
    }
}

struct Sensor<I2C> {
    sensor: BME280<I2C>,
    state: &'static BmeState,
    delay: embassy_time::Delay,
}

impl <I2C> Sensor <I2C>
where
    I2C: EmbeddedI2c + ErrorType, 
{
    fn new(sensor: BME280<I2C>, state: &'static BmeState) -> Self {
        Self {
            sensor,
            state, 
            delay: embassy_time::Delay,
        }
    }

    async fn init(&mut self) {
        loop {
            match self.sensor.init(&mut self.delay) {
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

    async fn measure(&mut self) {
        match self.sensor.measure(&mut self.delay) {
            Ok(data) => {
                let temp = data.temperature;
                let humid = data.humidity * 0.1818+44.81818;
                let pressure = (data.pressure*0.9869233)/100000.0;

                self.state.set_sensor_data(temp, humid, pressure).await;
                log::info!("Temp: {} C, RH: {} %, pressure: {} atm", temp, humid, pressure);
            },
            Err(err) => {
                log::info!("{:?}", err);
                self.state.set_sensor_data(0.0, 0.0, 0.0).await;
                self.init().await;
            }
        }
    }
}

#[embassy_executor::task]
async fn usb_logger_task(driver: Driver<'static, USB>) {
    embassy_usb_logger::run!(1024, log::LevelFilter::Info, driver);
}

#[embassy_executor::task]
async fn watchdog_task(p: WatchdogResources) {
    let mut watchdog = Watchdog::new(p.WATCHDOG);
    watchdog.start(Duration::from_secs(8));
    
    loop {
        let _ = WIFI_WATCHDOG.receive().await;
        watchdog.feed();
    }
}

#[embassy_executor::task]
async fn bme_task(p: BmeResources) {
    let i2c = I2c::new_async(p.I2C_CH, p.SCL_PIN, p.SDA_PIN, Irqs, I2cConfig::default());

    let mut bme280 = Sensor::new(BME280::new_primary(i2c), &BME);
    let mut tick = Ticker::every(Duration::from_millis(500));

    bme280.init().await;

    loop {
        bme280.measure().await;
        tick.next().await;
    }
}

fn process_ssi(html_file: &str, ssi_tag: &str, value: &str) -> Result<String<BUFF_SIZE>, ()> {
    let mut processed_html = String::<BUFF_SIZE>::new();
    
    for line in html_file.lines() {
        // Replace SSI tag with actual value

        if let Some(pos) = line.find(ssi_tag) {
            // Split line into parts before and after the tag
            let before = &line[..pos];
            let after = &line[pos + ssi_tag.len()..];
            
            // Write the reconstructed line
            safe_write!(&mut processed_html, "{}{}{}\r\n", before, value, after);
        } else {
            safe_write!(&mut processed_html, "{}\r\n", line);
        }
    }

    if processed_html.len() >= BUFF_SIZE - 100 {
        return Err(());
    }

    return Ok(processed_html);
}

#[embassy_executor::task]
async fn cyw43_task(runner: cyw43::Runner<'static, Output<'static>, PioSpi<'static, PIO0, 0, DMA_CH0>>) -> ! {
    runner.run().await
}

#[embassy_executor::task]
async fn net_task(mut runner: embassy_net::Runner<'static, cyw43::NetDriver<'static>>) -> ! {
    runner.run().await
}

#[embassy_executor::main]
async fn main(spawner: Spawner) {
    let p = embassy_rp::init(Default::default());
    let ph = split_resources!(p);
    let usb_driver = Driver::new(p.USB, Irqs);

    spawner.must_spawn(usb_logger_task(usb_driver));
    spawner.must_spawn(bme_task(ph.bme_resources));
    spawner.must_spawn(watchdog_task(ph.watchdog_resources));
    
    // WIFI Task
    let mut led_toggle_status = true;
    log::info!("Preparing the Server!");

    let mut rng = RoscRng;
    let fw = include_bytes!("../cyw43-firmware/43439A0.bin");
    let clm = include_bytes!("../cyw43-firmware/43439A0_clm.bin");

    let pwr = Output::new(p.PIN_23, Level::Low);
    let cs = Output::new(p.PIN_25, Level::High);
    let mut pio = Pio::new(p.PIO0, Irqs);
    let spi = PioSpi::new(
        &mut pio.common, 
        pio.sm0, 
        DEFAULT_CLOCK_DIVIDER,
        pio.irq0, 
        cs, 
        p.PIN_24, 
        p.PIN_29, 
        p.DMA_CH0
    );

    static STATE: StaticCell<cyw43::State> = StaticCell::new();
    let state = STATE.init(cyw43::State::new());
    let (net_device, mut control, runner) = cyw43::new(state, pwr, spi, fw).await;
    spawner.must_spawn(cyw43_task(runner));

    control.init(clm).await;
    control
        .set_power_management(cyw43::PowerManagementMode::PowerSave)
        .await;

    log::info!("CYW43 has been set!");    
    control.gpio_set(0, true).await;

    // Using DHCP config for the ipv4 address
    let mut dhcp_config = DhcpConfig::default();
    dhcp_config.hostname = Some(heapless::String::from_str(CLIENT_NAME).unwrap());
    let config = Config::dhcpv4(dhcp_config);

    // Generate random seed
    let seed = rng.next_u64();

    // Init network stack
    static RESOURCES: StaticCell<StackResources<3>> = StaticCell::new();
    let (stack, runner) = embassy_net::new(net_device, config, RESOURCES.init(StackResources::new()), seed);

    spawner.must_spawn(net_task(runner));

    // Connecting to the Network
    let mut retry_delay = Duration::from_secs(1);
    loop {
        let _ = WIFI_WATCHDOG.try_send(true);
        match control.join(WIFI_NETWORK, JoinOptions::new(WIFI_PASSWORD.as_bytes())).await {
            Ok(_) => {
                break
            },
            Err(err) => {
                if err.status<16 {
                    let error_code = err.status as usize;
                    control.gpio_set(0, led_toggle_status).await;
                    led_toggle_status = !led_toggle_status;
                    log::info!("Join failed with error = {}", CYW43_JOIN_ERROR[error_code]);
                    retry_delay = (retry_delay * 2).min(Duration::from_secs(30));
                }
            }
        }
    }

    // Wait for DHCP, not necessary when using static IP
    log::info!("Waiting for DHCP...");
    while !stack.is_config_up() {
        Timer::after_millis(100).await;
    }
    log::info!("DHCP is Now Up!");
    control.gpio_set(0, false).await;

    match stack.config_v4(){
        Some(value) => {
            log::info!("Server Address: {:?}", value.address.address());
            IP_ADDRESS.set_ip(value.address.address()).await;
            Timer::after_millis(100).await;
        },
        None => log::warn!("Unable to Get the Adrress")
    }

    let mut rx_buffer = [0; BUFF_SIZE];
    let mut tx_buffer = [0; BUFF_SIZE];
    let mut buf = [0; BUFF_SIZE];
    let html_str = core::str::from_utf8(HTML_BYTES).unwrap_or("[INVALID UTF-8]");
    let error_str = core::str::from_utf8(ERROR_BYTES).unwrap_or("[INVALID UTF-8]");
    
    led_toggle_status = false;

    loop {
        let _ = WIFI_WATCHDOG.try_send(true);
        let mut socket = TcpSocket::new(stack, &mut rx_buffer, &mut tx_buffer);
        let mut ip_str = String::<MAX_OLED_CHAR>::new();
        let ip = IP_ADDRESS.get_ip().await;
        safe_write!(&mut ip_str, "{}.{}.{}.{}", ip.octets()[0], ip.octets()[1], ip.octets()[2], ip.octets()[3]);

        log::info!("Listening on TCP: {} ...", TCP_PORT);
        let socket_timeout = 2;

        let _ = WIFI_WATCHDOG.try_send(true);
        match with_timeout(Duration::from_secs(socket_timeout), socket.accept(TCP_PORT)).await {
            Ok(value) => {
                match value {
                    Ok(()) => {
                        socket.set_timeout(Some(Duration::from_secs(socket_timeout+5)));
                    },
                    Err(e) => {
                        log::warn!("Accept Error: {:?}", e);
                        control.gpio_set(0, led_toggle_status).await;
                        led_toggle_status  = !led_toggle_status;
                        continue;
                    }
                }
            },
            Err(_) => {
                match stack.config_v4(){
                    Some(value) => {
                        log::info!("Server Address: {:?}", value.address.address());
                        IP_ADDRESS.set_ip(value.address.address()).await;
                        Timer::after_millis(100).await;
                    },
                    None => log::warn!("Unable to Get the Adrress")
                }
                log::warn!("No Connection after {}s", socket_timeout);
                control.gpio_set(0, led_toggle_status).await;
                led_toggle_status  = !led_toggle_status;
                let mut scan_result = control.scan(ScanOptions::default()).await;
                let mut is_connected = false;

                loop {
                    let _ = WIFI_WATCHDOG.try_send(true);
                    match scan_result.next().await {
                        Some(value) => {
                            let arr = value.ssid;
                            let end = value.ssid_len as usize;
                            let ssid_char = core::str::from_utf8(&arr[..end]).unwrap_or_default();
                            if ssid_char == WIFI_NETWORK {
                                log::info!("Found {}", ssid_char);
                                is_connected = true;
                                break;
                            }
                        }
                        None => {
                            log::info!("Missing SSID");
                            IP_ADDRESS.set_ip(Ipv4Address::new(0, 0, 0, 0)).await;
                            let mut ip_str = String::<MAX_OLED_CHAR>::new();
                            let ip = Ipv4Address::new(0, 0, 0, 0);
                            safe_write!(&mut ip_str, "{}.{}.{}.{}", ip.octets()[0], ip.octets()[1], ip.octets()[2], ip.octets()[3]);                  
                            break;
                        }
                    }
                    Timer::after_millis(100).await;
                }
        
                drop(scan_result);

                if !is_connected {
                    control.leave().await;
                    loop {
                        let _ = WIFI_WATCHDOG.try_send(true);
                        match control.join(WIFI_NETWORK, JoinOptions::new(WIFI_PASSWORD.as_bytes())).await {
                            Ok(_) => {
                                Timer::after_millis(100).await;
                                break
                            },
                            Err(err) => {
                                if err.status<16 {
                                    let error_code = err.status as usize;
                                    control.gpio_set(0, led_toggle_status).await;
                                    led_toggle_status = !led_toggle_status;
                                    log::info!("Join failed with error = {}", CYW43_JOIN_ERROR[error_code]);
                                }
                            }
                        }
                    }

                    // Wait for DHCP, not necessary when using static IP
                    log::info!("Waiting for DHCP...");
                    while !stack.is_config_up() {
                        let _ = WIFI_WATCHDOG.try_send(true);
                        Timer::after_millis(100).await;
                    }
                    log::info!("DHCP is Now Up!");
                    control.gpio_set(0, false).await;

                    match stack.config_v4(){
                        Some(value) => {
                            log::info!("Server Address: {:?}", value.address.address());
                            IP_ADDRESS.set_ip(value.address.address()).await;
                            Timer::after_millis(100).await;
                        },
                        None => log::warn!("Unable to Get the Adrress")
                    }

                }
                continue;
            }
        }

        log::info!("Received Connection from {:?}", socket.remote_endpoint());
        control.gpio_set(0, true).await;

        // Currently only accept 1 connection at a time
        loop {
            let _ = WIFI_WATCHDOG.try_send(true);
            match socket.read(&mut buf).await {
                Ok(0) => {
                    log::info!("Connection closed by client");
                    break;
                }
                Ok(n) => {
                    let mut total_read = n;    
                    while !buf[..total_read].ends_with(b"\r\n\r\n") {
                        if total_read >= BUFF_SIZE {
                            log::warn!("Request too large!");
                            break;
                        }
                            
                        // Read more data
                        match socket.read(&mut buf[total_read..]).await {
                            Ok(bytes_read) => total_read += bytes_read,
                            Err(e) => {
                                log::warn!("Read error during accumulation: {:?}", e);
                                break;
                            }
                        }
                    }
                    
                    let request = from_utf8(&buf[..total_read]).unwrap();
                    let mut processed_html = String::<BUFF_SIZE>::new();
                    safe_write!(&mut processed_html, "{}", html_str);

                    // Handle button request
                    if request.starts_with("GET /led") {
                        led_toggle_status = !led_toggle_status;
                        control.gpio_set(0, led_toggle_status).await;
                    } 

                    let mut temp_str = String::<MAX_OLED_CHAR>::new();
                    let mut humidity_str = String::<MAX_OLED_CHAR>::new();
                    let mut pressure_str = String::<MAX_OLED_CHAR>::new();
                    
                    let data = BME.get_data().await;
                    safe_write!(&mut temp_str, "{:.2}", data.temperature);
                    safe_write!(&mut humidity_str, "{:.2}", data.humidity);
                    safe_write!(&mut pressure_str, "{:.3}", data.pressure);
                    
                    // Process SSI template
                    processed_html = match process_ssi(processed_html.as_str(), SSI_TEMP_TAG, temp_str.as_str()) {
                        Ok(result) => { 
                            result
                        },
                        Err(_) => {
                            log::info!("HTML near capacity!");
                            String::new()
                        }
                    };

                    processed_html = match process_ssi(processed_html.as_str(), SSI_HUMID_TAG, humidity_str.as_str()) {
                        Ok(result) => { 
                            result
                        },
                        Err(_) => {
                            log::info!("HTML near capacity!");
                            String::new()
                        }
                    };

                    processed_html = match process_ssi(processed_html.as_str(), SSI_PRESSURE_TAG, pressure_str.as_str()) {
                        Ok(result) => { 
                            result
                        },
                        Err(_) => {
                            log::info!("HTML near capacity!");
                            String::new()
                        }
                    };
                    
                    if processed_html.is_empty() {
                        safe_write!(&mut processed_html, "{}", error_str);
                        log::info!("HTML generation failed");
                    }

                    // Build HTTP response
                    let mut response = String::<BUFF_SIZE>::new();

                    safe_write!(&mut response,
                        "HTTP/1.1 200 OK\r\nContent-Type: text/html\r\nContent-Length: {}\r\n\r\n{}",
                        processed_html.len(),
                        processed_html);
                    
                    if let Err(e) = socket.write_all(response.as_bytes()).await {
                        log::warn!("Write Error: {:?}", e);
                        break;
                    };
                },
                
                Err(e) => {
                    log::warn!("Read Error: {:?}", e);
                    break;
                }
            };
        }
    }   
}