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
    cyw43::JoinOptions,
    cyw43_pio::{PioSpi, DEFAULT_CLOCK_DIVIDER},
    embassy_executor::Spawner,
    embassy_time::{Duration, Timer},
    embassy_sync::{
        mutex::Mutex,
        blocking_mutex::raw::CriticalSectionRawMutex,
    },
    embassy_net::{
        tcp::TcpSocket,
        Config,
        DhcpConfig, 
        StackResources,
        Ipv4Address,
    },
    embassy_rp::{
        clocks::RoscRng,
        gpio::{Level, Output},
        peripherals::{DMA_CH0, PIO0, USB},
        pio::Pio,
        usb::Driver,
        i2c::{I2c, Config as I2cConfig},
    },
    embedded_io_async::Write as EmbeddedWriter,
    core::str::{from_utf8, FromStr},
    rand::RngCore,
    static_cell::StaticCell,
    defmt::{unwrap, info},
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

static BME: BmeState = BmeState::new();
static IP_ADDRESS: PicoAddress = PicoAddress::new();

const WIFI_NETWORK: &str = env!("WIFI_NETWORK");
const WIFI_PASSWORD: &str = env!("WIFI_PASSWORD");
const CLIENT_NAME: &str = "Pico-W";
const TCP_PORT: u16 = 80;
const BUFF_SIZE: usize = 8192;
const HTML_BYTES: &[u8] = include_bytes!("html/index.html");
const SSI_TEMP_TAG: &str = "<!--#TEMP-->";
const SSI_HUMID_TAG: &str = "<!--#HUMID-->";
const SSI_PRESSURE_TAG: &str = "<!--#PRESSURE-->";

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

fn process_ssi(html_file: &str, ssi_tag: &str, value: &str) -> String<BUFF_SIZE>{
    let mut processed_html = String::<BUFF_SIZE>::new();
    
    for line in html_file.lines() {
        // Replace SSI tag with actual value

        if let Some(pos) = line.find(ssi_tag) {
            // Split line into parts before and after the tag
            let before = &line[..pos];
            let after = &line[pos + ssi_tag.len()..];
            
            // Write the reconstructed line
            write!(&mut processed_html, "{}{}{}\r\n", before, value, after).unwrap();
        } else {
            write!(&mut processed_html, "{}\r\n", line).unwrap();
        }
    }

    return processed_html;
}

#[embassy_executor::task]
async fn usb_logger_task(driver: Driver<'static, USB>) {
    embassy_usb_logger::run!(1024, log::LevelFilter::Info, driver);
}

#[embassy_executor::task]
async fn cyw43_task(runner: cyw43::Runner<'static, Output<'static>, PioSpi<'static, PIO0, 0, DMA_CH0>>) -> ! {
    runner.run().await
}

#[embassy_executor::task]
async fn net_task(mut runner: embassy_net::Runner<'static, cyw43::NetDriver<'static>>) -> ! {
    runner.run().await
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
        let ip = IP_ADDRESS.get_ip().await;

        let mut ip_str = String::<32>::new();
        let mut temp_str = String::<32>::new();
        let mut humidity_str = String::<32>::new();
        let mut pressure_str = String::<32>::new();
        
        write!(&mut ip_str, "{}.{}.{}.{}", ip.octets()[0], ip.octets()[1], ip.octets()[2], ip.octets()[3]).unwrap();
        write!(&mut temp_str, "Temp: {:.2} C", data.temperature).unwrap();
        write!(&mut humidity_str, "Humidity: {:.1}%", data.humidity).unwrap();
        write!(&mut pressure_str, "P: {:.1} hPa", data.pressure).unwrap();

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
                BME.set_data(data.temperature, data.humidity, data.pressure/100.0).await;
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
    let mut led_toggle_status = true;
    let usb_driver = Driver::new(p.USB, Irqs);
    spawner.must_spawn(usb_logger_task(usb_driver));

    let ph = split_resources!(p);
    spawner.must_spawn(display_task(ph.display_resources));
    spawner.must_spawn(bme_task(ph.bme_resources));

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
    unwrap!(spawner.spawn(cyw43_task(runner)));

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

    unwrap!(spawner.spawn(net_task(runner)));

    // Connecting to the Network
    loop {
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
    info!("Waiting for DHCP...");
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
    let html_str = from_utf8(HTML_BYTES).unwrap();
    
    led_toggle_status = false;

    loop {
        let mut socket = TcpSocket::new(stack, &mut rx_buffer, &mut tx_buffer);
        socket.set_timeout(Some(Duration::from_secs(10)));

        if let Err(e) = socket.accept(TCP_PORT).await {
            log::warn!("Accept Error: {:?}", e);
            continue;
        }

        log::info!("Received Connection from {:?}", socket.remote_endpoint());
        
        // Currently only accept 1 connection at a time
        loop {
            match socket.read(&mut buf).await {
                Ok(0) => {
                    log::info!("Connection closed by client");
                    break;
                }
                Ok(n) => {
                    let request = from_utf8(&buf[..n]).unwrap();
                    let mut processed_html = String::<BUFF_SIZE>::new();
                    write!(&mut processed_html, "{}", html_str).unwrap();

                    // Handle button request
                    if request.starts_with("GET /led") {
                        led_toggle_status = !led_toggle_status;
                        control.gpio_set(0, led_toggle_status).await;
                    } 

                    let mut temp_str = String::<32>::new();
                    let mut humidity_str = String::<32>::new();
                    let mut pressure_str = String::<32>::new();
                    
                    let data = BME.get_data().await;
                    write!(&mut temp_str, "{:.2}", data.temperature).unwrap();
                    write!(&mut humidity_str, "{:.2}", data.humidity).unwrap();
                    write!(&mut pressure_str, "{:.2}", data.pressure).unwrap();
                    
                    // Process SSI template
                    processed_html = process_ssi(processed_html.as_str(), SSI_TEMP_TAG, temp_str.as_str());
                    processed_html = process_ssi(processed_html.as_str(), SSI_HUMID_TAG, humidity_str.as_str());
                    processed_html = process_ssi(processed_html.as_str(), SSI_PRESSURE_TAG, pressure_str.as_str());

                    // Build HTTP response
                    let mut response = String::<BUFF_SIZE>::new();
                    
                    match write!(&mut response,
                                "HTTP/1.1 200 OK\r\nContent-Type: text/html\r\nContent-Length: {}\r\n\r\n{}",
                                processed_html.len(),
                                processed_html) 
                    {
                        Ok(_) => {
                            if let Err(e) = socket.write_all(response.as_bytes()).await {
                                log::warn!("Write Error: {:?}", e);
                                break;
                            }
                        }
                        Err(_) => {
                            log::error!("Response buffer overflow: Buffer is too small");
                            break;
                        }
                    }
                }
                Err(e) => {
                    log::warn!("Read Error: {:?}", e);
                    break;
                }
            };
        }
    }
}
