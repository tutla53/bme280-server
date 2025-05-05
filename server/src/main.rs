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
    cyw43::{JoinOptions, ScanOptions},
    cyw43_pio::{PioSpi, DEFAULT_CLOCK_DIVIDER},
    embassy_executor::Spawner,
    embassy_time::{Duration, Timer, Instant, Ticker, with_timeout},
    embassy_sync::{
        mutex::Mutex,
        channel::Channel,
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
        usb::Driver,
        pio::Pio,
        i2c::{I2c, Config as I2cConfig},
    },
    embedded_hal::i2c::{I2c as EmbeddedI2c, ErrorType},
    heapless::String,
    core::{
        fmt::Write,
        str::from_utf8,
        str::FromStr,
    },
    rand_core::RngCore,
    bme280::i2c::BME280,
    ssd1306::{
        I2CDisplayInterface,
        Ssd1306Async,
        prelude::DisplayRotation,
        size::DisplaySize128x64,
        mode::{DisplayConfigAsync, TerminalModeAsync, TerminalDisplaySizeAsync},
    },
    embedded_io_async::Write as EmbeddedWriter,
    static_cell::StaticCell,
    display_interface::AsyncWriteOnlyDataCommand,
    {defmt_rtt as _, panic_probe as _},
};

static DISPLAY: DisplayMessage = DisplayMessage::new();
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

struct OledSsd1306<DI, SIZE> {
    oled: Ssd1306Async<DI, SIZE, TerminalModeAsync>,
    display: &'static DisplayMessage,
    ip_adress: &'static PicoAddress,
}

impl<DI, SIZE> OledSsd1306<DI, SIZE>
where
    DI: AsyncWriteOnlyDataCommand,
    SIZE: TerminalDisplaySizeAsync, 
{
    fn new(oled: Ssd1306Async<DI, SIZE, TerminalModeAsync>, display: &'static DisplayMessage, ip_adress: &'static PicoAddress) -> Self {
        Self {
            oled,
            display,
            ip_adress,
        }
    }
    
    async fn init(&mut self) {
        loop {
            match self.oled.init().await{
                Ok(()) => {
                    log::warn!("Display has been Initialized");
                    self.oled.clear().await.unwrap();
                    self.set_title().await;
                    break;
                }
                Err(e) => {
                    log::warn!("Write Error: {:?}", e);
                }
            }
            Timer::after(Duration::from_millis(500)).await;
        }   
    }

    async fn set_title(&mut self) {
        let mut title = String::<32>::new();
        let mut second = String::<32>::new();
        let mut third = String::<32>::new();
        
        let ip = self.ip_adress.get_ip().await;
        
        write!(&mut title, "{}.{}.{}.{}         ", ip.octets()[0], ip.octets()[1], ip.octets()[2], ip.octets()[3]).unwrap();
        write!(&mut second, "                   ").unwrap();
        write!(&mut third,  "                   ").unwrap();
    
        self.display.set_data(title, 0, 0).await;
        self.display.set_data(second, 1, 0).await;
        self.display.set_data(third, 2, 0).await;
    }

    async fn write_available_message(&mut self) {
        let data = self.display.state.receive().await;
        let set_pos_status = self.oled.set_position(data.col, data.row).await;
        let write_str_status = self.oled.write_str(&data.message).await;

        if set_pos_status.is_err() || write_str_status.is_err() {
            log::info!("{:?}", set_pos_status);
            log::info!("{:?}", write_str_status);
            self.init().await;
        }
    }
}

struct Sensor<I2C> {
    sensor: BME280<I2C>,
    display: &'static DisplayMessage,
    state: &'static BmeState,
    delay: embassy_time::Delay,
}

impl <I2C> Sensor <I2C>
where
    I2C: EmbeddedI2c + ErrorType, 
{
    fn new(sensor: BME280<I2C>, display: &'static DisplayMessage, state: &'static BmeState) -> Self {
        Self {
            sensor,
            display,
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

    async fn write_to_display(&mut self, temp: f32, humidity: f32, pressure:f32) {
        let mut temp_str = String::<32>::new();
        let mut humidity_str = String::<32>::new();
        let mut pressure_str = String::<32>::new();

        write!(&mut temp_str,     "Temp:{:.2} C ", temp).unwrap();
        write!(&mut humidity_str, "RH  :{:.2} % ", humidity).unwrap();
        write!(&mut pressure_str, "P   :{:.3} atm ", pressure).unwrap();

        self.display.set_data(temp_str, 5, 0).await;
        self.display.set_data(humidity_str, 6, 0).await;
        self.display.set_data(pressure_str, 7, 0).await;
    }

    async fn measure(&mut self) {
        match self.sensor.measure(&mut self.delay) {
            Ok(data) => {
                self.write_to_display(data.temperature, data.humidity, (data.pressure*0.9869233)/100000.0).await;
                self.state.set_data(data.temperature, data.humidity, (data.pressure*0.9869233)/100000.0).await;
            },
            Err(err) => {
                log::info!("{:?}", err);
                self.write_to_display(0.0, 0.0, 0.0).await;
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
async fn display_task(p: DisplayResources) {
    let i2c = I2c::new_async(p.I2C_CH, p.SCL_PIN, p.SDA_PIN, Irqs, I2cConfig::default());

    let interface = I2CDisplayInterface::new(i2c);
    let display = Ssd1306Async::new(interface, DisplaySize128x64, DisplayRotation::Rotate0).into_terminal_mode();
    let mut oled = OledSsd1306::new(display, &DISPLAY, &IP_ADDRESS);
    
    oled.init().await;

    loop {
        oled.write_available_message().await;
    }
}

#[embassy_executor::task]
async fn bme_task(p: BmeResources) {
    let i2c = I2c::new_async(p.I2C_CH, p.SCL_PIN, p.SDA_PIN, Irqs, I2cConfig::default());

    let mut bme280 = Sensor::new(BME280::new_primary(i2c), &DISPLAY, &BME);
    let mut tick = Ticker::every(Duration::from_millis(500));

    bme280.init().await;

    loop {
        bme280.measure().await;
        tick.next().await;
    }
}

#[embassy_executor::task]
async fn pico_on_timer() {
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
    spawner.must_spawn(display_task(ph.display_resources));
    spawner.must_spawn(pico_on_timer());
    spawner.must_spawn(bme_task(ph.bme_resources));
    
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
    let html_str = from_utf8(HTML_BYTES).unwrap();
    
    led_toggle_status = false;

    loop {
        let mut socket = TcpSocket::new(stack, &mut rx_buffer, &mut tx_buffer);
        let mut ip_str = String::<32>::new();
        let ip = IP_ADDRESS.get_ip().await;
        write!(&mut ip_str, "{}.{}.{}.{}         ", ip.octets()[0], ip.octets()[1], ip.octets()[2], ip.octets()[3]).unwrap();
        DISPLAY.set_data(ip_str, 0, 0).await;

        log::info!("Listening on TCP: {} ...", TCP_PORT);
        let socket_timeout = 10;
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
                log::warn!("No Connection after {}s", socket_timeout);
                control.gpio_set(0, led_toggle_status).await;
                led_toggle_status  = !led_toggle_status;
                let mut scan_result = control.scan(ScanOptions::default()).await;
                let mut is_connected = false;

                loop {
                    match scan_result.next().await {
                        Some(value) => {
                            let arr = value.ssid;
                            let end = value.ssid_len as usize;
                            let ssid_char = core::str::from_utf8(&arr[..end]).expect("Valid UTF-8");
                            if ssid_char == "hades" {
                                log::info!("Found {}", ssid_char);
                                is_connected = true;
                                break;
                            }
                        }
                        None => {
                            log::info!("Missing SSID");
                            IP_ADDRESS.set_ip(Ipv4Address::new(0, 0, 0, 0)).await;
                            let mut ip_str = String::<32>::new();
                            let ip = Ipv4Address::new(0, 0, 0, 0);
                            write!(&mut ip_str, "{}.{}.{}.{}         ", ip.octets()[0], ip.octets()[1], ip.octets()[2], ip.octets()[3]).unwrap();
                            DISPLAY.set_data(ip_str, 0, 0).await;                    
                            break;
                        }
                    }
                    Timer::after_millis(100).await;
                }
        
                drop(scan_result);

                if !is_connected {
                    control.leave().await;
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

                }
                continue;
            }
        }

        log::info!("Received Connection from {:?}", socket.remote_endpoint());
        control.gpio_set(0, true).await;

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
                    write!(&mut pressure_str, "{:.3}", data.pressure).unwrap();
                    
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
