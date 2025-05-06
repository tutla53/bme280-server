#![no_std]
#![no_main]

mod gpio_list;

use {
    crate::gpio_list::{
        Irqs,
        AssignedResources,
        DisplayResources,
        BmeResources,
        WatchdogResources,
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
    embassy_futures::select::{select4, select, Either4, Either},
    embassy_rp::{
        watchdog::Watchdog,
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

static DISPLAY: DisplayMessage = DisplayMessage::new();
static BME: BmeState = BmeState::new();
static IP_ADDRESS: PicoAddress = PicoAddress::new();
static BME_WATCHDOG: Channel<CriticalSectionRawMutex, bool, 50> = Channel::new();
static DISPLAY_WATCHDOG: Channel<CriticalSectionRawMutex, bool, 50> = Channel::new();
static TIMER_WATCHDOG: Channel<CriticalSectionRawMutex, bool, 50> = Channel::new();
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
    message: String<MAX_OLED_CHAR>,
    row: u8,
    col: u8,
}

struct DisplayMessage {
    state : Channel<CriticalSectionRawMutex, MessageFormat, 32>,
}

impl DisplayMessage {
    const fn new() -> Self {
        Self {
            state: Channel::new(),
        }
    }

    fn set_data(&self, message: String<MAX_OLED_CHAR>, row: u8, col:u8) {
        
        let new_state = MessageFormat {
            message: message,
            row: row, 
            col: col,
        };

        match self.state.try_send(new_state) {
            Ok(()) => {},
            Err(_) => {
                log::warn!("Display channel full!");
            },
        };
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
                    self.safe_clear().await;
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

    async fn safe_clear(&mut self) {
        let mut retries = 0;
        loop {
            match self.oled.clear().await {
                Ok(_) => break,
                Err(e) if retries < 3 => {
                    log::warn!("Clear failed (retry {}): {:?}", retries, e);
                    Timer::after(Duration::from_millis(100 * retries)).await;
                    retries += 1;
                }
                Err(e) => {
                    log::error!("Permanent clear failure: {:?}", e);
                    break;
                }
            }
        }
    }

    async fn set_title(&mut self) {
        let mut title = String::<MAX_OLED_CHAR>::new();
        let mut blank = String::<MAX_OLED_CHAR>::new();
        
        let ip = self.ip_adress.get_ip().await;
        
        safe_write!(&mut title, "{}.{}.{}.{}", ip.octets()[0], ip.octets()[1], ip.octets()[2], ip.octets()[3]);
        safe_write!(&mut blank, "");
    
        self.display.set_data(title, 0, 0);
        self.display.set_data(blank.clone(), 1, 0);
        self.display.set_data(blank.clone(), 2, 0);
    }

    async fn write_available_message(&mut self) {
        let mut data = self.display.state.receive().await;

        // Add Space to Clear Uwanted Character on OLED
        let _ = write!(data.message, "{: <1$}", "", MAX_OLED_CHAR - data.message.len());
        
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
        let mut temp_str = String::<MAX_OLED_CHAR>::new();
        let mut humidity_str = String::<MAX_OLED_CHAR>::new();
        let mut pressure_str = String::<MAX_OLED_CHAR>::new();
        
        safe_write!(&mut temp_str,     "Temp:{:.2} C", temp);
        safe_write!(&mut humidity_str, "RH  :{:.2} %", humidity);
        safe_write!(&mut pressure_str, "P   :{:.3} atm", pressure);

        self.display.set_data(temp_str, 5, 0);
        self.display.set_data(humidity_str, 6, 0);
        self.display.set_data(pressure_str, 7, 0);
    }

    async fn measure(&mut self) {
        match self.sensor.measure(&mut self.delay) {
            Ok(data) => {
                self.state.set_sensor_data(data.temperature, data.humidity, (data.pressure*0.9869233)/100000.0).await;
                self.write_to_display(data.temperature, data.humidity, (data.pressure*0.9869233)/100000.0).await;
            },
            Err(err) => {
                log::info!("{:?}", err);
                self.state.set_sensor_data(0.0, 0.0, 0.0).await;

                let mut temp_str = String::<MAX_OLED_CHAR>::new();
                let mut humidity_str = String::<MAX_OLED_CHAR>::new();
                let mut pressure_str = String::<MAX_OLED_CHAR>::new();
                
                safe_write!(&mut temp_str,     "Temp: Error");
                safe_write!(&mut humidity_str, "RH  : Error");
                safe_write!(&mut pressure_str, "P   : Error");

                self.display.set_data(temp_str, 5, 0);
                self.display.set_data(humidity_str, 6, 0);
                self.display.set_data(pressure_str, 7, 0);
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
        // Create futures for each watchdog channel
        let bme_fut = BME_WATCHDOG.receive();
        let timer_fut = TIMER_WATCHDOG.receive();
        let display_fut = DISPLAY_WATCHDOG.receive();
        let wifi_fut = WIFI_WATCHDOG.receive();

        // Wait for all signals with timeout
        match select4(
            select(bme_fut, Timer::after(Duration::from_secs(5))),
            select(timer_fut, Timer::after(Duration::from_secs(5))),
            select(display_fut, Timer::after(Duration::from_secs(5))),
            select(wifi_fut, Timer::after(Duration::from_secs(5))),
        ).await {
            Either4::First(value) => {
                match value {
                    Either::First(_) => {},
                    Either::Second(_) => {
                        break;
                    }
                }
            },
            Either4::Second(value) => {
                match value {
                    Either::First(_) => {},
                    Either::Second(_) => {
                        break;
                    }
                }
            },
            Either4::Third(value) => {
                match value {
                    Either::First(_) => {},
                    Either::Second(_) => {
                        break;
                    }
                }
            },
            Either4::Fourth(value) => {
                match value {
                    Either::First(_) => {},
                    Either::Second(_) => {
                        break;
                    }
                }
            },
        }

        watchdog.feed();
    }
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
        let _ = DISPLAY_WATCHDOG.try_send(true);
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
        let _ = BME_WATCHDOG.try_send(true);
        tick.next().await;
    }
}

#[embassy_executor::task]
async fn pico_on_timer() {
    let mut tick = Ticker::every(Duration::from_secs(1));
    let uptime = Instant::now();
    
    loop {
        let uptime_format = TimeFormat::get_format(uptime.elapsed());
        let mut uptime_str = String::<MAX_OLED_CHAR>::new();
        safe_write!(&mut uptime_str,   "{}d:{:02}h:{:02}m:{:02}s", uptime_format.days, uptime_format.hours, uptime_format.minutes, uptime_format.seconds);
        DISPLAY.set_data(uptime_str, 4, 0);
        let _ = TIMER_WATCHDOG.try_send(true);
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
    spawner.must_spawn(display_task(ph.display_resources));
    spawner.must_spawn(pico_on_timer());
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
        DISPLAY.set_data(ip_str, 0, 0);

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
                            DISPLAY.set_data(ip_str, 0, 0);                    
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