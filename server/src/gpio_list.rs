//! Resource Allocation Module
//!
//! This module defines the hardware resources used by various components of the robot.
//! It uses the `assign_resources` macro to allocate specific pins and peripherals to each component.

use {
    assign_resources::assign_resources,
    embassy_rp::{
        bind_interrupts,
        peripherals,
        peripherals::{PIO0, USB, I2C0, I2C1},
        pio::InterruptHandler as PioInterruptHandler,
        usb::InterruptHandler as UsbInterruptHandler,
        i2c::InterruptHandler as I2cInterruptHandler,
    },
};

assign_resources! {
    display_resources: DisplayResources {
        I2C_CH: I2C1,
        SCL_PIN: PIN_27,
        SDA_PIN: PIN_26,
    },
    bme_resources: BmeResources{
        I2C_CH: I2C0,
        SCL_PIN: PIN_13,
        SDA_PIN: PIN_12,
    }
    watchdog_resources: WatchdogResources{
        WATCHDOG: WATCHDOG,
    }
}

bind_interrupts!(pub struct Irqs {
    PIO0_IRQ_0 => PioInterruptHandler<PIO0>;
    USBCTRL_IRQ => UsbInterruptHandler<USB>;
    I2C0_IRQ => I2cInterruptHandler<I2C0>;
    I2C1_IRQ => I2cInterruptHandler<I2C1>;
});
