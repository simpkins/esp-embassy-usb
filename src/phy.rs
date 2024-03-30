use crate::{Config, PhyType};
use esp_hal::gpio::{connect_high_to_peripheral, connect_low_to_peripheral, InputSignal};
use esp_hal::peripherals::{LPWR, USB_WRAP};

pub(crate) fn init_phy(config: &Config, usb_wrap: &USB_WRAP, rtc: &LPWR) {
    phy_hal_otg_conf(config, usb_wrap, rtc);

    if let PhyType::External = config.phy_type {
        // TODO: Configure the pins for the external PHY.
        // phy_external_iopins_configure(phy_context->iopins)
    }

    phy_otg_set_mode_device();
    phy_otg_set_speed_device(usb_wrap);
    phy_otg_configure_iopins();
}

fn phy_hal_otg_conf(config: &Config, usb_wrap: &USB_WRAP, rtc: &LPWR) {
    usb_wrap.otg_conf().modify(|_, w| {
        let w = if let PhyType::External = config.phy_type {
            w.phy_sel().set_bit()
        } else {
            w.phy_sel().clear_bit()
        };
        w.usb_pad_enable()
            .set_bit()
            .clk_en()
            .set_bit()
            .ahb_clk_force_on()
            .set_bit()
            .phy_clk_force_on()
            .set_bit()
    });

    #[cfg(feature = "esp32s3")]
    {
        rtc.usb_conf().modify(|_, w| {
            let w = if let PhyType::External = config.phy_type {
                w.sw_usb_phy_sel().clear_bit()
            } else {
                w.sw_usb_phy_sel().set_bit()
            };
            w.sw_hw_usb_phy_sel().set_bit()
        });
    }
    #[cfg(not(feature = "esp32s3"))]
    {
        let _ = rtc; // Avoid an unused variable warning
    }
}

fn phy_otg_set_mode_device() {
    // Configure for device mode operation.
    connect_high_to_peripheral(InputSignal::USB_OTG_IDDIG); // connected connector is B side
    connect_high_to_peripheral(InputSignal::USB_SRP_BVALID); // HIGH to force USB device mode
    connect_high_to_peripheral(InputSignal::USB_OTG_VBUSVALID); // receiving a valid Vbus from device
    connect_low_to_peripheral(InputSignal::USB_OTG_AVALID);
}

fn phy_otg_set_speed_device(usb_wrap: &USB_WRAP) {
    // We always configure the device as full speed here.  (DP pulled up, no pull-up on DM)
    // If we wanted to advertise ourself as a low-speed device we should pull up DM instead of DP.
    //
    // We possibly would need to also set a different value for the devspd bits in the usb0.dcfg()
    // register.  Espressif doesn't provide much documentation on their core, and from some of the
    // comments in the tinyusb implementation it seems like the core may be fixed to full speed.
    usb_wrap.otg_conf().write(|w| {
        w.pad_pull_override()
            .set_bit()
            .dp_pullup()
            .set_bit()
            .dp_pulldown()
            .clear_bit()
            .dm_pullup()
            .clear_bit()
            .dm_pulldown()
            .clear_bit()
    });
}

fn phy_otg_configure_iopins() {
    // TODO: configure the I/O pins
    // This is needed for configuring the pin that is monitoring VBUS
}
