use crate::{Config, PhyType};
use esp_hal::gpio::{
    connect_high_to_peripheral, connect_low_to_peripheral, InputPin, InputSignal, OutputPin,
    OutputSignal,
};
use esp_hal::peripherals::{LPWR, USB_WRAP};

pub(crate) fn init_phy(config: &mut Config, usb_wrap: &USB_WRAP, rtc: &LPWR) {
    phy_hal_otg_conf(config, usb_wrap, rtc);

    if let PhyType::External(ext_config) = &mut config.phy_type {
        ext_config
            .vp
            .connect_input_to_peripheral(InputSignal::USB_EXTPHY_VP);
        ext_config
            .vm
            .connect_input_to_peripheral(InputSignal::USB_EXTPHY_VM);
        ext_config
            .rcv
            .connect_input_to_peripheral(InputSignal::USB_EXTPHY_RCV);
        ext_config
            .oen
            .connect_peripheral_to_output(OutputSignal::USB_EXTPHY_OEN);
        ext_config
            .vpo
            .connect_peripheral_to_output(OutputSignal::USB_EXTPHY_VPO);
        ext_config
            .vmo
            .connect_peripheral_to_output(OutputSignal::USB_EXTPHY_VMO);
    }

    phy_otg_set_mode_device();
    phy_otg_set_speed_device(usb_wrap);

    if let Some(pin) = &mut config.vbus_detection_pin {
        // Note: I'm following the ESP-IDF code from components/usb/include/esp_private/usb_phy.h
        // here.  It sets USB_SRP_BVALID and not USB_OTG_VBUSVALID.
        pin.connect_input_to_peripheral(InputSignal::USB_SRP_BVALID);
    }
}

fn phy_hal_otg_conf(config: &Config, usb_wrap: &USB_WRAP, rtc: &LPWR) {
    usb_wrap.otg_conf().modify(|_, w| {
        let w = if let PhyType::External(_) = config.phy_type {
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
