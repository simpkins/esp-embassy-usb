This is a work-in-progress embassy-usb-driver for ESP32-S3 and ESP32-S2 chips.

I have written a C++ USB implementation for these chips in the past
(https://github.com/simpkins/ausb), and this code was created through a mix of
porting the ESP32 logic to Rust, and partly from looking at the embassy-stm32
crate for examples of an existing embassy-usb-driver.

This code isn't quite complete yet, and there are still plenty of TODOs in the
code, but the HID example generally appears to function as expected.

Ideally this code probably should be unified with the embassy-stm32
implementation, since the USB core is the same.  The main item that needs to be
resolved first is just the different register access APIs for the `esp-hal` vs
the stm32 registers.  We should be able to just pick one API however and cast
the register block address to the desired type.  At some point I might try and
look to see what parts of this implementation are worth upstreaming into the
embassy-stm32 implementation.
