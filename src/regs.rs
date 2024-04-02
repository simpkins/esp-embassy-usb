use static_assertions::const_assert_eq;

#[cfg(feature = "esp32s2")]
use esp32s2 as pac;
#[cfg(feature = "esp32s3")]
use esp32s3 as pac;
use pac::usb0::*;

pub const NUM_IN_ENDPOINTS: usize = 7;
pub const NUM_OUT_ENDPOINTS: usize = 7;

// An alternate version of the esp_hal::peripherals::USB0 register block.
// The one in the esp-pacs crate has bugs.
//
// I'm working to submit PRs for some of these issues
// https://github.com/embassy-rs/embassy/issues/2751
//
// In the meantime, this is a bit of a hack to unblock my own local development
// esp32s2 and esp32s3 both appear to use the same register layout for these fields.
#[repr(C)]
pub struct Usb0RegisterBlock {
    gotgctl: GOTGCTL,     // 0x0000
    gotgint: GOTGINT,     // 0x0004
    gahbcfg: GAHBCFG,     // 0x0008
    gusbcfg: GUSBCFG,     // 0x000c
    grstctl: GRSTCTL,     // 0x0010
    gintsts: GINTSTS,     // 0x0014
    gintmsk: GINTMSK,     // 0x0018
    grxstsr: GRXSTSR,     // 0x001c
    grxstsp: GRXSTSP,     // 0x0020
    grxfsiz: GRXFSIZ,     // 0x0024
    gnptxfsiz: GNPTXFSIZ, // 0x0028
    gnptxsts: GNPTXSTS,   // 0x002c
    _reserved0030: [u8; 0x10],
    gsnpsid: GSNPSID, // 0x0040
    ghwcfg1: GHWCFG1, // 0x0044
    ghwcfg2: GHWCFG2, // 0x0048
    ghwcfg3: GHWCFG3, // 0x004c
    ghwcfg4: GHWCFG4, // 0x0050
    _reserved0054: [u8; 0xac],
    hptxfsiz: HPTXFSIZ,     // 0x0100
    dieptxf: [DIEPTXF1; 4], // 0x0104
    _reserved0114: [u8; 0x6ec],
    dcfg: DCFG, // 0x0800
    dctl: DCTL, // 0x0804
    dsts: DSTS, // 0x0808
    _reserved080c: [u8; 0x04],
    diepmsk: DIEPMSK,   // 0x0810
    doepmsk: DOEPMSK,   // 0x0814
    daint: DAINT,       // 0x0818
    daintmsk: DAINTMSK, // 0x081c
    _reserved0820: [u8; 0x08],
    dvbusdis: DVBUSDIS,     // 0x0828
    dvbuspulse: DVBUSPULSE, // 0x082c
    dthrctl: DTHRCTL,       // 0x0830
    diepempmsk: DIEPEMPMSK, // 0x0834
    _reserved0838: [u8; 0x0c8],
    in_ep_reg: [InEpRegisterBlock; NUM_IN_ENDPOINTS], // 0x0900 to 0x09e0
    _reserved09e0: [u8; 0x120],
    out_ep_reg: [OutEpRegisterBlock; NUM_IN_ENDPOINTS], // 0x0b00 to 0x0be0
    _reserved0be0: [u8; 0x420],
    fifo: [FifoRegisterBlock; 16], // 0x1000 to 0x2000
}

// Sanity check our layout
const_assert_eq!(core::mem::offset_of!(Usb0RegisterBlock, hptxfsiz), 0x100);
const_assert_eq!(core::mem::offset_of!(Usb0RegisterBlock, dcfg), 0x800);
const_assert_eq!(core::mem::offset_of!(Usb0RegisterBlock, in_ep_reg), 0x900);
const_assert_eq!(core::mem::offset_of!(Usb0RegisterBlock, out_ep_reg), 0xb00);
const_assert_eq!(core::mem::offset_of!(Usb0RegisterBlock, fifo), 0x1000);

#[repr(C)]
pub struct InEpRegisterBlock {
    diepctl: DIEPCTL1,
    _reserved04: [u8; 0x04],
    diepint: DIEPINT1,
    _reserved0c: [u8; 0x04],
    dieptsiz: DIEPTSIZ1,
    diepdma: DIEPDMA1,
    dtxfsts: DTXFSTS1,
    diepdmab: DIEPDMAB1,
}

#[repr(C)]
pub struct InEp0RegisterBlock {
    diepctl: DIEPCTL0,
    _reserved04: [u8; 0x04],
    diepint: DIEPINT0,
    _reserved0c: [u8; 0x04],
    dieptsiz: DIEPTSIZ0,
    diepdma: DIEPDMA0,
    dtxfsts: DTXFSTS0,
    diepdmab: DIEPDMAB0,
}

#[repr(C)]
pub struct OutEpRegisterBlock {
    doepctl: DOEPCTL1,
    _reserved04: [u8; 0x04],
    doepint: DOEPINT1,
    _reserved0c: [u8; 0x04],
    doeptsiz: DOEPTSIZ1,
    doepdma: DOEPDMA1,
    _reserved28: [u8; 0x04],
    doepdmab: DOEPDMAB1,
}

#[repr(C)]
pub struct OutEp0RegisterBlock {
    doepctl: DOEPCTL0,
    _reserved04: [u8; 0x04],
    doepint: DOEPINT0,
    _reserved0c: [u8; 0x04],
    doeptsiz: DOEPTSIZ0,
    doepdma: DOEPDMA0,
    _reserved28: [u8; 0x04],
    doepdmab: DOEPDMAB0,
}

#[repr(C)]
pub struct FifoRegisterBlock {
    // Each word in FIFO the range behaves the same.
    word: [vcell::VolatileCell<u32>; 1024],
}

impl FifoRegisterBlock {
    pub fn read(&self) -> u32 {
        self.word[0].get()
    }
    pub fn write(&self, value: u32) {
        self.word[0].set(value)
    }
}

impl Usb0RegisterBlock {
    pub const fn gotgctl(&self) -> &GOTGCTL {
        &self.gotgctl
    }
    pub const fn gotgint(&self) -> &GOTGINT {
        &self.gotgint
    }
    pub const fn gahbcfg(&self) -> &GAHBCFG {
        &self.gahbcfg
    }
    pub const fn gusbcfg(&self) -> &GUSBCFG {
        &self.gusbcfg
    }
    pub const fn grstctl(&self) -> &GRSTCTL {
        &self.grstctl
    }
    pub const fn gintsts(&self) -> &GINTSTS {
        &self.gintsts
    }
    pub const fn gintmsk(&self) -> &GINTMSK {
        &self.gintmsk
    }
    pub const fn grxstsr(&self) -> &GRXSTSR {
        &self.grxstsr
    }
    pub const fn grxstsp(&self) -> &GRXSTSP {
        &self.grxstsp
    }
    pub const fn grxfsiz(&self) -> &GRXFSIZ {
        &self.grxfsiz
    }
    pub const fn gnptxfsiz(&self) -> &GNPTXFSIZ {
        &self.gnptxfsiz
    }
    pub const fn gnptxsts(&self) -> &GNPTXSTS {
        &self.gnptxsts
    }
    pub const fn gsnpsid(&self) -> &GSNPSID {
        &self.gsnpsid
    }
    pub const fn dieptxf(&self, n: usize) -> &DIEPTXF1 {
        &self.dieptxf[n]
    }
    pub const fn dcfg(&self) -> &DCFG {
        &self.dcfg
    }
    pub const fn dctl(&self) -> &DCTL {
        &self.dctl
    }
    pub const fn dsts(&self) -> &DSTS {
        &self.dsts
    }
    pub const fn diepmsk(&self) -> &DIEPMSK {
        &self.diepmsk
    }
    pub const fn doepmsk(&self) -> &DOEPMSK {
        &self.doepmsk
    }
    pub const fn daint(&self) -> &DAINT {
        &self.daint
    }
    pub const fn daintmsk(&self) -> &DAINTMSK {
        &self.daintmsk
    }
    pub const fn dvbusdis(&self) -> &DVBUSDIS {
        &self.dvbusdis
    }
    pub const fn dvbuspulse(&self) -> &DVBUSPULSE {
        &self.dvbuspulse
    }
    pub const fn dthrctl(&self) -> &DTHRCTL {
        &self.dthrctl
    }
    pub const fn diepempmsk(&self) -> &DIEPEMPMSK {
        &self.diepempmsk
    }
    pub const fn in_ep(&self, n: usize) -> &InEpRegisterBlock {
        // Note: we do allow returning InEpRegisterBlock for endpoint 0 here,
        // even though it technically uses a slightly different register layout.
        //
        // The EP0 layout is very similar, but has a few minor differences (e.g., different bit
        // width for the max packet size, disable bit is read-only, etc).  When transmitting
        // packets it is convenient for users to be able to treat the EP0 registers just like other
        // endpoints, since the behavior here is the same.
        &self.in_ep_reg[n]
    }
    pub const fn in_ep0(&self) -> &InEp0RegisterBlock {
        unsafe { core::mem::transmute(&self.in_ep_reg[0]) }
    }
    pub const fn out_ep(&self, n: usize) -> &OutEpRegisterBlock {
        // Similar to in_ep(), we do allow returning OutEpRegisterBlock for endpoint 0,
        // even though the register layout for EP0 technically does have some minor differences
        // from the other endpoints.
        &self.out_ep_reg[n]
    }
    pub const fn out_ep0(&self) -> &OutEp0RegisterBlock {
        unsafe { core::mem::transmute(&self.out_ep_reg[0]) }
    }
    pub const fn fifo(&self, n: usize) -> &FifoRegisterBlock {
        &self.fifo[n]
    }
}

pub unsafe fn get_usb0_register_block() -> &'static Usb0RegisterBlock {
    const REGISTERS: *const () = esp_hal::peripherals::USB0::ptr() as *const ();
    let address = REGISTERS as usize;
    &*(address as *const Usb0RegisterBlock)
}

impl InEpRegisterBlock {
    pub const fn diepctl(&self) -> &DIEPCTL1 {
        &self.diepctl
    }
    pub const fn diepint(&self) -> &DIEPINT1 {
        &self.diepint
    }
    pub const fn dieptsiz(&self) -> &DIEPTSIZ1 {
        &self.dieptsiz
    }
    pub const fn diepdma(&self) -> &DIEPDMA1 {
        &self.diepdma
    }
    pub const fn dtxfsts(&self) -> &DTXFSTS1 {
        &self.dtxfsts
    }
    pub const fn diepdmab(&self) -> &DIEPDMAB1 {
        &self.diepdmab
    }
}

impl InEp0RegisterBlock {
    pub const fn diepctl(&self) -> &DIEPCTL0 {
        &self.diepctl
    }
    pub const fn diepint(&self) -> &DIEPINT0 {
        &self.diepint
    }
    pub const fn dieptsiz(&self) -> &DIEPTSIZ0 {
        &self.dieptsiz
    }
    pub const fn diepdma(&self) -> &DIEPDMA0 {
        &self.diepdma
    }
    pub const fn dtxfsts(&self) -> &DTXFSTS0 {
        &self.dtxfsts
    }
    pub const fn diepdmab(&self) -> &DIEPDMAB0 {
        &self.diepdmab
    }
}

impl OutEpRegisterBlock {
    pub const fn doepctl(&self) -> &DOEPCTL1 {
        &self.doepctl
    }
    pub const fn doepint(&self) -> &DOEPINT1 {
        &self.doepint
    }
    pub const fn doeptsiz(&self) -> &DOEPTSIZ1 {
        &self.doeptsiz
    }
    pub const fn doepdma(&self) -> &DOEPDMA1 {
        &self.doepdma
    }
    pub const fn doepdmab(&self) -> &DOEPDMAB1 {
        &self.doepdmab
    }
}

impl OutEp0RegisterBlock {
    pub const fn doepctl(&self) -> &DOEPCTL0 {
        &self.doepctl
    }
    pub const fn doepint(&self) -> &DOEPINT0 {
        &self.doepint
    }
    pub const fn doeptsiz(&self) -> &DOEPTSIZ0 {
        &self.doeptsiz
    }
    pub const fn doepdma(&self) -> &DOEPDMA0 {
        &self.doepdma
    }
    pub const fn doepdmab(&self) -> &DOEPDMAB0 {
        &self.doepdmab
    }
}
