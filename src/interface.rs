use embedded_hal::blocking::spi::{Transfer, Write};
use embedded_hal::digital::v2::OutputPin;
use usb_device::{Result, UsbError};

pub struct SpiInterface<SPI, SS> {
    spi: SPI,
    ss: SS,
    bav: u8,
}

impl<SPI, SS> SpiInterface<SPI, SS>
where
    SPI: Transfer<u8> + Write<u8>,
    SS: OutputPin,
{
    pub fn new(spi: SPI, ss: SS) -> Self {
        Self {
            spi,
            ss,
            bav: 0,
        }
    }

    pub fn write(&mut self, reg: Reg, value: u8) -> Result<()> {
        self.ss.set_low().ok();
        self.spi
            .write(&mut [((reg as u8) << 3) | 0x02, value])
            .map_err(|_| UsbError::Platform)?;
        self.ss.set_high().ok();

        Ok(())
    }

    pub fn ackstat(&mut self) -> Result<()> {
        self.ss.set_low().ok();
        self.spi.write(&mut [0x01]).map_err(|_| UsbError::Platform)?;
        self.ss.set_high().ok();

        Ok(())
    }

    pub fn write_ackstat(&mut self, reg: Reg, value: u8) -> Result<()> {
        self.ss.set_low().ok();
        self.spi
            .write(&mut [((reg as u8) << 3) | 0x03, value])
            .map_err(|_| UsbError::Platform)?;
        self.ss.set_high().ok();

        Ok(())
    }

    #[allow(unused)]
    pub fn reg_dump(&mut self) -> Result<()> {
        for reg in 5..21 {
            let mut buf = [((reg as u8) << 3), 0x00];

            self.ss.set_low().ok();
            self.spi
                .transfer(&mut buf)
                .map_err(|_| UsbError::Platform)?;
            self.ss.set_high().ok();

            rtt::rprintln!("R{} = {:02x}", reg, buf[1]);
        }

        Ok(())
    }

    pub fn read(&mut self, reg: Reg) -> Result<u8> {
        let mut buf = [((reg as u8) << 3), 0x00];

        self.ss.set_low().ok();
        self.spi
            .transfer(&mut buf)
            .map_err(|_| UsbError::Platform)?;
        self.ss.set_high().ok();

        Ok(buf[1])
    }

    pub fn set_bits(&mut self, reg: Reg, bits: u8) -> Result<()> {
        let v = self.read(reg)?;
        self.write(reg, v | bits)
    }

    pub fn clear_bits(&mut self, reg: Reg, bits: u8) -> Result<()> {
        let v = self.read(reg)?;
        self.write(reg, v & !bits)
    }

    pub fn read_status(&mut self) -> Result<u8> {
        let mut buf = [0x00];

        self.ss.set_low().ok();
        self.spi
            .transfer(&mut buf)
            .map_err(|_| UsbError::Platform)?;
        self.ss.set_high().ok();

        let status = buf[0];

        self.bav = status;

        Ok(status)
    }

    pub fn write_fifo(&mut self, fifo: Infifo, data: &[u8]) -> Result<()> {
        let irq = match fifo {
            Infifo::Ep0in => Epirq::In0bav,
            Infifo::Ep2in => Epirq::In2bav,
            Infifo::Ep3in => Epirq::In3bav,
        } as u8;

        if fifo == Infifo::Ep0in && data.is_empty() {
            // For empty EP0 writes which are always either a status stage or the end of an IN
            // transfer, there is no need to write any data, just set ACKSTAT.
            self.ackstat()?;
            return Ok(());
        }

        if self.bav & irq == 0 {
            return Err(UsbError::WouldBlock);
        }

        self.ss.set_low().ok();
        self.spi
            .write(&[((fifo as u8) << 3) | 0x02])
            .map_err(|_| UsbError::Platform)?;
        self.spi.write(&data).map_err(|_| UsbError::Platform)?;
        self.ss.set_high().ok();

        let bcreg = match fifo {
            Infifo::Ep0in => Reg::Ep0bc,
            Infifo::Ep2in => Reg::Ep2inbc,
            Infifo::Ep3in => Reg::Ep3inbc,
        };

        if fifo == Infifo::Ep0in {
            // EP0 IN is only written to in successful transfers - otherwise it's stalled. Always
            // write ACKSTAT if writing to EP0 IN.

            self.write_ackstat(bcreg, data.len() as u8)?;
        } else {
            self.write(bcreg, data.len() as u8)?;
        }

        //rtt::rprintln!("wrote {:02x?} bytes to {:?} {:?} stat {:02x}", data, fifo, bcreg, self.read_status()?);

        Ok(())
    }

    pub fn read_fifo(&mut self, fifo: Outfifo, data: &mut [u8]) -> Result<usize> {
        let irq = match fifo {
            Outfifo::Ep0out => Epirq::Out0dav,
            Outfifo::Ep1out => Epirq::Out1dav,
            Outfifo::Sud => Epirq::Sudav,
        } as u8;

        if self.read(Reg::Epirq)? & irq == 0 {
            return Err(UsbError::WouldBlock);
        }

        let count = match fifo {
            Outfifo::Ep0out => self.read(Reg::Ep0bc)? as usize,
            Outfifo::Ep1out => self.read(Reg::Ep1outbc)? as usize,
            Outfifo::Sud => 8,
        };

        if data.len() < count {
            return Err(UsbError::WouldBlock);
        }

        self.ss.set_low().ok();
        self.spi
            .write(&[(fifo as u8) << 3])
            .map_err(|_| UsbError::Platform)?;
        self.spi
            .transfer(&mut data[..count])
            .map_err(|_| UsbError::Platform)?;
        self.ss.set_high().ok();

        self.write(Reg::Epirq, irq)?;

        //rtt::rprintln!("read {:02x?} bytes from {:?} stat {:02x}", &data[..count], fifo, self.read_status()?);

        Ok(count)
    }

    pub fn wait_irq(&mut self, irq: Usbirq) -> Result<()> {
        loop {
            let curirq = self.read(Reg::Usbirq)?;

            if curirq & (irq as u8) != 0 {
                self.write(Reg::Usbirq, irq as u8)?;
                break;
            }
        }

        Ok(())
    }
}

#[repr(u8)]
#[derive(Eq, PartialEq, Copy, Clone, Debug)]
#[allow(unused)]
pub enum Reg {
    Ep0bc = 5,
    Ep1outbc = 6,
    Ep2inbc = 7,
    Ep3inbc = 8,
    Epstalls = 9,
    Clrtogs = 10,
    Epirq = 11,
    Epien = 12,
    Usbirq = 13,
    Usbien = 14,
    Usbctl = 15,
    Cpuctl = 16,
    Pinctl = 17,
    Revision = 18,
    Fnaddr = 19,
    Iopins = 20,
}

#[repr(u8)]
#[derive(Eq, PartialEq, Copy, Clone, Debug)]
#[allow(unused)]
pub enum Infifo {
    Ep0in = 0,
    Ep2in = 2,
    Ep3in = 3,
}

#[repr(u8)]
#[derive(Eq, PartialEq, Copy, Clone, Debug)]
#[allow(unused)]
pub enum Outfifo {
    Ep0out = 0,
    Ep1out = 1,
    Sud = 4,
}

#[repr(u8)]
#[derive(Eq, PartialEq, Copy, Clone, Debug)]
#[allow(unused)]
pub enum Epirq {
    In0bav = (1 << 0),
    Out0dav = (1 << 1),
    Out1dav = (1 << 2),
    In2bav = (1 << 3),
    In3bav = (1 << 4),
    Sudav = (1 << 5),
}

impl From<Epirq> for u8 {
    fn from(irq: Epirq) -> u8 {
        irq as u8
    }
}

#[repr(u8)]
#[derive(Eq, PartialEq, Copy, Clone, Debug)]
#[allow(unused)]
pub enum Usbirq {
    Oscok = (1 << 0),
    Rwudn = (1 << 1),
    Busact = (1 << 2),
    Ures = (1 << 3),
    Susp = (1 << 4),
    Novbus = (1 << 5),
    Vbus = (1 << 6),
    Uresdn = (1 << 7),
}

impl From<Usbirq> for u8 {
    fn from(irq: Usbirq) -> u8 {
        irq as u8
    }
}
