use embedded_hal::blocking::spi::{Transfer, Write};
use embedded_hal::digital::v2::OutputPin;
use usb_device::{
    endpoint::{EndpointAddress, EndpointConfig, OutPacketType},
    usbcore::{self, PollResult},
    Result, UsbDirection, UsbError,
};

use crate::endpoint::*;
use crate::interface::*;

pub struct UsbCore<SPI, SS, EV> {
    i: SpiInterface<SPI, SS>,
    event: EV,
    buffers: &'static Buffers,
    status_mask: u8,
}

impl<SPI, SS, EV> UsbCore<SPI, SS, EV>
where
    SPI: Transfer<u8> + Write<u8>,
    SS: OutputPin,
    EV: Event,
{
    pub fn new(spi: SPI, ss: SS, event: EV, buffers: &'static Buffers) -> Self {
        Self {
            i: SpiInterface::new(spi, ss),
            buffers,
            event,
            status_mask: 0b11100110,
        }
    }

    pub fn reset(&mut self) -> Result<()> {
        self.i.write(Reg::Pinctl, 0b00010000)?; // FDUPSPI
        self.i.write(Reg::Usbctl, 0b00100000)?; // CHIPRES
        self.i.write(Reg::Usbctl, 0b00000000)?; // Clear CHIPRES

        self.i.wait_irq(Usbirq::Oscok)?;

        self.i.reg_dump()?;

        self.i.write(Reg::Epirq, 0b00111111)?; // Clear all interrupts
        self.i.write(Reg::Epien, 0b00111111)?; // Enable all interrupts
        self.i.write(Reg::Usbirq, 0b11111111)?; // Clear all interrupts

        self.status_mask = 0b11100110;

        Ok(())
    }

    pub fn revision(&mut self) -> Result<u8> {
        self.i.read(Reg::Revision)
    }

    fn poll_in_buffer(
        &mut self,
        index: usize,
        addr: EndpointAddress,
        stalls: u8,
        fifo: Infifo,
        bit: u8) -> Result<()>
    {
        let buf = &self.buffers.ep_in[index];

        if buf.state() == State::Stall {
            if stalls & bit == 0 {
                self.i.set_bits(Reg::Epstalls, ep_stall_bits(addr))?;
            }

            return Ok(());
        }

        match buf.read_data(false, |d| self.i.write_fifo(fifo, d)) {
            Ok(_) => {
                self.status_mask |= bit;
            },
            Err(UsbError::WouldBlock) => { }
            Err(e) => return Err(e),
        };

        Ok(())
    }
}

impl<SPI, CS, EV> usbcore::UsbCore for UsbCore<SPI, CS, EV>
where
    SPI: Transfer<u8> + Write<u8>,
    CS: OutputPin,
    EV: Event,
{
    type EndpointOut = UsbEndpointOut<EV>;

    type EndpointIn = UsbEndpointIn<EV>;

    type EndpointAllocator = UsbEndpointAllocator<EV>;

    fn create_allocator(&mut self) -> Self::EndpointAllocator {
        UsbEndpointAllocator::new(self.buffers, self.event.clone())
    }

    fn enable(&mut self, _allocator: Self::EndpointAllocator) -> Result<()> {
        self.i.set_bits(Reg::Usbctl, 0b00001000)?; // CONNECT

        Ok(())
    }

    fn reset(&mut self) -> Result<()> {
        //self.i.write(Reg::Clrtogs, 0b11111100)?; // Disable all endpoints and clear toggle bits
        self.i.write(Reg::Fnaddr, 0)?; // Clear address

        self.i.write(Reg::Epirq, 0b00111111)?; // Clear all interrupts
        self.i.write(Reg::Epien, 0b00111111)?; // Enable all interrupts
        self.i.write(Reg::Usbirq, 0b11111111)?; // Clear all interrupts

        self.status_mask = 0b11100110;

        //self.i.reg_dump()?;

        self.buffers.ep_out[0].set_state(State::Empty);
        self.buffers.ep_out[1].set_state(State::Disabled);
        self.buffers.ep_in[0].set_state(State::Empty);
        self.buffers.ep_in[1].set_state(State::Disabled);
        self.buffers.ep_in[2].set_state(State::Disabled);

        Ok(())
    }

    fn poll(&mut self) -> Result<PollResult> {
        let status = self.i.read_status()? & self.status_mask;

        let stalls = self.i.read(Reg::Epstalls)?;

        self.poll_in_buffer(
            0,
            EndpointAddress::from_parts(0, UsbDirection::In),
            stalls,
            Infifo::Ep0in,
            Epirq::In0bav as u8)?;

        self.poll_in_buffer(
            1,
            EndpointAddress::from_parts(2, UsbDirection::In),
            stalls,
            Infifo::Ep2in,
            Epirq::In2bav as u8)?;

        self.poll_in_buffer(
            2,
            EndpointAddress::from_parts(3, UsbDirection::In),
            stalls,
            Infifo::Ep3in,
            Epirq::In3bav as u8)?;

        if status == 0 {
            return Err(UsbError::WouldBlock);
        }

        //rtt::rprintln!("stat {:02x}", status);

        let result = if status & 0b10000000 != 0 {
            self.i.write(Reg::Usbirq, u8::from(Usbirq::Susp))?;

            PollResult::Suspend
        } else if status & 0b01000000 != 0 {
            self.i.write(Reg::Usbirq, u8::from(Usbirq::Ures))?;

            PollResult::Reset
        } else if status & 0b00111111 != 0 {
            // Data IRQs
            let mut ep_out = 0;
            let mut ep_in_complete = 0;

            if status & u8::from(Epirq::In0bav) != 0 {
                ep_in_complete |= 1 << 0;
                self.buffers.ep_in[0].set_state(State::Empty);
                self.status_mask &= !u8::from(Epirq::In0bav);
            }

            if status & u8::from(Epirq::In2bav) != 0 {
                ep_in_complete |= 1 << 2;
                self.buffers.ep_in[1].set_state(State::Empty);
                self.status_mask &= !u8::from(Epirq::In2bav);
            }

            if status & u8::from(Epirq::In3bav) != 0 {
                ep_in_complete |= 1 << 3;
                self.buffers.ep_in[2].set_state(State::Empty);
                self.status_mask &= !u8::from(Epirq::In3bav);
            }

            if status & (u8::from(Epirq::Out0dav) | u8::from(Epirq::Sudav)) != 0 {
                let (fifo, packet_type) = if status & u8::from(Epirq::Sudav) != 0 {
                    // SETUP always unstalls EP0
                    self.buffers.ep_out[0].set_state(State::Empty);
                    self.buffers.ep_in[0].set_state(State::Empty);

                    (Outfifo::Sud, OutPacketType::Setup)
                } else {
                    (Outfifo::Ep0out, OutPacketType::Data)
                };

                match self.buffers.ep_out[0].write_data(packet_type, |d| self.i.read_fifo(fifo, d)) {
                    Ok(()) => {
                        ep_out |= 1 << 0;
                    }
                    Err(UsbError::WouldBlock) => { }
                    Err(e) => return Err(e),
                }
            }

            if status & u8::from(Epirq::Out1dav) != 0 {
                match self.buffers.ep_out[1].write_data(OutPacketType::Data, |d| self.i.read_fifo(Outfifo::Ep1out, d)) {
                    Ok(()) => {
                        ep_out |= 1 << 1;
                    }
                    Err(UsbError::WouldBlock) => {}
                    Err(e) => return Err(e),
                }
            }

            PollResult::Data {
                ep_out,
                ep_in_complete,
            }
        } else {
            unreachable!();
        };

        Ok(result)
    }

    fn set_device_address(&mut self, _addr: u8) -> Result<()> {
        // This controller sets the address automatically.

        Ok(())
    }

    fn set_stalled(&mut self, ep_addr: EndpointAddress, stalled: bool) -> Result<()> {
        // TODO: set buffer is stalled status

        if stalled {
            self.i
                .set_bits(Reg::Epstalls, ep_stall_bits(ep_addr))?;
        } else {
            self.i
                .clear_bits(Reg::Epstalls, ep_stall_bits(ep_addr))?;
        }

        Ok(())
    }

    fn is_stalled(&mut self, ep_addr: EndpointAddress) -> Result<bool> {
        Ok((self.i.read(Reg::Epstalls)? & ep_stall_bits(ep_addr)) != 0)
    }

    fn suspend(&mut self) -> Result<()> {
        self.i.set_bits(Reg::Usbctl, 0b10010000)?; // HOSCSTEN, PWRDOWN

        Ok(())
    }

    fn resume(&mut self) -> Result<()> {
        self.i.clear_bits(Reg::Usbctl, 0b10010000)?; // HOSCSTEN, PWRDOWN

        Ok(())
    }

    const QUIRK_SET_ADDRESS_BEFORE_STATUS: bool = true;
}

fn ep_stall_bits(addr: EndpointAddress) -> u8 {
    match u8::from(addr) {
        0x80 => 0b00100001, // STLEP0IN | STLSTAT
        0x00 => 0b00100010, // STLEP0OUT | STLSTAT
        0x01 => 0b00000100, // STLEP1OUT
        0x82 => 0b00001000, // STLEP2IN
        0x83 => 0b00010000, // STLEP3IN
        _ => 0,
    }
}

#[derive(Default)]
struct EpConfig {
    iface: Option<u8>,
    used_in_alt: bool,
}

pub struct UsbEndpointAllocator<EV> {
    iface: u8,
    ep_out: [EpConfig; 2],
    ep_in: [EpConfig; 4], // index 1 isn't really there but who cares
    buffers: &'static Buffers,
    event: EV,
}

impl<EV> UsbEndpointAllocator<EV> {
    fn new(buffers: &'static Buffers, event: EV) -> Self {
        Self {
            iface: 0,
            ep_out: Default::default(),
            ep_in: Default::default(),
            buffers,
            event,
        }
    }

    fn alloc(
        &mut self,
        dir: UsbDirection,
        config: &EndpointConfig,
    ) -> Result<(u8, &'static Buffer)> {
        if config.max_packet_size() > 64 {
            return Err(UsbError::BufferOverflow);
        }

        let eps = match dir {
            UsbDirection::Out => self.ep_out.as_mut(),
            UsbDirection::In => self.ep_in.as_mut(),
        };

        let range = match config.fixed_address() {
            Some(addr) => {
                if addr.direction() != dir {
                    return Err(UsbError::EndpointUnavailable);
                }

                let i = addr.number() as usize;

                i..(i + 1)
            }
            None => (1..4),
        };

        for i in range {
            let iface = self.iface;
            let ep = &mut eps[i];

            if (dir == UsbDirection::In && i == 1) // this endpoint doesn't exist
                || ep.iface.map(|i| i != iface).unwrap_or(false)
                || ep.used_in_alt
            {
                continue;
            }

            ep.iface = Some(self.iface);
            ep.used_in_alt = true;

            let buffers = match dir {
                UsbDirection::Out => self.buffers.ep_out.as_ref(),
                UsbDirection::In => self.buffers.ep_in.as_ref(),
            };

            let buf_index = if dir == UsbDirection::In && i > 1 {
                i - 1
            } else {
                i
            };

            return Ok((i as u8, &buffers[buf_index]));
        }

        return Err(if config.fixed_address().is_some() {
            UsbError::EndpointUnavailable
        } else {
            UsbError::EndpointOverflow
        });
    }
}

impl<SPI, CS, EV> usbcore::UsbEndpointAllocator<UsbCore<SPI, CS, EV>> for UsbEndpointAllocator<EV>
where
    SPI: Transfer<u8> + Write<u8>,
    CS: OutputPin,
    EV: Event,
{
    fn alloc_out(&mut self, config: &EndpointConfig) -> Result<UsbEndpointOut<EV>> {
        self.alloc(UsbDirection::Out, config)
            .map(|(index, buffer)| UsbEndpointOut::new(index, buffer, self.event.clone()))
    }

    fn alloc_in(&mut self, config: &EndpointConfig) -> Result<UsbEndpointIn<EV>> {
        self.alloc(UsbDirection::In, config)
            .map(|(index, buffer)| UsbEndpointIn::new(index, buffer, self.event.clone()))
    }

    fn begin_interface(&mut self) -> Result<()> {
        self.iface += 1;
        usbcore::UsbEndpointAllocator::<UsbCore<SPI, CS, EV>>::next_alt_setting(self)
    }

    fn next_alt_setting(&mut self) -> Result<()> {
        for ep in self.ep_out.iter_mut() {
            ep.used_in_alt = false;
        }

        for ep in self.ep_in.iter_mut() {
            ep.used_in_alt = false;
        }

        Ok(())
    }
}

pub struct Buffers {
    ep_out: [Buffer; 2],
    ep_in: [Buffer; 3],
}

impl Buffers {
    pub const fn new() -> Self {
        Buffers {
            ep_out: [Buffer::new("OUT 0"), Buffer::new("OUT 1")],
            ep_in: [Buffer::new("IN 0"), Buffer::new("IN 2"), Buffer::new("IN 3")],
        }
    }
}

/// Used to request a poll.
pub trait Event: Clone {
    /// When called, the implementor must schedule `UsbDevice::poll` to be called in a timely
    /// fashion. If called multiple times before the poll can be executed, a single poll is enough.
    fn request(&self);
}

/// An `Event` that does nothing, for implementations that poll constantly.
#[derive(Copy, Clone)]
pub struct NullEvent;

impl Event for NullEvent {
    fn request(&self) {}
}
