use core::cell::UnsafeCell;
use core::mem::MaybeUninit;
use core::sync::atomic::{AtomicUsize, Ordering::SeqCst};
use usb_device::{
    endpoint::{EndpointAddress, EndpointConfig, OutPacketType},
    usbcore, Result, UsbDirection, UsbError,
};

use crate::usbcore::Event;

pub struct Buffer {
    debug_name: &'static str,
    state: AtomicUsize,
    //prev_state: State,
    data: UnsafeCell<MaybeUninit<[u8; 64]>>,
}

unsafe impl Sync for Buffer {}
unsafe impl Send for Buffer {}

impl Buffer {
    pub const fn new(debug_name: &'static str) -> Self {
        Self {
            debug_name,
            state: AtomicUsize::new(0),
            //prev_state: State::Disabled,
            data: UnsafeCell::new(MaybeUninit::new([0xba; 64])),
        }
    }
}

#[derive(Copy, Clone, Eq, PartialEq, Debug)]
pub enum State {
    Disabled,
    Empty,
    Stall,
    Setup,
    Valid(u8),
}

impl Buffer {
    const DISABLED: usize = 0;
    const EMPTY: usize = 1;
    const STALL: usize = 2;
    const SETUP: usize = 3;
    const VALID: usize = 0xffffff80; // the lower bits contain packet length (0-64)

    pub fn state(&self) -> State {
        match self.state.load(SeqCst) {
            Self::DISABLED => State::Disabled,
            Self::EMPTY => State::Empty,
            Self::STALL => State::Stall,
            Self::SETUP => State::Setup,
            other => State::Valid((other & !Self::VALID) as u8),
        }
    }

    // TODO: Find the hazards in this
    pub fn set_state(&self, state: State) {
        if state == State::Stall {
            rtt::rprintln!("{} STALLED", self.debug_name);
        }

        let state = match state {
            State::Disabled => Self::DISABLED,
            State::Empty => Self::EMPTY,
            State::Stall => Self::STALL,
            State::Setup => Self::SETUP,
            State::Valid(count) => usize::from(count) | Self::VALID,
        };

        self.state.store(state, SeqCst);
    }

    fn set_stalled(&self, ev: &mut impl Event, stalled: bool) {
        if stalled && self.state() != State::Stall {
            self.set_state(State::Stall);
            ev.request();
        } else if !stalled && self.state() == State::Stall {
            self.set_state(State::Empty);
            ev.request();
        }
    }

    pub fn write_data(
        &self,
        packet_type: OutPacketType,
        f: impl FnOnce(&mut [u8]) -> Result<usize>,
    ) -> Result<()> {
        //rtt::rprintln!("write {} {:?} write state: {:?}", self.debug_name, packet_type, self.state());

        match self.state() {
            State::Empty => {}
            State::Stall if packet_type == OutPacketType::Setup => {},
            _ => return Err(UsbError::WouldBlock),
        };

        let data = unsafe { &mut *(*self.data.get()).as_mut_ptr() };

        let count = f(data)?;

        //rtt::rprintln!("  count: {} data: {:02x?}", count, &data[..count]);

        if count > 64 {
            return Err(UsbError::BufferOverflow);
        }

        self.set_state(match packet_type {
            OutPacketType::Data => State::Valid(count as u8),
            OutPacketType::Setup => State::Setup
        });

        Ok(())
    }

    pub fn read_data(&self, empty_on_error: bool, f: impl FnOnce(&[u8]) -> Result<()>) -> Result<(usize, OutPacketType)> {
        let (count, packet_type) = match self.state() {
            State::Setup => (8, OutPacketType::Setup),
            State::Valid(count) => (usize::from(count), OutPacketType::Data),
            _ => return Err(UsbError::WouldBlock),
        };

        let data = unsafe { &*(*self.data.get()).as_ptr() };

        //rtt::rprintln!("read {} data: {:02x?}", self.debug_name, &data[..count]);

        let res = f(&data[..count]);

        //rtt::rprintln!("  res: {:?}", res);

        if res.is_ok() || empty_on_error {
            self.set_state(State::Empty);
        }

        res?;

        Ok((count, packet_type))
    }
}

pub struct UsbEndpointOut<EV> {
    index: u8,
    buffer: &'static Buffer,
    event: EV,
}

impl<EV> UsbEndpointOut<EV>
where
    EV: Event,
{
    pub(crate) fn new(index: u8, buffer: &'static Buffer, event: EV) -> Self {
        Self {
            index,
            buffer,
            event,
        }
    }
}

impl<EV> usbcore::UsbEndpoint for UsbEndpointOut<EV>
where
    EV: Event,
{
    fn address(&self) -> EndpointAddress {
        EndpointAddress::from_parts(self.index, UsbDirection::Out)
    }

    unsafe fn enable(&mut self, _config: &EndpointConfig) -> Result<()> {
        self.buffer.set_state(State::Empty);
        self.event.request();

        Ok(())
    }

    fn disable(&mut self) -> Result<()> {
        self.buffer.set_state(State::Disabled);
        self.event.request();

        Ok(())
    }

    fn is_stalled(&mut self) -> Result<bool> {
        Ok(self.buffer.state() == State::Stall)
    }

    fn set_stalled(&mut self, stalled: bool) -> Result<()> {
        self.buffer.set_stalled(&mut self.event, stalled);

        Ok(())
    }
}

impl<EV> usbcore::UsbEndpointOut for UsbEndpointOut<EV>
where
    EV: Event,
{
    fn read_packet(&mut self, data: &mut [u8]) -> Result<(usize, OutPacketType)> {
        self.buffer.read_data(true, |d| {
            data[..d.len()].copy_from_slice(d);
            Ok(())
        })
    }
}

pub struct UsbEndpointIn<EV> {
    index: u8,
    buffer: &'static Buffer,
    event: EV,
}

impl<EV> UsbEndpointIn<EV>
where
    EV: Event,
{
    pub(crate) fn new(index: u8, buffer: &'static Buffer, event: EV) -> Self {
        Self {
            index,
            buffer,
            event,
        }
    }
}

impl<EV> usbcore::UsbEndpoint for UsbEndpointIn<EV>
where
    EV: Event,
{
    fn address(&self) -> EndpointAddress {
        EndpointAddress::from_parts(self.index, UsbDirection::In)
    }

    unsafe fn enable(&mut self, _config: &EndpointConfig) -> Result<()> {
        self.buffer.set_state(State::Empty);
        self.event.request();

        Ok(())
    }

    fn disable(&mut self) -> Result<()> {
        self.buffer.set_state(State::Disabled);
        self.event.request();

        Ok(())
    }

    fn is_stalled(&mut self) -> Result<bool> {
        Ok(self.buffer.state() == State::Stall)
    }

    fn set_stalled(&mut self, stalled: bool) -> Result<()> {
        self.buffer.set_stalled(&mut self.event, stalled);
        self.event.request();

        Ok(())
    }
}

impl<EV> usbcore::UsbEndpointIn for UsbEndpointIn<EV>
where
    EV: Event,
{
    fn write_packet(&mut self, data: &[u8]) -> Result<()> {
        self.buffer.write_data(OutPacketType::Data, |d| {
            d[..data.len()].copy_from_slice(data);

            Ok(data.len())
        })?;

        self.event.request();

        Ok(())
    }

    fn flush(&mut self) -> Result<()> {
        // TODO: Implement
        Err(UsbError::WouldBlock)
    }
}
