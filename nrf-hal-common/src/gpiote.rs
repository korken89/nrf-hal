#[cfg(not(any(feature = "9160", feature = "5340-net")))]
use {
    crate::pac::gpiote::{EVENTS_IN, TASKS_OUT},
    crate::pac::GPIOTE,
};

#[cfg(feature = "9160")]
use {
    crate::pac::gpiote0_s::{EVENTS_IN, TASKS_CLR, TASKS_OUT, TASKS_SET},
    crate::pac::GPIOTE1_NS as GPIOTE,
};

#[cfg(feature = "5340-net")]
use {
    crate::pac::gpiote_ns::{EVENTS_IN, TASKS_CLR, TASKS_OUT, TASKS_SET},
    crate::pac::GPIOTE_NS as GPIOTE,
};

#[cfg(not(any(feature = "51", feature = "9160", feature = "5340-net")))]
use crate::pac::gpiote::{TASKS_CLR, TASKS_SET};

#[cfg(not(feature = "51"))]
const NUM_CHANNELS: usize = 8;
#[cfg(feature = "51")]
const NUM_CHANNELS: usize = 4;

use crate::{
    gpio::{Floating, Input, Pin, Port, PullDown, PullUp},
    waker_registration::CriticalSectionWakerRegistration,
    InterruptToken,
};
use core::task::Poll;
use cortex_m::peripheral::NVIC;
use embedded_hal::digital::InputPin;

// TODO: Figure out how to do this
#[macro_export]
macro_rules! register_gpiote_interrupt {
    ($token_name:ident) => {
        pub struct $token_name;

        unsafe impl async_nrf52832_hal::InterruptToken<async_nrf52832_hal::gpiote::export::GPIOTE>
            for $token_name
        {
        }

        #[no_mangle]
        #[allow(non_snake_case)]
        unsafe extern "C" fn GPIOTE() {
            async_nrf52832_hal::gpiote::export::on_interrupt_gpiote();
        }
    };
}

// Hidden export only for use by the macro
#[doc(hidden)]
pub mod export {
    pub use crate::pac::GPIOTE;

    /// This happens on interrupt.
    pub fn on_interrupt_gpiote() {
        let gpiote = unsafe { &*GPIOTE::ptr() };

        for ch in 0..8 {
            if gpiote.events_in[ch].read().bits() != 0 {
                gpiote.intenclr.write(|w| unsafe { w.bits(1 << ch) });
                super::WAKER_REGISTRATION[ch].wake();
            }
        }
    }
}

const NEW_WAKER_REG: CriticalSectionWakerRegistration = CriticalSectionWakerRegistration::new();

// Waker registration for each GPIOTE channel.
static WAKER_REGISTRATION: [CriticalSectionWakerRegistration; NUM_CHANNELS] =
    [NEW_WAKER_REG; NUM_CHANNELS];

/// Uninit state for a gpiote channel
pub struct Uninit;

/// Configured state for a gpiote channel with a pin
pub struct Configured<Pin>(Pin);

/// Convert the Gpiote to its channels and set the priority of the GPIOTE interrupt.
#[inline(always)]
pub fn new_with_priority(
    gpiote: GPIOTE,
    interrupt_token: impl InterruptToken<GPIOTE>,
    nvic: &mut NVIC,
    prio: u8,
) -> (
    Channel<0, Uninit>,
    Channel<1, Uninit>,
    Channel<2, Uninit>,
    Channel<3, Uninit>,
    Channel<4, Uninit>,
    Channel<5, Uninit>,
    Channel<6, Uninit>,
    Channel<7, Uninit>,
) {
    unsafe { nvic.set_priority(crate::pac::Interrupt::GPIOTE, prio) };

    new(gpiote, interrupt_token)
}

/// Return from new
#[cfg(not(feature = "51"))]
pub type GpioteNewReturn = (
    Channel<0, Uninit>,
    Channel<1, Uninit>,
    Channel<2, Uninit>,
    Channel<3, Uninit>,
    Channel<4, Uninit>,
    Channel<5, Uninit>,
    Channel<6, Uninit>,
    Channel<7, Uninit>,
);
#[cfg(feature = "51")]
pub type GpioteNewReturn = (
    Channel<0, Uninit>,
    Channel<1, Uninit>,
    Channel<2, Uninit>,
    Channel<3, Uninit>,
);

#[cfg(not(feature = "51"))]
const GPIOTE_NEW: GpioteNewReturn = (
    Channel { conf: Uninit },
    Channel { conf: Uninit },
    Channel { conf: Uninit },
    Channel { conf: Uninit },
    Channel { conf: Uninit },
    Channel { conf: Uninit },
    Channel { conf: Uninit },
    Channel { conf: Uninit },
);
#[cfg(feature = "51")]
const GPIOTE_NEW: GpioteNewReturn = (
    Channel { conf: Uninit },
    Channel { conf: Uninit },
    Channel { conf: Uninit },
    Channel { conf: Uninit },
);

/// Convert the Gpiote to its channels
pub fn new(gpiote: GPIOTE, _interrupt_token: impl InterruptToken<GPIOTE>) -> GpioteNewReturn {
    // Clear any old pending interrupts
    gpiote.intenclr.write(|w| unsafe { w.bits(0xff) });
    for ch in 0..NUM_CHANNELS {
        gpiote.events_in[ch].write(|w| w);
    }

    unsafe { NVIC::unmask(crate::pac::Interrupt::GPIOTE) };

    GPIOTE_NEW
}

/// A Gpiote channel
#[derive(Clone)]
pub struct Channel<const CH: usize, T> {
    conf: T,
}

impl<const CH: usize> Channel<CH, Uninit> {
    /// Configure a channel
    pub fn configure<Pin>(self, input_pin: Pin) -> Channel<CH, Configured<Pin>>
    where
        Pin: GpioteInputPin,
    {
        let gpiote = unsafe { &*GPIOTE::ptr() };

        gpiote.config[CH].write(|w| unsafe { w.psel().bits(input_pin.pin()) });

        // TODO: Is port event needed?
        // gpiote.intenset.write(|w| w.port().set());

        Channel {
            conf: Configured(input_pin),
        }
    }
}

impl<const CH: usize, Pin> Channel<CH, Configured<Pin>>
where
    Pin: InputPin,
{
    /// Enable interrupts from this channel.
    fn enable_interrupt() {
        // SAFETY: Atomic write.
        unsafe {
            let gpiote = &*GPIOTE::ptr();
            gpiote.intenset.write(|w| w.bits(1 << CH));
        }
    }

    /// Disable interrupts from this channel.
    fn disable_interrupt() {
        // SAFETY: Atomic write.
        unsafe {
            let gpiote = &*GPIOTE::ptr();
            gpiote.intenclr.write(|w| w.bits(1 << CH));
        }
    }

    /// Checks if the event is triggered on this channel.
    fn is_event_triggered() -> bool {
        // SAFETY: Atomic read.
        unsafe {
            let gpiote = &*GPIOTE::ptr();
            gpiote.events_in[CH].read().bits() != 0
        }
    }

    /// Resets the event on this channel.
    fn reset_events() {
        // SAFETY: Atomic write.
        unsafe {
            let gpiote = &*GPIOTE::ptr();
            gpiote.events_in[CH].write(|w| w);
        }
    }

    fn set_trigger(trigger_mode: EventPolarity) {
        let gpiote = unsafe { &*GPIOTE::ptr() };
        gpiote.config[CH].modify(|_, w| match trigger_mode {
            EventPolarity::HiToLo => w.mode().event().polarity().hi_to_lo(),
            EventPolarity::LoToHi => w.mode().event().polarity().lo_to_hi(),
            EventPolarity::None => w.mode().event().polarity().none(),
            EventPolarity::Toggle => w.mode().event().polarity().toggle(),
        });
    }

    /// Returns reference to channel event endpoint for PPI.
    pub fn event(&self) -> &EVENTS_IN {
        let gpiote = unsafe { &*GPIOTE::ptr() };
        &gpiote.events_in[CH]
    }

    /// Returns reference to task_out endpoint for PPI.
    pub fn task_out(&self) -> &TASKS_OUT {
        let gpiote = unsafe { &*GPIOTE::ptr() };
        &gpiote.tasks_out[CH]
    }

    /// Returns reference to task_clr endpoint for PPI.
    #[cfg(not(feature = "51"))]
    pub fn task_clr(&self) -> &TASKS_CLR {
        let gpiote = unsafe { &*GPIOTE::ptr() };
        &gpiote.tasks_clr[CH]
    }

    /// Returns reference to task_set endpoint for PPI.
    #[cfg(not(feature = "51"))]
    pub fn task_set(&self) -> &TASKS_SET {
        let gpiote = unsafe { &*GPIOTE::ptr() };
        &gpiote.tasks_set[CH]
    }

    /// Main API for waiting on a pin.
    async fn wait_for(&mut self, waiting_for: WaitingFor) {
        let dropper = crate::OnDrop::new(|| {
            Self::disable_interrupt();
            Self::reset_events();
        });

        Self::reset_events();

        core::future::poll_fn(|cx| {
            Self::disable_interrupt();

            match waiting_for {
                WaitingFor::RisingEdge | WaitingFor::FallingEdge | WaitingFor::AnyEdge => {
                    if Self::is_event_triggered() {
                        return Poll::Ready(());
                    }
                }
                _ => {}
            }

            Self::reset_events();

            match waiting_for {
                WaitingFor::High => {
                    // Check fall through
                    if matches!(self.conf.0.is_high(), Ok(true)) {
                        return Poll::Ready(());
                    }

                    Self::set_trigger(EventPolarity::LoToHi);
                }
                WaitingFor::Low => {
                    // Check fall through
                    if matches!(self.conf.0.is_low(), Ok(true)) {
                        return Poll::Ready(());
                    }

                    Self::set_trigger(EventPolarity::HiToLo);
                }
                WaitingFor::RisingEdge => {
                    Self::set_trigger(EventPolarity::LoToHi);
                }
                WaitingFor::FallingEdge => {
                    Self::set_trigger(EventPolarity::HiToLo);
                }
                WaitingFor::AnyEdge => {
                    Self::set_trigger(EventPolarity::Toggle);
                }
            }

            WAKER_REGISTRATION[CH].register(cx.waker());

            // Enable interrupts after registration
            Self::enable_interrupt();

            Poll::Pending
        })
        .await;

        dropper.defuse();
    }

    /// Free the channel an pin.
    pub fn free(self) -> (Pin, Channel<CH, Uninit>) {
        Self::disable_interrupt();
        Self::reset_events();

        let gpiote = unsafe { &*GPIOTE::ptr() };
        gpiote.config[CH].write(|w| w);

        (self.conf.0, Channel { conf: Uninit })
    }
}

// A channel's pin can be read like any input pin.
impl<const CH: usize, Pin> InputPin for Channel<CH, Configured<Pin>>
where
    Pin: InputPin,
{
    fn is_low(&self) -> Result<bool, Self::Error> {
        self.conf.0.is_low()
    }

    fn is_high(&self) -> Result<bool, Self::Error> {
        self.conf.0.is_high()
    }
}

// Error type for the async traits.
impl<const CH: usize, Pin> embedded_hal::digital::ErrorType for Channel<CH, Configured<Pin>>
where
    Pin: InputPin,
{
    type Error = <Pin as embedded_hal::digital::ErrorType>::Error;
}

impl<const CH: usize, Pin> embedded_hal_async::digital::Wait for Channel<CH, Configured<Pin>>
where
    Pin: GpioteInputPin + InputPin,
{
    #[inline(always)]
    async fn wait_for_high(&mut self) -> Result<(), Self::Error> {
        self.wait_for(WaitingFor::High).await;
        Ok(())
    }

    #[inline(always)]
    async fn wait_for_low(&mut self) -> Result<(), Self::Error> {
        self.wait_for(WaitingFor::Low).await;
        Ok(())
    }

    #[inline(always)]
    async fn wait_for_rising_edge(&mut self) -> Result<(), Self::Error> {
        self.wait_for(WaitingFor::RisingEdge).await;
        Ok(())
    }

    #[inline(always)]
    async fn wait_for_falling_edge(&mut self) -> Result<(), Self::Error> {
        self.wait_for(WaitingFor::FallingEdge).await;
        Ok(())
    }

    #[inline(always)]
    async fn wait_for_any_edge(&mut self) -> Result<(), Self::Error> {
        self.wait_for(WaitingFor::AnyEdge).await;
        Ok(())
    }
}

#[derive(Debug, Copy, Clone, PartialEq, Eq, Hash)]
pub enum WaitingFor {
    High,
    Low,
    RisingEdge,
    FallingEdge,
    AnyEdge,
}

pub enum EventPolarity {
    None,
    HiToLo,
    LoToHi,
    Toggle,
}

/// Trait to represent event input pin.
pub trait GpioteInputPin {
    fn pin(&self) -> u8;
    fn port(&self) -> Port;
}

impl GpioteInputPin for Pin<Input<PullUp>> {
    fn pin(&self) -> u8 {
        self.pin()
    }
    fn port(&self) -> Port {
        self.port()
    }
}

impl GpioteInputPin for Pin<Input<PullDown>> {
    fn pin(&self) -> u8 {
        self.pin()
    }
    fn port(&self) -> Port {
        self.port()
    }
}

impl GpioteInputPin for Pin<Input<Floating>> {
    fn pin(&self) -> u8 {
        self.pin()
    }
    fn port(&self) -> Port {
        self.port()
    }
}
