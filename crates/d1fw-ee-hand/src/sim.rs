//! A simulated hand on the bus seam, and the arm end module's repeat-frame
//! suppression as a bus that wraps another.
//!
//! [`SimBus`] presents a device model — anything that answers a frame with
//! frames, such as the protocol crates' `SimulatedHand` — as a [`CanFdBus`]:
//! the SAME link code that drives a hand over the daemon's flange relay
//! drives it here, with every frame built and parsed and nothing on a wire.
//! Replies are queued the instant a request is sent, so a hand transaction
//! never depends on a timer. It does not model the end module's repeat
//! suppression; [`SuppressRepeats`] does, around any bus.

use std::{
    collections::VecDeque,
    sync::{Arc, Mutex, MutexGuard, PoisonError},
    time::Duration,
};

use d1fw_canfd::{CanFdBus, CanFdError, CanFdFrame};

type Answer = Box<dyn FnMut(&CanFdFrame) -> Vec<CanFdFrame> + Send>;

struct Inner {
    answer: Answer,
    inbox: VecDeque<CanFdFrame>,
    sent: Vec<CanFdFrame>,
    closed: bool,
}

/// A simulated device presented as a CAN FD bus. Clones share the device.
#[derive(Clone)]
pub struct SimBus {
    inner: Arc<Mutex<Inner>>,
}

impl std::fmt::Debug for SimBus {
    fn fmt(&self, formatter: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        formatter.debug_struct("SimBus").finish_non_exhaustive()
    }
}

impl SimBus {
    /// A bus whose device answers every frame sent (padded to a legal CAN FD
    /// length, as a real transport sends it) with what `answer` returns.
    #[must_use]
    pub fn new(answer: impl FnMut(&CanFdFrame) -> Vec<CanFdFrame> + Send + 'static) -> Self {
        Self {
            inner: Arc::new(Mutex::new(Inner {
                answer: Box::new(answer),
                inbox: VecDeque::new(),
                sent: Vec::new(),
                closed: false,
            })),
        }
    }

    /// Every frame put on this bus, padded, for assertions.
    #[must_use]
    pub fn sent(&self) -> Vec<CanFdFrame> {
        lock(&self.inner).sent.clone()
    }
}

impl CanFdBus for SimBus {
    fn send(&self, frame: &CanFdFrame) -> Result<(), CanFdError> {
        let padded = frame.padded()?;
        let mut inner = lock(&self.inner);
        if inner.closed {
            return Err(CanFdError::Closed);
        }
        inner.sent.push(padded.clone());
        let replies = (inner.answer)(&padded);
        inner.inbox.extend(replies);
        Ok(())
    }

    fn recv(&self, _timeout: Duration) -> Result<Option<CanFdFrame>, CanFdError> {
        let mut inner = lock(&self.inner);
        if inner.closed {
            return Err(CanFdError::Closed);
        }
        // No wait on an empty inbox: a simulated device has already answered
        // by the time the request returns.
        Ok(inner.inbox.pop_front())
    }

    fn close(&self) {
        lock(&self.inner).closed = true;
    }
}

/// Wraps a bus so that a received frame identical to the last one handed
/// over is swallowed, the way the D1 arm's end module swallows it (measured
/// on arm A, 2026-08-13): an idle hand asked the same question twice answers
/// identically, and the second answer is never delivered.
///
/// Only the receive direction is filtered: the relay passes every request
/// through, and it is the answers that collide.
#[derive(Clone, Debug)]
pub struct SuppressRepeats<B> {
    inner: B,
    /// The payload most recently handed to the caller: the relay compares
    /// against what it last delivered, so a frame it swallowed does not
    /// become what the next one is compared against.
    last_delivered: Arc<Mutex<Option<Vec<u8>>>>,
    /// How many frames were swallowed, so a test can tell "the driver coped"
    /// from "the suppression never fired".
    swallowed: Arc<Mutex<usize>>,
}

impl<B: CanFdBus> SuppressRepeats<B> {
    /// Put `inner` behind a relay that suppresses repeated frames.
    #[must_use]
    pub fn new(inner: B) -> Self {
        Self {
            inner,
            last_delivered: Arc::new(Mutex::new(None)),
            swallowed: Arc::new(Mutex::new(0)),
        }
    }

    /// How many frames the relay has swallowed so far.
    #[must_use]
    pub fn swallowed(&self) -> usize {
        *lock(&self.swallowed)
    }
}

impl<B: CanFdBus> CanFdBus for SuppressRepeats<B> {
    fn send(&self, frame: &CanFdFrame) -> Result<(), CanFdError> {
        self.inner.send(frame)
    }

    fn recv(&self, timeout: Duration) -> Result<Option<CanFdFrame>, CanFdError> {
        loop {
            let Some(frame) = self.inner.recv(timeout)? else {
                return Ok(None);
            };
            let mut last = lock(&self.last_delivered);
            if last.as_deref() == Some(frame.data.as_slice()) {
                // Swallowed without a word; the caller's next answer, if one
                // is queued, is what it would actually receive.
                *lock(&self.swallowed) += 1;
                continue;
            }
            *last = Some(frame.data.clone());
            return Ok(Some(frame));
        }
    }

    fn close(&self) {
        self.inner.close();
    }

    fn suppresses_repeat_frames(&self) -> bool {
        true
    }
}

fn lock<T>(mutex: &Mutex<T>) -> MutexGuard<'_, T> {
    mutex.lock().unwrap_or_else(PoisonError::into_inner)
}

#[cfg(test)]
mod tests {
    use super::*;

    /// A device that answers every frame with its first byte, alone.
    fn echo() -> SimBus {
        SimBus::new(|frame| vec![CanFdFrame::new(frame.id | 0x400, vec![frame.data[0]])])
    }

    #[test]
    fn a_request_is_answered_before_the_send_returns_and_sent_padded() {
        let bus = echo();
        bus.send(&CanFdFrame::new(1, vec![7; 28])).unwrap();
        let answer = bus.recv(Duration::ZERO).unwrap().unwrap();
        assert_eq!((answer.id, answer.data), (0x401, vec![7]));
        assert_eq!(bus.sent()[0].data.len(), 32);
        bus.close();
        assert_eq!(
            bus.send(&CanFdFrame::new(1, vec![1])),
            Err(CanFdError::Closed)
        );
    }

    #[test]
    fn an_identical_answer_is_swallowed_and_a_different_one_is_not() {
        let device = echo();
        let bus = SuppressRepeats::new(device.clone());
        assert!(bus.suppresses_repeat_frames());
        assert!(!device.suppresses_repeat_frames());

        bus.send(&CanFdFrame::new(1, vec![5])).unwrap();
        assert!(bus.recv(Duration::ZERO).unwrap().is_some());
        bus.send(&CanFdFrame::new(1, vec![5])).unwrap();
        assert_eq!(bus.recv(Duration::ZERO).unwrap(), None);
        assert_eq!(bus.swallowed(), 1);
        bus.send(&CanFdFrame::new(1, vec![6])).unwrap();
        assert_eq!(bus.recv(Duration::ZERO).unwrap().unwrap().data, vec![6]);
        // Requests are never filtered.
        assert_eq!(device.sent().len(), 3);
    }
}
