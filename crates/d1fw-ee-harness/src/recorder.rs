//! A channel that records every frame sent through it.

use std::{
    sync::{Arc, Mutex},
    time::{Duration, Instant},
};

use d1fw_canfd::{CanFdBus, CanFdError, CanFdFrame, FrameKind};

/// One recorded frame: when it was sent, the frame, and the kind it was
/// sent as.
pub type Sent = (Instant, CanFdFrame, FrameKind);

/// A bus that records every frame sent through it, with when it was sent
/// and the kind it was sent as. Clones share the bus and the record.
pub struct Recorder<B> {
    inner: Arc<B>,
    sent: Arc<Mutex<Vec<Sent>>>,
}

impl<B> Clone for Recorder<B> {
    fn clone(&self) -> Self {
        Self {
            inner: Arc::clone(&self.inner),
            sent: Arc::clone(&self.sent),
        }
    }
}

impl<B: CanFdBus> Recorder<B> {
    /// Record what is sent on `inner`.
    pub fn new(inner: B) -> Self {
        Self {
            inner: Arc::new(inner),
            sent: Arc::default(),
        }
    }

    /// The recorded bus.
    #[must_use]
    pub fn inner(&self) -> &B {
        &self.inner
    }

    /// Every frame recorded since the last take.
    ///
    /// # Panics
    ///
    /// On a poisoned lock (a test thread panicked while recording).
    pub fn take(&self) -> Vec<Sent> {
        std::mem::take(&mut *self.sent.lock().expect("recorder lock"))
    }

    /// The kinds of every frame recorded since the last take.
    pub fn take_kinds(&self) -> Vec<FrameKind> {
        self.take().into_iter().map(|(_, _, kind)| kind).collect()
    }
}

impl<B: CanFdBus> CanFdBus for Recorder<B> {
    fn send(&self, frame: &CanFdFrame) -> Result<(), CanFdError> {
        self.send_as(frame, FrameKind::Motion)
    }

    fn send_as(&self, frame: &CanFdFrame, kind: FrameKind) -> Result<(), CanFdError> {
        self.sent
            .lock()
            .expect("recorder lock")
            .push((Instant::now(), frame.clone(), kind));
        self.inner.send_as(frame, kind)
    }

    fn recv(&self, timeout: Duration) -> Result<Option<CanFdFrame>, CanFdError> {
        self.inner.recv(timeout)
    }

    fn close(&self) {
        self.inner.close();
    }

    fn suppresses_repeat_frames(&self) -> bool {
        self.inner.suppresses_repeat_frames()
    }
}
