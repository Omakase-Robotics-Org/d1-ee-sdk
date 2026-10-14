//! The flange channel as a [`CanFdBus`], so a hand link written against the
//! bus seam runs over the daemon's relay unchanged.

use std::{sync::Arc, time::Duration};

use d1fw_canfd::{CanFdBus, CanFdError, CanFdFrame, FrameKind};
use d1fw_ee_provider::{FlangeBus, FlangeError, Frame, TxKind};

/// How long a flush waits for the relay to take what was sent: the relay
/// takes frames at least every millisecond, so this only expires on a relay
/// that has stopped.
const FLUSH_WAIT: Duration = Duration::from_millis(100);

/// One side's flange channel ([`FlangeBus`]) seen as a bus. Clones share
/// the channel and its received-frame queue.
#[derive(Clone, Debug)]
pub struct FlangeChannel {
    bus: Arc<FlangeBus>,
}

impl FlangeChannel {
    /// The bus over `bus`.
    #[must_use]
    pub const fn new(bus: Arc<FlangeBus>) -> Self {
        Self { bus }
    }

    /// Discard every received frame not yet taken, so a link opened now
    /// starts from frames that answer it (a late answer to a probe that gave
    /// up would otherwise be taken for the first reply).
    pub fn drain(&self) {
        while let Ok(Some(_)) = self.bus.recv(Duration::ZERO) {}
    }
}

fn refusal(error: FlangeError) -> CanFdError {
    match error {
        FlangeError::Closed => CanFdError::Closed,
        FlangeError::ChannelDown(_) => CanFdError::Transport(error.to_string()),
        FlangeError::NotAdmitted(_)
        | FlangeError::Latched
        | FlangeError::Invalid(_)
        | FlangeError::Busy(_) => CanFdError::Refused(error.to_string()),
    }
}

impl CanFdBus for FlangeChannel {
    /// A frame sent without a kind is labelled motion: the relay gates it
    /// the most.
    fn send(&self, frame: &CanFdFrame) -> Result<(), CanFdError> {
        self.send_as(frame, FrameKind::Motion)
    }

    fn send_as(&self, frame: &CanFdFrame, kind: FrameKind) -> Result<(), CanFdError> {
        if frame.data.len() > d1fw_canfd::MAX_PAYLOAD {
            return Err(CanFdError::Oversize {
                length: frame.data.len(),
            });
        }
        let kind = match kind {
            FrameKind::Motion => TxKind::Motion,
            FrameKind::Read => TxKind::Read,
            FrameKind::Stop => TxKind::Stop,
        };
        self.bus
            .send(&Frame::standard(frame.id, frame.data.clone()), kind)
            .map_err(refusal)
    }

    fn flush(&self) -> Result<(), CanFdError> {
        self.bus.flush(FLUSH_WAIT).map_err(refusal)
    }

    fn recv(&self, timeout: Duration) -> Result<Option<CanFdFrame>, CanFdError> {
        loop {
            let Some(received) = self.bus.recv(timeout).map_err(refusal)? else {
                return Ok(None);
            };
            // The relay carries 11-bit identifiers only; anything else is not
            // a frame a hand protocol speaks.
            if let (false, Ok(id)) = (received.frame.extended, u16::try_from(received.frame.id)) {
                return Ok(Some(CanFdFrame::new(id, received.frame.data)));
            }
        }
    }

    fn close(&self) {}

    fn suppresses_repeat_frames(&self) -> bool {
        self.bus
            .status()
            .is_some_and(|status| status.channel.suppresses_repeats)
    }
}
