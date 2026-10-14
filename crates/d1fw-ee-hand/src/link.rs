//! What the hand drivers on the arm's end-module channel share: the per-side
//! link serialization, the bus-yield priority, the stale-sample cache and the
//! startup probe band.
//!
//! Nothing here touches a transport or names a model: a device crate owns
//! its wire and passes its link in.
//!
//! # Bus-yield priority
//!
//! Transactions on one hand are serialized, because a reply carries no
//! sequence number — the O30 matches only on the echoed register index, and
//! the DH116S on the frame type — so two requests in flight could take each
//! other's answers. Serialization alone would put a READ in front of a
//! motion command, and a read holds the bus for its whole reply wait: on a
//! hand that has gone quiet a single command would then wait out the full
//! timeout, which a 50 Hz command stream cannot absorb.
//!
//! So the priority is inverted, exactly as the Python driver does it: a read
//! in flight polls a waiting-command counter ([`CommandsWaiting`]) every
//! [`COMMAND_PRIORITY_SLICE`] and gives the bus up the moment a command
//! appears. The read reports [`ReadError::Yielded`], which is not a fault —
//! the sample is simply dropped in favour of the command, and
//! `HandState::live` says so. [`SerializedLink`] is that discipline over any
//! link type; the driver hands it closures, so no link trait is needed.

use std::{
    sync::{
        Arc, Mutex,
        atomic::{AtomicUsize, Ordering},
    },
    time::Duration,
};

use d1fw_ee_provider::{ErrorKind, ProviderError};
use tokio::sync::Mutex as AsyncMutex;

/// How long one reply is awaited.
///
/// The Python O30 driver's own `REQUEST_TIMEOUT` is 0.3 s, and the gripper
/// backend measured 250 ms as what an end-module round trip actually needs
/// (below that every query timed out and every stroke reported blind on d1-2,
/// 2026-09-08). 300 ms covers both.
pub const REPLY_TIMEOUT: Duration = Duration::from_millis(300);

/// How long a read may sit on the bus before it notices a command waiting.
///
/// A command's worst-case wait is one slice plus the lock handoff, so this is
/// the number that has to fit inside a control period: 5 ms into the 20 ms of
/// a 50 Hz stream.
pub const COMMAND_PRIORITY_SLICE: Duration = Duration::from_millis(5);

/// The largest hand id the startup probe tries.
///
/// Both hand protocols address far more ids than any D1 uses (the O30 up to
/// `0x03FE`, the DH116S up to 127). A probe costs one reply timeout per id
/// that does not answer, so the band is bounded where an unanswered scan
/// still fits a startup: fifteen ids, a few seconds. A hand flashed to an id
/// above it is reported as absent, with this bound named in the log.
pub const PROBE_ID_MAX: u16 = 0x0f;

/// The order a startup probe tries ids in: `first` (the model's factory id
/// for the side), then the rest of `[1, PROBE_ID_MAX]` ascending.
#[must_use]
pub fn probe_order(first: u16) -> Vec<u16> {
    std::iter::once(first)
        .chain((1..=PROBE_ID_MAX).filter(|id| *id != first))
        .collect()
}

/// The first id in `order` that `answers`, tried one at a time on a blocking
/// thread; `None` when none did. `answers` performs one read that commands
/// nothing.
pub async fn detect_node<F>(order: Vec<u16>, answers: F) -> Option<u16>
where
    F: Fn(u16) -> bool + Send + 'static,
{
    tokio::task::spawn_blocking(move || order.into_iter().find(|candidate| answers(*candidate)))
        .await
        .ok()
        .flatten()
}

/// Why a read did not produce a value.
#[derive(Debug)]
pub enum ReadError {
    /// A motion command was waiting, so the read gave the bus up. Not a
    /// fault: the sample was dropped on purpose.
    Yielded,
    /// No reply arrived inside the deadline.
    Timeout(String),
    /// The device answered with a refusal.
    Device(String),
    /// The transport failed.
    Transport(String),
}

impl ReadError {
    /// A transport failure, from the transport's own error.
    pub fn transport(error: impl std::fmt::Display) -> Self {
        Self::Transport(error.to_string())
    }
}

impl From<ReadError> for ProviderError {
    fn from(error: ReadError) -> Self {
        match error {
            ReadError::Yielded => Self::new(
                ErrorKind::Refused,
                "the hand read gave the bus up to a waiting motion command",
            ),
            ReadError::Timeout(detail) => Self::new(ErrorKind::Timeout, detail),
            ReadError::Device(detail) | ReadError::Transport(detail) => {
                Self::new(ErrorKind::Device, detail)
            }
        }
    }
}

/// The count of motion commands waiting for one side's link, which a read in
/// flight polls so it can give the bus up.
#[derive(Debug, Clone, Default)]
pub struct CommandsWaiting(Arc<AtomicUsize>);

impl CommandsWaiting {
    /// Whether a motion command is waiting for the link.
    #[must_use]
    pub fn any(&self) -> bool {
        self.0.load(Ordering::Acquire) > 0
    }
}

/// Announces a motion command waiting for the bus, so a read in flight exits.
///
/// Constructed BEFORE the bus lock is asked for, which is the whole point:
/// the reader has to learn about this command while it is still waiting, not
/// after it has given up on its own.
struct CommandPriority(CommandsWaiting);

impl CommandPriority {
    fn new(waiting: &CommandsWaiting) -> Self {
        waiting.0.fetch_add(1, Ordering::AcqRel);
        Self(waiting.clone())
    }
}

impl Drop for CommandPriority {
    fn drop(&mut self) {
        self.0.0.fetch_sub(1, Ordering::AcqRel);
    }
}

/// One side's link, used by one transaction at a time, with motion commands
/// taking the bus ahead of reads. Every transaction blocks on the bus, so it
/// runs on a blocking thread.
pub struct SerializedLink<L> {
    link: Arc<AsyncMutex<L>>,
    commands_waiting: CommandsWaiting,
}

impl<L: Send + 'static> SerializedLink<L> {
    /// The link `open` builds; it receives the counter its reads poll.
    pub fn new(open: impl FnOnce(CommandsWaiting) -> L) -> Self {
        let commands_waiting = CommandsWaiting::default();
        Self {
            link: Arc::new(AsyncMutex::new(open(commands_waiting.clone()))),
            commands_waiting,
        }
    }

    /// Run one motion command, announcing it so a read in flight steps aside.
    ///
    /// # Errors
    ///
    /// The command's own error, or [`ErrorKind::Device`] when its task
    /// failed.
    pub async fn command<F>(&self, action: F) -> Result<(), ProviderError>
    where
        F: FnOnce(&mut L) -> Result<(), ProviderError> + Send + 'static,
    {
        // The announcement lasts exactly as long as the WAIT for the bus, and
        // not a moment longer. Holding it across the command itself would
        // starve the command's own verification reads — an O30 enable writes
        // the masks and then reads them back with RTS=1, and with the flag
        // still up that read would yield to the very command that raised it,
        // failing every retry on a hand that is working perfectly.
        let guard = {
            let _priority = CommandPriority::new(&self.commands_waiting);
            Arc::clone(&self.link).lock_owned().await
        };
        tokio::task::spawn_blocking(move || {
            let mut guard = guard;
            action(&mut guard)
        })
        .await
        .map_err(|error| {
            ProviderError::new(
                ErrorKind::Device,
                format!("hand command task failed: {error}"),
            )
        })?
    }

    /// Run one read, unless a command or another read owns the link: a busy
    /// link is NOT interrupted for a read, and `Ok(None)` says it was busy.
    ///
    /// # Errors
    ///
    /// [`ErrorKind::Device`] when the read's task failed.
    pub async fn try_read<T, F>(
        &self,
        read: F,
    ) -> Result<Option<Result<T, ReadError>>, ProviderError>
    where
        T: Send + 'static,
        F: FnOnce(&mut L) -> Result<T, ReadError> + Send + 'static,
    {
        let Ok(guard) = Arc::clone(&self.link).try_lock_owned() else {
            return Ok(None);
        };
        tokio::task::spawn_blocking(move || {
            let mut guard = guard;
            read(&mut guard)
        })
        .await
        .map(Some)
        .map_err(|error| {
            ProviderError::new(ErrorKind::Device, format!("hand read task failed: {error}"))
        })
    }
}

/// The last value a LIVE read produced, answered when the link is busy or a
/// read yields.
#[derive(Debug, Default)]
pub struct StaleCache<T>(Mutex<Option<T>>);

impl<T: Clone> StaleCache<T> {
    /// Remember `value` as the newest live one.
    pub fn remember(&self, value: T) {
        *self.lock() = Some(value);
    }

    /// The newest live value, if any.
    #[must_use]
    pub fn last(&self) -> Option<T> {
        self.lock().clone()
    }

    fn lock(&self) -> std::sync::MutexGuard<'_, Option<T>> {
        self.0
            .lock()
            .unwrap_or_else(std::sync::PoisonError::into_inner)
    }
}

/// Refuse a command that is not shaped like the hand before anything is put
/// on the wire: `num_axes` values, each a fraction in `[0, 1]`.
///
/// The SDK refuses such a command before it reaches a provider; this is the
/// link's own bound for a caller that reaches it directly.
///
/// # Errors
///
/// [`ErrorKind::Invalid`] naming the count or the first value out of range.
pub fn validate_fractions(axes: &[f64], num_axes: usize) -> Result<(), ProviderError> {
    if axes.len() != num_axes {
        return Err(ProviderError::new(
            ErrorKind::Invalid,
            format!(
                "hand command needs {num_axes} axis values, got {}",
                axes.len()
            ),
        ));
    }
    for (index, value) in axes.iter().enumerate() {
        if !value.is_finite() || !(0.0..=1.0).contains(value) {
            return Err(ProviderError::new(
                ErrorKind::Invalid,
                format!("hand command axis {index} is {value}, outside [0, 1]"),
            ));
        }
    }
    Ok(())
}

/// A codec failure on a hand's wire: a frame that could not be encoded
/// from the values given, refused as invalid.
pub fn hand_codec<E: std::fmt::Display>(error: E) -> ProviderError {
    ProviderError::new(ErrorKind::Invalid, format!("hand codec: {error}"))
}

/// A write the transport did not take.
pub fn hand_write<E: std::fmt::Display>(error: E) -> ProviderError {
    ProviderError::new(ErrorKind::Device, format!("hand write: {error}"))
}

#[cfg(test)]
mod tests {
    use std::sync::atomic::AtomicBool;

    use super::*;

    #[test]
    fn the_probe_tries_the_factory_id_first_then_the_rest_of_the_band() {
        assert_eq!(probe_order(2)[..3], [2, 1, 3]);
        assert_eq!(probe_order(1).len(), usize::from(PROBE_ID_MAX));
    }

    #[tokio::test]
    async fn detection_answers_the_first_id_that_responds() {
        assert_eq!(detect_node(probe_order(2), |id| id == 3).await, Some(3));
        assert_eq!(detect_node(probe_order(2), |_| false).await, None);
    }

    /// A read polling the counter gives the bus up to a command that starts
    /// waiting while it holds the link, and the command then runs.
    #[tokio::test]
    async fn a_read_in_flight_yields_to_a_waiting_command() {
        let link = Arc::new(SerializedLink::new(|waiting| (waiting, 0_u32)));
        let reading = Arc::new(AtomicBool::new(false));
        let flag = Arc::clone(&reading);
        let reader = Arc::clone(&link);
        let read = tokio::spawn(async move {
            reader
                .try_read(move |(waiting, _): &mut (CommandsWaiting, u32)| {
                    flag.store(true, Ordering::Release);
                    while !waiting.any() {
                        std::thread::sleep(COMMAND_PRIORITY_SLICE);
                    }
                    Err::<(), _>(ReadError::Yielded)
                })
                .await
        });
        while !reading.load(Ordering::Acquire) {
            tokio::task::yield_now().await;
        }
        link.command(|(_, commands)| {
            *commands += 1;
            Ok(())
        })
        .await
        .unwrap();
        assert!(matches!(
            read.await.unwrap().unwrap(),
            Some(Err(ReadError::Yielded))
        ));
        let count = link
            .try_read(|(_, commands)| Ok(*commands))
            .await
            .unwrap()
            .unwrap()
            .unwrap();
        assert_eq!(count, 1);
        assert!(!link.commands_waiting.any(), "the announcement is dropped");
    }

    #[test]
    fn the_stale_cache_answers_the_newest_live_value() {
        let cache = StaleCache::default();
        assert_eq!(cache.last(), None::<u8>);
        cache.remember(3);
        cache.remember(4);
        assert_eq!(cache.last(), Some(4));
    }

    #[test]
    fn fractions_must_match_the_axis_count_and_lie_in_the_unit_interval() {
        assert!(validate_fractions(&[0.0, 1.0], 2).is_ok());
        for (axes, count) in [
            (&[0.5][..], 2),
            (&[0.5, f64::NAN][..], 2),
            (&[0.5, 1.5][..], 2),
        ] {
            assert_eq!(
                validate_fractions(axes, count).map_err(|error| error.kind),
                Err(ErrorKind::Invalid)
            );
        }
    }
}
