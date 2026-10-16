//! The iceoryx2 settings of every arm service.
//!
//! Either side may create a service first, and iceoryx2 refuses to open a
//! service with settings incompatible with those it was created with, so
//! the daemon and every client open each service through these functions
//! (a client in another language uses the same values; each contract's text
//! lists them). The functions are generic over the iceoryx2 service
//! variant: the daemon publishes from several threads and uses
//! `ipc_threadsafe`, a client typically uses `ipc`; both variants share
//! the same shared-memory layout and talk to each other.

use core::fmt::Debug;

use iceoryx2::{
    port::{listener::Listener, notifier::Notifier},
    prelude::{Node, ServiceName, ZeroCopySend},
    service::{
        Service,
        port_factory::{event, publish_subscribe},
    },
};

use d1fw_ee_shm_contract::model::ContractError;
pub use d1fw_ee_shm_contract::ports::TopicSpec;

/// `feedback`: a stream every record of which a reader can take. A buffer
/// of 256 holds a quarter second of records at the controller's 1 kHz, so a
/// reader that wakes every few milliseconds loses none; one that falls
/// further behind loses the oldest and counts the loss from the gaps in the
/// header `seq`. History 1, so a reader that starts gets the newest record
/// on the next publish.
pub const FEEDBACK_TOPIC: TopicSpec = TopicSpec {
    history_size: 1,
    buffer_size: 256,
    safe_overflow: true,
    max_publishers: 1,
    max_subscribers: 8,
};

/// `status`: latched; only the newest matters.
pub const STATUS_TOPIC: TopicSpec = TopicSpec {
    history_size: 1,
    buffer_size: 2,
    safe_overflow: true,
    max_publishers: 1,
    max_subscribers: 8,
};

/// `events`: a reader that starts gets the last 64 events, and one that
/// falls behind by more than 256 loses the oldest (gaps in `seq`).
pub const EVENTS_TOPIC: TopicSpec = TopicSpec {
    history_size: 64,
    buffer_size: 256,
    safe_overflow: true,
    max_publishers: 1,
    max_subscribers: 8,
};

/// `command`: never replayed (history 0: a daemon that starts must not act
/// on a target sent before it), newest wins (the daemon writes the newest
/// command it holds and counts the older ones as superseded). Up to four
/// clients may hold a publisher; the lease token decides whose commands are
/// written.
pub const COMMAND_TOPIC: TopicSpec = TopicSpec {
    history_size: 0,
    buffer_size: 4,
    safe_overflow: true,
    max_publishers: 4,
    max_subscribers: 1,
};

/// `command_status`: one sample per command the daemon took, a stream; the
/// cumulative counters in each let a reader that lost some count them.
pub const COMMAND_STATUS_TOPIC: TopicSpec = TopicSpec {
    history_size: 1,
    buffer_size: 256,
    safe_overflow: true,
    max_publishers: 1,
    max_subscribers: 8,
};

/// The settings of an event service.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct EventSpec {
    /// Notifiers the service admits.
    pub max_notifiers: usize,
    /// Listeners the service admits.
    pub max_listeners: usize,
}

/// `command` (event): a client notifies it after each command it
/// publishes, which wakes the daemon's command thread at once; the daemon
/// alone listens.
pub const COMMAND_EVENT: EventSpec = EventSpec {
    max_notifiers: 4,
    max_listeners: 1,
};

fn service_name(name: &str) -> Result<ServiceName, ContractError> {
    ServiceName::new(name).map_err(|error| {
        ContractError::Invalid(format!("invalid iceoryx2 service name {name:?}: {error:?}"))
    })
}

/// Open (or create) the publish-subscribe service `name` with `spec`, on
/// either service variant.
///
/// # Errors
///
/// [`ContractError::Transport`] when iceoryx2 refuses: most often a service
/// that exists with another payload type (a peer built against another
/// major) or other settings.
pub fn open_topic<S: Service, T: Debug + ZeroCopySend>(
    node: &Node<S>,
    name: &str,
    spec: TopicSpec,
) -> Result<publish_subscribe::PortFactory<S, T, ()>, ContractError> {
    node.service_builder(&service_name(name)?)
        .publish_subscribe::<T>()
        .history_size(spec.history_size)
        .subscriber_max_buffer_size(spec.buffer_size)
        .enable_safe_overflow(spec.safe_overflow)
        .max_publishers(spec.max_publishers)
        .max_subscribers(spec.max_subscribers)
        .open_or_create()
        .map_err(|error| ContractError::Transport(format!("open topic {name}: {error:?}")))
}

/// Open (or create) the event service `name` with `spec`.
///
/// # Errors
///
/// As [`open_topic`].
pub fn open_event<S: Service>(
    node: &Node<S>,
    name: &str,
    spec: EventSpec,
) -> Result<event::PortFactory<S>, ContractError> {
    node.service_builder(&service_name(name)?)
        .event()
        .max_notifiers(spec.max_notifiers)
        .max_listeners(spec.max_listeners)
        .open_or_create()
        .map_err(|error| ContractError::Transport(format!("open event {name}: {error:?}")))
}

/// A notifier on the `command` event service.
///
/// # Errors
///
/// As [`open_topic`].
pub fn command_notifier<S: Service>(
    node: &Node<S>,
    name: &str,
) -> Result<Notifier<S>, ContractError> {
    open_event(node, name, COMMAND_EVENT)?
        .notifier_builder()
        .create()
        .map_err(|error| ContractError::Transport(format!("notifier on {name}: {error:?}")))
}

/// The daemon's listener on the `command` event service.
///
/// # Errors
///
/// As [`open_topic`].
pub fn command_listener<S: Service>(
    node: &Node<S>,
    name: &str,
) -> Result<Listener<S>, ContractError> {
    open_event(node, name, COMMAND_EVENT)?
        .listener_builder()
        .create()
        .map_err(|error| ContractError::Transport(format!("listener on {name}: {error:?}")))
}
