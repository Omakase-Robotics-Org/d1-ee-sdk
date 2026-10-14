//! The iceoryx2 settings of every service, and the timing rules of the
//! contract.
//!
//! Either side may create a service first, and iceoryx2 refuses to open a
//! service with settings incompatible with the ones it was created with, so
//! both sides open every service through these functions. A provider written
//! against another binding (the Python one) must use the same values; the
//! contract's table lists them.

use core::fmt::Debug;

use iceoryx2::{
    prelude::{Node, ServiceName, ZeroCopySend, ipc},
    service::port_factory::{publish_subscribe, request_response},
};

use crate::model::ContractError;

/// The iceoryx2 service variant of the contract: inter-process shared
/// memory.
pub type Ipc = ipc::Service;

/// A publish-subscribe service of the contract.
pub type Topic<T> = publish_subscribe::PortFactory<Ipc, T, ()>;

/// A request-response service of the contract.
pub type Rpc<Req, Res> = request_response::PortFactory<Ipc, Req, (), Res, ()>;

/// The settings of one publish-subscribe service.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct TopicSpec {
    /// Samples a late subscriber receives on the publisher's next send.
    pub history_size: usize,
    /// Samples a subscriber buffers.
    pub buffer_size: usize,
    /// Whether a full buffer drops its oldest sample instead of refusing the
    /// newest.
    pub safe_overflow: bool,
    /// Publishers the service admits. Exactly one on every topic, so a
    /// second provider (or a second daemon) cannot bind a side that already
    /// has one.
    pub max_publishers: usize,
    /// Subscribers the service admits: the peer and diagnostic observers.
    pub max_subscribers: usize,
}

/// `descriptor`: latched, so a daemon that starts after the provider gets it
/// at once.
pub const DESCRIPTOR_TOPIC: TopicSpec = TopicSpec {
    history_size: 1,
    buffer_size: 2,
    safe_overflow: true,
    max_publishers: 1,
    max_subscribers: 4,
};

/// `heartbeat`: only the newest matters.
pub const HEARTBEAT_TOPIC: TopicSpec = DESCRIPTOR_TOPIC;

/// `state`: only the newest matters.
pub const STATE_TOPIC: TopicSpec = DESCRIPTOR_TOPIC;

/// `tick`: a stream; an observer that falls behind loses the oldest, and
/// counts what it lost from `tick_seq`.
pub const TICK_TOPIC: TopicSpec = TopicSpec {
    history_size: 0,
    buffer_size: 64,
    safe_overflow: true,
    max_publishers: 1,
    max_subscribers: 4,
};

/// `joints`: never replayed (history 0: a provider that starts must not act
/// on a target sent before it), and newest wins (a target that is dropped
/// because the provider fell behind is superseded by a newer one, which the
/// timestamp rule would prefer anyway).
pub const JOINTS_TOPIC: TopicSpec = TopicSpec {
    history_size: 0,
    buffer_size: 4,
    safe_overflow: true,
    max_publishers: 1,
    max_subscribers: 2,
};

/// `control`: latched, so a provider that starts learns the current stop
/// generation (as its baseline, never as a stop to act on).
pub const CONTROL_TOPIC: TopicSpec = DESCRIPTOR_TOPIC;

/// The settings of one request-response service.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct RpcSpec {
    /// Servers the service admits: one, the provider.
    pub max_servers: usize,
    /// Clients the service admits: the daemon and one diagnostic client.
    pub max_clients: usize,
    /// Requests one client may have in flight.
    pub max_active_requests_per_client: usize,
    /// Responses buffered per request.
    pub max_response_buffer_size: usize,
}

/// `command` and `fault`. Neither direction overflows safely: a request or
/// a response is never dropped silently. A `stop` that cannot be delivered
/// is reported to the caller, and the control topic's stop generation
/// carries it regardless.
pub const RPC: RpcSpec = RpcSpec {
    max_servers: 1,
    max_clients: 2,
    max_active_requests_per_client: 4,
    max_response_buffer_size: 2,
};

/// How often a latched topic is republished, milliseconds. iceoryx2 hands a
/// late subscriber its history on the publisher's next send, so a latched
/// value must be sent again on a bounded period.
pub const LATCHED_REPUBLISH_MS: u64 = 500;

/// The longest a provider may leave between two heartbeats, milliseconds.
pub const MAX_HEARTBEAT_PERIOD_MS: u64 = 100;

/// How long the daemon waits past a provider's newest heartbeat before it
/// reports the side unavailable, milliseconds, unless the side's
/// configuration names another value.
pub const DEFAULT_STALE_AFTER_MS: u64 = 500;

/// How often the daemon publishes `control`, milliseconds; a provider
/// treats the daemon as absent after [`DEFAULT_STALE_AFTER_MS`] without it.
pub const CONTROL_PERIOD_MS: u64 = 100;

/// How long after a `set` request is issued the provider may still start
/// it, milliseconds: `start_by_mono_ns` is the issue time plus this.
pub const START_WINDOW_MS: u64 = 250;

/// How old a `joints` sample may be when the provider takes it,
/// milliseconds (from its header's `mono_ns`); an older one is dropped.
pub const JOINTS_MAX_AGE_MS: u64 = 200;

fn service_name(name: &str) -> Result<ServiceName, ContractError> {
    ServiceName::new(name).map_err(|error| {
        ContractError::Invalid(format!("invalid iceoryx2 service name {name:?}: {error:?}"))
    })
}

/// Open (or create) the publish-subscribe service `name` with `spec`.
///
/// # Errors
///
/// [`ContractError::Transport`] when iceoryx2 refuses: most often a service
/// that exists with another payload type (a peer built against another
/// major version or another layout) or with other settings.
pub fn open_topic<T: Debug + ZeroCopySend>(
    node: &Node<Ipc>,
    name: &str,
    spec: TopicSpec,
) -> Result<Topic<T>, ContractError> {
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

/// Open (or create) the request-response service `name` with `spec`.
///
/// # Errors
///
/// As [`open_topic`].
pub fn open_rpc<Req: Debug + ZeroCopySend, Res: Debug + ZeroCopySend>(
    node: &Node<Ipc>,
    name: &str,
    spec: RpcSpec,
) -> Result<Rpc<Req, Res>, ContractError> {
    node.service_builder(&service_name(name)?)
        .request_response::<Req, Res>()
        .max_servers(spec.max_servers)
        .max_clients(spec.max_clients)
        .max_active_requests_per_client(spec.max_active_requests_per_client)
        .max_response_buffer_size(spec.max_response_buffer_size)
        .enable_safe_overflow_for_requests(false)
        .enable_safe_overflow_for_responses(false)
        .open_or_create()
        .map_err(|error| ContractError::Transport(format!("open service {name}: {error:?}")))
}
