//! A hand provider's process: its command line, its startup (the flange
//! channel, the relay's admission, the probe for the hand's id) and its
//! service until a signal.

use std::{process::ExitCode, sync::Arc, time::Duration};

use clap::Parser;
use d1fw_canfd::CanFdBus;
use d1fw_ee_provider::{
    FlangeBus, FlangeNames, FlangeOptions, ProviderHandle, ServeOptions, ServiceNames, ShmRoot,
    contract::{flange, names, root},
};
use tracing::{info, warn};

use crate::{
    channel::FlangeChannel,
    link::{CommandsWaiting, PROBE_ID_MAX, detect_node, probe_order},
    provider::{HandFacts, HandLink, HandProvider, Side},
};

/// How long a side whose hand answered no probed id waits before it is
/// probed again.
pub const REPROBE_INTERVAL: Duration = Duration::from_secs(2);

/// How long the provider waits before it tries again to open services it
/// could not open (a daemon that has not created its runtime directory yet,
/// another provider still holding the side).
const REOPEN: Duration = Duration::from_secs(1);

/// What a hand model's crate supplies to run as a provider.
pub trait HandDevice: Send + Sync + 'static {
    /// Its link.
    type Link: HandLink;

    /// What the model declares.
    fn facts(&self) -> HandFacts;

    /// The id the startup probe tries first on `side` (the factory id).
    fn probe_first(&self, side: Side) -> u16;

    /// A link to the hand at `node` over `bus`, whose reads poll `waiting`.
    fn open(&self, bus: Box<dyn CanFdBus>, node: u16, waiting: CommandsWaiting) -> Self::Link;

    /// A simulated hand of this model answering at `node`, on a bus of its
    /// own (`--sim`).
    fn simulated(&self, node: u16) -> Box<dyn CanFdBus>;
}

/// The provider's command line.
#[derive(Debug, Clone, Parser)]
#[command(version)]
pub struct Args {
    /// The arm side the hand is on: `a` (the physical left arm) or `b`.
    #[arg(long)]
    pub side: Side,
    /// The shared-memory root d1-firmwared serves under
    /// (`[end_effector] shm_root_path`).
    #[arg(long, default_value = root::DEFAULT_ROOT_PATH)]
    pub root: String,
    /// The shared-memory prefix (`[end_effector] shm_prefix`).
    #[arg(long, default_value = root::DEFAULT_PREFIX)]
    pub prefix: String,
    /// The base of the side's `end_effector.shm/1` services; unset,
    /// `omakase/ee/<side>`.
    #[arg(long)]
    pub service: Option<String>,
    /// The base of the side's `flange_bus.shm/1` services; unset,
    /// `omakase/flange/<side>`.
    #[arg(long)]
    pub flange: Option<String>,
    /// How often the hand is read and its state published, milliseconds
    /// (1 to 1000).
    #[arg(long, default_value_t = 100)]
    pub state_period_ms: u64,
    /// Drive a simulated hand on a bus of its own instead of the flange
    /// channel: the side's `end_effector.shm/1` services are served as on a
    /// robot, and nothing reaches the arm.
    #[arg(long)]
    pub sim: bool,
}

/// Where and how one hand is served.
#[derive(Debug, Clone)]
pub struct HandOptions {
    /// The arm side.
    pub side: Side,
    /// The shared-memory root.
    pub root: ShmRoot,
    /// The side's `end_effector.shm/1` services.
    pub service: ServiceNames,
    /// The side's `flange_bus.shm/1` services.
    pub flange: FlangeNames,
    /// How often the hand is read.
    pub state_period: Duration,
    /// Drive a simulated hand instead of the flange channel.
    pub sim: bool,
}

impl HandOptions {
    /// The options `args` names.
    ///
    /// # Errors
    ///
    /// A root, prefix, service base or state period the contracts refuse.
    pub fn from_args(args: &Args) -> Result<Self, String> {
        let side = args.side.as_str();
        let root = ShmRoot::new(&args.root, &args.prefix).map_err(|error| error.to_string())?;
        let service = ServiceNames::new(
            &args
                .service
                .clone()
                .unwrap_or_else(|| names::default_base(side)),
        )
        .map_err(|error| format!("--service: {error}"))?;
        let flange = FlangeNames::new(
            &args
                .flange
                .clone()
                .unwrap_or_else(|| flange::default_base(side)),
        )
        .map_err(|error| format!("--flange: {error}"))?;
        if !(1..=1000).contains(&args.state_period_ms) {
            return Err(format!(
                "--state-period-ms {} must be 1 to 1000",
                args.state_period_ms
            ));
        }
        Ok(Self {
            side: args.side,
            root,
            service,
            flange,
            state_period: Duration::from_millis(args.state_period_ms),
            sim: args.sim,
        })
    }
}

/// A hand being served. Dropping it stops serving and closes the flange
/// channel.
pub struct Running {
    /// The `end_effector.shm/1` side being served.
    pub provider: ProviderHandle,
    /// The flange channel, unless the hand is simulated.
    pub flange: Option<Arc<FlangeBus>>,
    /// The id the hand answered at.
    pub node: u16,
}

/// Open the side's flange channel, retrying until the services can be
/// created.
async fn open_flange(options: &HandOptions) -> Arc<FlangeBus> {
    let mut attempts = 0_u32;
    loop {
        let mut flange_options = FlangeOptions::new(options.root.clone(), options.flange.clone());
        flange_options.node_name = format!("ee-hand-flange-{}", options.side.as_str());
        match FlangeBus::open(flange_options) {
            Ok(bus) => return Arc::new(bus),
            Err(error) => {
                if attempts.is_multiple_of(30) {
                    warn!(
                        %error,
                        service = options.flange.base(),
                        "the flange channel cannot be opened yet; retrying every second"
                    );
                }
                attempts = attempts.saturating_add(1);
                tokio::time::sleep(REOPEN).await;
            }
        }
    }
}

/// Wait until the relay binds this provider with the channel up.
async fn admitted(bus: &Arc<FlangeBus>) {
    let mut reported = None;
    loop {
        let waiting = Arc::clone(bus);
        let outcome =
            tokio::task::spawn_blocking(move || waiting.wait_admitted(Duration::from_secs(1)))
                .await;
        match outcome {
            Ok(Ok(status)) => {
                info!(
                    suppresses_repeats = status.channel.suppresses_repeats,
                    max_tx_hz = status.max_tx_hz,
                    latched = status.latched,
                    "admitted by the flange relay"
                );
                return;
            }
            Ok(Err(error)) => {
                let text = error.to_string();
                if reported.as_ref() != Some(&text) {
                    warn!(reason = text.as_str(), "waiting for the flange relay");
                    reported = Some(text);
                }
            }
            Err(error) => warn!(%error, "the admission wait failed"),
        }
    }
}

/// Probe `side` for the hand until it answers, and return its id.
async fn find<D: HandDevice>(
    device: &Arc<D>,
    side: Side,
    bus: impl Fn() -> Box<dyn CanFdBus> + Clone + Send + Sync + 'static,
) -> u16 {
    let mut reported = false;
    loop {
        let probe_device = Arc::clone(device);
        let probe_bus = bus.clone();
        let found = detect_node(probe_order(device.probe_first(side)), move |node| {
            probe_device
                .open(probe_bus(), node, CommandsWaiting::default())
                .probe()
        })
        .await;
        if let Some(node) = found {
            return node;
        }
        if !reported {
            warn!(
                side = side.as_str(),
                "no hand answered ids 1-{PROBE_ID_MAX}; probing again every {} s",
                REPROBE_INTERVAL.as_secs()
            );
            reported = true;
        }
        tokio::time::sleep(REPROBE_INTERVAL).await;
    }
}

/// Bring the hand up and serve it: the flange channel (or a simulated hand),
/// the relay's admission, the probe, and the side's `end_effector.shm/1`
/// services. Waits as long as each step needs.
///
/// # Errors
///
/// The services of the side cannot be served (another provider holds them).
pub async fn start<D: HandDevice>(device: D, options: &HandOptions) -> Result<Running, String> {
    let device = Arc::new(device);
    let facts = device.facts();
    let (bus, flange): (Box<dyn Fn() -> Box<dyn CanFdBus> + Send + Sync>, _) = if options.sim {
        let simulated: Arc<dyn CanFdBus> =
            Arc::from(device.simulated(device.probe_first(options.side)));
        (
            Box::new(move || Box::new(Shared(Arc::clone(&simulated))) as Box<dyn CanFdBus>),
            None,
        )
    } else {
        let flange = open_flange(options).await;
        admitted(&flange).await;
        let channel = FlangeChannel::new(Arc::clone(&flange));
        (
            Box::new(move || Box::new(channel.clone()) as Box<dyn CanFdBus>),
            Some(flange),
        )
    };
    let bus: Arc<dyn Fn() -> Box<dyn CanFdBus> + Send + Sync> = Arc::from(bus);
    let node = {
        let bus = Arc::clone(&bus);
        find(&device, options.side, move || bus()).await
    };
    info!(
        side = options.side.as_str(),
        model = facts.model,
        node,
        "hand found by the probe"
    );
    if let Some(flange) = &flange {
        FlangeChannel::new(Arc::clone(flange)).drain();
    }
    let link_bus = bus();
    let provider = HandProvider::new(facts, options.side, node, {
        let device = Arc::clone(&device);
        move |waiting| device.open(link_bus, node, waiting)
    });
    let mut serve_options = ServeOptions::new(options.root.clone(), options.service.clone());
    serve_options.node_name = format!("ee-hand-{}", options.side.as_str());
    serve_options.state_period = options.state_period;
    let provider = d1fw_ee_provider::serve(Arc::new(provider), serve_options)
        .await
        .map_err(|error| error.to_string())?;
    Ok(Running {
        provider,
        flange,
        node,
    })
}

/// One simulated bus shared by every link the provider opens.
struct Shared(Arc<dyn CanFdBus>);

impl CanFdBus for Shared {
    fn send(&self, frame: &d1fw_canfd::CanFdFrame) -> Result<(), d1fw_canfd::CanFdError> {
        self.0.send(frame)
    }

    fn send_as(
        &self,
        frame: &d1fw_canfd::CanFdFrame,
        kind: d1fw_canfd::FrameKind,
    ) -> Result<(), d1fw_canfd::CanFdError> {
        self.0.send_as(frame, kind)
    }

    fn recv(
        &self,
        timeout: Duration,
    ) -> Result<Option<d1fw_canfd::CanFdFrame>, d1fw_canfd::CanFdError> {
        self.0.recv(timeout)
    }

    fn close(&self) {}

    fn suppresses_repeat_frames(&self) -> bool {
        self.0.suppresses_repeat_frames()
    }
}

/// The provider process: parse the command line, serve the hand until
/// SIGTERM or SIGINT, and exit.
#[must_use]
pub fn main<D: HandDevice>(device: D) -> ExitCode {
    let args = Args::parse();
    tracing_subscriber::fmt()
        .with_env_filter(
            tracing_subscriber::EnvFilter::try_from_default_env()
                .unwrap_or_else(|_| tracing_subscriber::EnvFilter::new("info")),
        )
        .with_writer(std::io::stderr)
        .init();
    let options = match HandOptions::from_args(&args) {
        Ok(options) => options,
        Err(error) => {
            eprintln!("{error}");
            return ExitCode::from(2);
        }
    };
    let runtime = match tokio::runtime::Builder::new_multi_thread()
        .worker_threads(2)
        .enable_all()
        .build()
    {
        Ok(runtime) => runtime,
        Err(error) => {
            eprintln!("start the runtime: {error}");
            return ExitCode::FAILURE;
        }
    };
    runtime.block_on(async move {
        let running = match start(device, &options).await {
            Ok(running) => running,
            Err(error) => {
                tracing::error!(%error, "the hand cannot be served");
                return ExitCode::FAILURE;
            }
        };
        info!(
            side = options.side.as_str(),
            service = options.service.base(),
            flange = options.flange.base(),
            simulated = options.sim,
            node = running.node,
            "hand provider serving"
        );
        shutdown_signal().await;
        info!("hand provider stopping");
        drop(running);
        ExitCode::SUCCESS
    })
}

async fn shutdown_signal() {
    let terminate = async {
        match tokio::signal::unix::signal(tokio::signal::unix::SignalKind::terminate()) {
            Ok(mut signal) => {
                signal.recv().await;
            }
            Err(_) => std::future::pending::<()>().await,
        }
    };
    tokio::select! {
        _ = tokio::signal::ctrl_c() => {}
        () = terminate => {}
    }
}
