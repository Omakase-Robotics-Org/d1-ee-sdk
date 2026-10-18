//! The provider's process: its command line, its startup and its service
//! until SIGTERM.

use std::{path::PathBuf, process::ExitCode, sync::Arc, time::Duration};

use clap::Parser;
use d1fw_ee_provider::{
    ServeOptions, ServiceNames, ShmRoot,
    contract::{names, root},
};

use crate::{AcmeProvider, Config, FakeTransport, Transport, config};

/// The command line. `launch.command` in the manifest is
/// `/opt/d1-ee-acme/bin/d1-ee-acme-provider --side {side}`; the test harness
/// appends `--root` and `--prefix`.
#[derive(Debug, Parser)]
#[command(
    version,
    about = "Serves an ACME G1 gripper on one arm side to d1-firmwared"
)]
pub struct Args {
    /// The arm side: `a` (the physical LEFT arm) or `b` (the RIGHT).
    #[arg(long, value_parser = ["a", "b"])]
    pub side: String,
    /// The configuration file; unset, `/etc/d1-ee-acme/<side>.toml`.
    #[arg(long)]
    pub config: Option<PathBuf>,
    /// The shared-memory root d1-firmwared serves under.
    #[arg(long, default_value = root::DEFAULT_ROOT_PATH)]
    pub root: String,
    /// The shared-memory prefix.
    #[arg(long, default_value = root::DEFAULT_PREFIX)]
    pub prefix: String,
    /// How often the state is read and published, milliseconds (10 to 1000).
    #[arg(long, default_value_t = 50)]
    pub state_period_ms: u64,
    /// Drive the fake gripper instead of a device.
    #[arg(long)]
    pub fake: bool,
}

/// The device transport of a real run. The template has none: implement
/// [`Transport`] over the device's bus and open it here.
fn open_device(_settings: &crate::Settings) -> Result<Box<dyn Transport>, String> {
    Err("no device transport is implemented yet; run with --fake".to_owned())
}

/// Parse the command line and serve until SIGTERM or SIGINT.
#[must_use]
pub fn main() -> ExitCode {
    tracing_subscriber::fmt()
        .with_env_filter(
            tracing_subscriber::EnvFilter::try_from_default_env()
                .unwrap_or_else(|_| tracing_subscriber::EnvFilter::new("info")),
        )
        .with_writer(std::io::stderr)
        .init();
    let args = Args::parse();
    let runtime = match tokio::runtime::Runtime::new() {
        Ok(runtime) => runtime,
        Err(error) => {
            eprintln!("d1-ee-acme-provider: start the runtime: {error}");
            return ExitCode::FAILURE;
        }
    };
    match runtime.block_on(run(args)) {
        Ok(()) => ExitCode::SUCCESS,
        Err(error) => {
            eprintln!("d1-ee-acme-provider: {error}");
            ExitCode::from(2)
        }
    }
}

async fn run(args: Args) -> Result<(), String> {
    let path = args
        .config
        .clone()
        .unwrap_or_else(|| config::default_path(&args.side));
    let settings = Config::load(&path)?.settings()?;
    let transport: Box<dyn Transport> = if args.fake {
        Box::new(FakeTransport::default())
    } else {
        open_device(&settings)?
    };
    let root = ShmRoot::new(&args.root, &args.prefix).map_err(|error| error.to_string())?;
    let names =
        ServiceNames::new(&names::default_base(&args.side)).map_err(|error| error.to_string())?;
    let mut options = ServeOptions::new(root, names);
    options.node_name = format!("d1-ee-acme-{}", args.side);
    options.state_period = Duration::from_millis(args.state_period_ms.clamp(10, 1000));
    let provider = Arc::new(AcmeProvider::new(settings, transport));
    let _handle = d1fw_ee_provider::serve(provider, options)
        .await
        .map_err(|error| error.to_string())?;
    let mut terminate = tokio::signal::unix::signal(tokio::signal::unix::SignalKind::terminate())
        .map_err(|error| format!("install the SIGTERM handler: {error}"))?;
    tokio::select! {
        _ = terminate.recv() => {}
        _ = tokio::signal::ctrl_c() => {}
    }
    Ok(())
}
