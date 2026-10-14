//! `d1-ee-test-provider`: the SDK's `TestProvider` (a simulated one-joint
//! gripper) served as a process on one side, until SIGTERM or SIGINT.

use std::{process::ExitCode, sync::Arc, time::Duration};

use clap::Parser;
use d1fw_ee_provider::{
    ServeOptions, ServiceNames, ShmRoot, TestProvider,
    contract::{names, root},
};

/// The command line.
#[derive(Debug, Parser)]
#[command(
    version,
    about = "Serves a simulated one-joint gripper over end_effector.shm/1"
)]
struct Args {
    /// The arm side: `a` or `b`.
    #[arg(long, default_value = "a")]
    side: String,
    /// The shared-memory root d1-firmwared serves under.
    #[arg(long, default_value = root::DEFAULT_ROOT_PATH)]
    root: String,
    /// The shared-memory prefix.
    #[arg(long, default_value = root::DEFAULT_PREFIX)]
    prefix: String,
    /// The base of the side's services; unset, `omakase/ee/<side>`.
    #[arg(long)]
    service: Option<String>,
    /// How often the state is published, milliseconds.
    #[arg(long, default_value_t = 100)]
    state_period_ms: u64,
}

#[tokio::main]
async fn main() -> ExitCode {
    tracing_subscriber::fmt()
        .with_env_filter(
            tracing_subscriber::EnvFilter::try_from_default_env()
                .unwrap_or_else(|_| tracing_subscriber::EnvFilter::new("info")),
        )
        .with_writer(std::io::stderr)
        .init();
    let args = Args::parse();
    let root = match ShmRoot::new(&args.root, &args.prefix) {
        Ok(root) => root,
        Err(error) => {
            eprintln!("d1-ee-test-provider: {error}");
            return ExitCode::from(2);
        }
    };
    let base = args
        .service
        .clone()
        .unwrap_or_else(|| names::default_base(&args.side));
    let names = match ServiceNames::new(&base) {
        Ok(names) => names,
        Err(error) => {
            eprintln!("d1-ee-test-provider: --service: {error}");
            return ExitCode::from(2);
        }
    };
    let mut options = ServeOptions::new(root, names);
    options.node_name = format!("test-provider-{}", args.side);
    options.state_period = Duration::from_millis(args.state_period_ms.clamp(1, 1000));
    let _handle = match d1fw_ee_provider::serve(Arc::new(TestProvider::default()), options).await {
        Ok(handle) => handle,
        Err(error) => {
            eprintln!("d1-ee-test-provider: {error}");
            return ExitCode::FAILURE;
        }
    };
    let mut terminate = tokio::signal::unix::signal(tokio::signal::unix::SignalKind::terminate())
        .expect("install the SIGTERM handler");
    tokio::select! {
        _ = terminate.recv() => {}
        _ = tokio::signal::ctrl_c() => {}
    }
    ExitCode::SUCCESS
}
