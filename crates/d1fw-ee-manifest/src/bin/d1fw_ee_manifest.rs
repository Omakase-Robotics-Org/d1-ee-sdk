//! `d1fw-ee-manifest`: check a provider manifest (`omakase-ee-provider/1`),
//! alone or against the descriptor a provider announced.
//!
//! Exit status: 0 valid, 1 invalid, 2 a file that cannot be read or a
//! descriptor that is not one.

use std::{path::PathBuf, process::ExitCode};

use clap::{Parser, Subcommand};
use d1fw_ee_manifest::{DescriptorView, ProviderManifest, SCHEMA_JSON, SIDES};
use serde_json::Value;

/// The command line.
#[derive(Debug, Parser)]
#[command(
    version,
    about = "Checks D1 end-effector provider manifests (omakase-ee-provider/1)"
)]
struct Args {
    #[command(subcommand)]
    command: Command,
}

#[derive(Debug, Subcommand)]
enum Command {
    /// Validate a manifest: the JSON Schema, then the rules it cannot state.
    /// With --descriptor, also hold its models, joints and postures to a
    /// provider's announced descriptor.
    Check {
        /// The manifest, JSON.
        file: PathBuf,
        /// The answer of `GET /v1/end_effectors/{side}` (or a bare
        /// descriptor), JSON.
        #[arg(long)]
        descriptor: Option<PathBuf>,
    },
    /// Print the JSON Schema of omakase-ee-provider/1.
    Schema,
}

fn read_json(path: &PathBuf) -> Result<String, String> {
    std::fs::read_to_string(path).map_err(|error| format!("read {}: {error}", path.display()))
}

fn check(file: &PathBuf, descriptor: Option<&PathBuf>) -> ExitCode {
    let text = match read_json(file) {
        Ok(text) => text,
        Err(error) => {
            eprintln!("d1fw-ee-manifest: {error}");
            return ExitCode::from(2);
        }
    };
    let manifest = match ProviderManifest::parse(&text) {
        Ok(manifest) => manifest,
        Err(invalid) => {
            eprintln!("{}: invalid", file.display());
            for problem in &invalid.0 {
                eprintln!("  {problem}");
            }
            return ExitCode::from(1);
        }
    };
    println!(
        "{}: valid {} {} ({} {}, {} model(s))",
        file.display(),
        manifest.id,
        manifest.version,
        manifest.package.name,
        manifest.package.arch,
        manifest.models.len()
    );
    for side in SIDES {
        if let (Ok(command), Ok(config)) =
            (manifest.command_for(side), manifest.config_path_for(side))
        {
            println!("  side {side}: EE_PROVIDER_COMMAND={command}  config {config}");
        }
    }
    let Some(path) = descriptor else {
        return ExitCode::SUCCESS;
    };
    let view = match read_json(path).and_then(|text| {
        serde_json::from_str::<Value>(&text)
            .map_err(|error| format!("{}: not JSON: {error}", path.display()))
            .and_then(|value| DescriptorView::from_json(&value))
    }) {
        Ok(view) => view,
        Err(error) => {
            eprintln!("d1fw-ee-manifest: {error}");
            return ExitCode::from(2);
        }
    };
    match manifest.check_descriptor(&view) {
        Ok(matched) => {
            println!(
                "{}: the descriptor matches model {}",
                path.display(),
                matched.model
            );
            for (name, targets) in &matched.postures {
                println!("  posture {name}: {targets:?}");
            }
            ExitCode::SUCCESS
        }
        Err(invalid) => {
            eprintln!(
                "{}: the descriptor disagrees with the manifest",
                path.display()
            );
            for problem in &invalid.0 {
                eprintln!("  {problem}");
            }
            ExitCode::from(1)
        }
    }
}

fn main() -> ExitCode {
    match Args::parse().command {
        Command::Check { file, descriptor } => check(&file, descriptor.as_ref()),
        Command::Schema => {
            print!("{SCHEMA_JSON}");
            ExitCode::SUCCESS
        }
    }
}
