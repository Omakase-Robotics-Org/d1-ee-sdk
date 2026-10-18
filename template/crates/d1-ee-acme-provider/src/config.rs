//! A side's configuration file, `/etc/d1-ee-acme/<side>.toml`: the manifest's
//! `launch.config_dir` and `launch.config_file`, described by its
//! `config_schema`. Unknown keys are refused, and a value outside its band
//! makes the provider refuse to start rather than run on a default.

use std::path::{Path, PathBuf};

use serde::Deserialize;

/// Where a robot keeps each side's configuration (`launch.config_dir`).
pub const CONFIG_DIR: &str = "/etc/d1-ee-acme";

/// The file of `side` on a robot (`launch.config_file` is `{side}.toml`).
#[must_use]
pub fn default_path(side: &str) -> PathBuf {
    Path::new(CONFIG_DIR).join(format!("{side}.toml"))
}

/// The file's keys; every one is optional.
#[derive(Debug, Clone, Default, PartialEq, Eq, Deserialize)]
#[serde(default, deny_unknown_fields)]
pub struct Config {
    /// The device's bus address, 1 to 247 (default 1).
    pub device_id: Option<u8>,
    /// The stroke speed, percent, 5 to 100 (default 50).
    pub speed_percent: Option<u8>,
}

/// A side's validated settings.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Settings {
    /// The device's bus address.
    pub device_id: u8,
    /// The stroke speed, percent.
    pub speed_percent: u8,
}

impl Default for Settings {
    fn default() -> Self {
        Self {
            device_id: 1,
            speed_percent: 50,
        }
    }
}

impl Config {
    /// Read `path`; a missing file is every built-in value.
    ///
    /// # Errors
    ///
    /// A file that cannot be read or does not parse, naming the path.
    pub fn load(path: &Path) -> Result<Self, String> {
        match std::fs::read_to_string(path) {
            Ok(text) => {
                toml::from_str(&text).map_err(|error| format!("{}: {error}", path.display()))
            }
            Err(error) if error.kind() == std::io::ErrorKind::NotFound => Ok(Self::default()),
            Err(error) => Err(format!("read {}: {error}", path.display())),
        }
    }

    /// The settings, every value checked against its band.
    ///
    /// # Errors
    ///
    /// The first key outside its band.
    pub fn settings(&self) -> Result<Settings, String> {
        let defaults = Settings::default();
        let device_id = self.device_id.unwrap_or(defaults.device_id);
        if !(1..=247).contains(&device_id) {
            return Err(format!("device_id = {device_id} is outside [1, 247]"));
        }
        let speed_percent = self.speed_percent.unwrap_or(defaults.speed_percent);
        if !(5..=100).contains(&speed_percent) {
            return Err(format!(
                "speed_percent = {speed_percent} is outside [5, 100]"
            ));
        }
        Ok(Settings {
            device_id,
            speed_percent,
        })
    }
}
