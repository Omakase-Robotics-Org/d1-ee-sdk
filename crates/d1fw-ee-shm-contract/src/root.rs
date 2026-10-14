//! The shared-memory root every participant of one robot uses.
//!
//! iceoryx2 0.9.3 creates every shared-memory segment, socket and lock file
//! readable and writable by its owner only, so two processes see each
//! other's services only when they run as the same Unix account AND resolve
//! the same root directory and prefix. The contract therefore names both
//! explicitly instead of leaving them to iceoryx2's global configuration,
//! which differs per account (`$HOME/.config/iceoryx2/iceoryx2.toml`) and
//! per unit (`PrivateTmp=` hides the default `/tmp/iceoryx2/`):
//!
//! - the root directory is [`DEFAULT_ROOT_PATH`], inside `d1-firmwared`'s
//!   own runtime directory (`/run/d1-firmwared`, created by systemd for the
//!   unit's account, mode 0750);
//! - the prefix is [`DEFAULT_PREFIX`], which keeps these segments apart from
//!   every other iceoryx2 user's in `/dev/shm`.
//!
//! The account that owns the root is the account `d1-firmwared.service`
//! runs as (`User=` in the unit); a provider runs as that account.

use std::sync::atomic::{AtomicU64, Ordering};

use iceoryx2::{
    config::Config,
    prelude::{FileName, Node, NodeBuilder, NodeName, Path, SemanticString, ipc},
};

use crate::model::ContractError;

/// The root directory of the end-effector services on a robot.
pub const DEFAULT_ROOT_PATH: &str = "/run/d1-firmwared/iceoryx2/";

/// The prefix of every shared-memory resource of the end-effector services.
pub const DEFAULT_PREFIX: &str = "omakase_ee_";

/// Where the services of one robot live: a root directory and a prefix.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct ShmRoot {
    root_path: String,
    prefix: String,
}

impl Default for ShmRoot {
    fn default() -> Self {
        Self {
            root_path: DEFAULT_ROOT_PATH.to_owned(),
            prefix: DEFAULT_PREFIX.to_owned(),
        }
    }
}

impl ShmRoot {
    /// A root at `root_path` (an absolute directory) with `prefix`.
    ///
    /// # Errors
    ///
    /// [`ContractError::Invalid`] when the path is not absolute, or iceoryx2
    /// does not take the path or the prefix.
    pub fn new(root_path: &str, prefix: &str) -> Result<Self, ContractError> {
        if !root_path.starts_with('/') {
            return Err(ContractError::Invalid(format!(
                "the shared-memory root {root_path:?} must be an absolute path"
            )));
        }
        let root_path = if root_path.ends_with('/') {
            root_path.to_owned()
        } else {
            format!("{root_path}/")
        };
        let root = Self {
            root_path,
            prefix: prefix.to_owned(),
        };
        root.config()?;
        Ok(root)
    }

    /// A root of its own for one test: a fresh directory under the system
    /// temporary directory and a prefix no other root of this host uses, so
    /// tests running concurrently (in one process or in several) never see
    /// each other's services. Dropping it removes the directory and every
    /// shared-memory segment under its prefix.
    #[must_use]
    pub fn scratch() -> ScratchRoot {
        static NEXT: AtomicU64 = AtomicU64::new(0);
        let id = NEXT.fetch_add(1, Ordering::Relaxed);
        let pid = std::process::id();
        let nanos = crate::clock::monotonic_ns() % 1_000_000_000;
        let dir = std::env::temp_dir().join(format!("d1fw-ee-shm-{pid}-{id}-{nanos}"));
        ScratchRoot(Self {
            root_path: format!("{}/", dir.display()),
            prefix: format!("t{pid}x{id}x{nanos}_"),
        })
    }

    /// The root directory, with a trailing `/`.
    #[must_use]
    pub fn root_path(&self) -> &str {
        &self.root_path
    }

    /// The resource prefix.
    #[must_use]
    pub fn prefix(&self) -> &str {
        &self.prefix
    }

    /// iceoryx2's default configuration with this root and prefix: the
    /// configuration every participant builds its node from.
    ///
    /// # Errors
    ///
    /// [`ContractError::Invalid`] when iceoryx2 does not take the path or
    /// the prefix.
    pub fn config(&self) -> Result<Config, ContractError> {
        let mut config = Config::default();
        let path = Path::new(self.root_path.as_bytes()).map_err(|error| {
            ContractError::Invalid(format!(
                "iceoryx2 does not take the root {:?}: {error:?}",
                self.root_path
            ))
        })?;
        config.global.set_root_path(&path);
        config.global.prefix = FileName::new(self.prefix.as_bytes()).map_err(|error| {
            ContractError::Invalid(format!(
                "iceoryx2 does not take the prefix {:?}: {error:?}",
                self.prefix
            ))
        })?;
        Ok(config)
    }

    /// Create the iceoryx2 node `name` under this root, creating the root
    /// directory first when its parent allows it.
    ///
    /// # Errors
    ///
    /// [`ContractError::Transport`] when the directory or the node cannot be
    /// created (a root whose parent does not exist yet, an account that does
    /// not own it).
    pub fn node(&self, name: &str) -> Result<Node<ipc::Service>, ContractError> {
        std::fs::create_dir_all(&self.root_path).map_err(|error| {
            ContractError::Transport(format!(
                "create the shared-memory root {}: {error}",
                self.root_path
            ))
        })?;
        let config = self.config()?;
        let node_name = NodeName::new(name).map_err(|error| {
            ContractError::Invalid(format!("invalid iceoryx2 node name {name:?}: {error:?}"))
        })?;
        NodeBuilder::new()
            .config(&config)
            .name(&node_name)
            .create::<ipc::Service>()
            .map_err(|error| {
                ContractError::Transport(format!(
                    "create iceoryx2 node {name:?} under {} (prefix {}): {error:?}",
                    self.root_path, self.prefix
                ))
            })
    }
}

/// A [`ShmRoot`] for one test, removed with everything under it when
/// dropped: the root directory, and the segments iceoryx2 left in
/// `/dev/shm` under the prefix (the per-root management segment outlives
/// every node, and a provider killed in a test leaves its own).
#[derive(Debug)]
pub struct ScratchRoot(ShmRoot);

impl std::ops::Deref for ScratchRoot {
    type Target = ShmRoot;

    fn deref(&self) -> &ShmRoot {
        &self.0
    }
}

impl Drop for ScratchRoot {
    fn drop(&mut self) {
        let _ = std::fs::remove_dir_all(&self.0.root_path);
        if let Ok(entries) = std::fs::read_dir("/dev/shm") {
            for entry in entries.flatten() {
                if entry
                    .file_name()
                    .to_string_lossy()
                    .starts_with(self.0.prefix.as_str())
                {
                    let _ = std::fs::remove_file(entry.path());
                }
            }
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn the_default_root_is_the_daemons_runtime_directory() {
        let root = ShmRoot::default();
        assert_eq!(root.root_path(), "/run/d1-firmwared/iceoryx2/");
        assert_eq!(root.prefix(), "omakase_ee_");
        let config = root.config().unwrap();
        assert_eq!(
            config.global.root_path().to_string(),
            "/run/d1-firmwared/iceoryx2/"
        );
    }

    #[test]
    fn a_relative_root_is_refused_and_a_slash_is_added() {
        assert!(ShmRoot::new("run/x", "p_").is_err());
        assert_eq!(ShmRoot::new("/run/x", "p_").unwrap().root_path(), "/run/x/");
    }

    #[test]
    fn scratch_roots_differ() {
        let first = ShmRoot::scratch();
        let second = ShmRoot::scratch();
        assert_ne!(*first, *second);
        assert!(first.config().is_ok());
    }
}
