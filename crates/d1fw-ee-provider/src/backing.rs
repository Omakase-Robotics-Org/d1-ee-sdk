//! Noticing that another process removed the services' files.
//!
//! systemd-logind's `RemoveIPC=yes` (its default) deletes every POSIX
//! shared-memory object an account owns when that account's last login
//! session ends, and d1-firmwared and its providers run under a login
//! account. The participants keep their mappings, so the topics they already
//! exchange keep flowing, but nothing can be joined any more: a connection
//! made later lands on a segment the other side never maps, and a restarted
//! participant finds the static configuration without its dynamic segment.
//! The daemon then opens its services afresh (removing what is left of the
//! old ones); a provider that notices the removal gives up its ports and
//! opens them again, which succeeds once the daemon has.

use std::{os::unix::fs::MetadataExt, path::PathBuf};

use d1fw_ee_shm_contract::{model::ContractError, ports::Ipc};
use iceoryx2::{
    config::Config,
    prelude::{FileName, SemanticString},
    service::{Service, dynamic_config::DynamicConfig, port_factory::PortFactory},
};
use iceoryx2_cal::named_concept::{NamedConceptConfiguration, NamedConceptMgmt};

/// How often the services' dynamic segments are checked, and how often a
/// port thread that gave up its ports tries to open them again.
pub(crate) const PERIOD: std::time::Duration = std::time::Duration::from_secs(1);

/// Where POSIX shared-memory objects live on Linux: `shm_open(name)` is the
/// file `/dev/shm/name`.
const POSIX_SHM_DIR: &str = "/dev/shm";

/// The dynamic segments of the services one participant opened, each with
/// the inode it had when opened: another participant may have created a new
/// segment under the same name since.
#[derive(Debug, Default)]
pub(crate) struct Backing {
    segments: Vec<(String, PathBuf, u64)>,
}

impl Backing {
    /// `factory`, its service's dynamic segment added to the watched ones.
    pub(crate) fn watch<F: PortFactory>(
        &mut self,
        config: &Config,
        factory: F,
    ) -> Result<F, ContractError> {
        let hash = factory.service_hash().as_str();
        let name = FileName::new(hash.as_bytes()).map_err(|error| {
            ContractError::Transport(format!("service hash {hash:?}: {error:?}"))
        })?;
        let dynamic = <<Ipc as Service>::DynamicStorage<DynamicConfig> as NamedConceptMgmt>::Configuration::default()
            .prefix(&config.global.prefix)
            .suffix(&config.global.service.dynamic_config_storage_suffix)
            .path_hint(config.global.root_path())
            .path_for(&name)
            .to_string();
        let file = std::path::Path::new(&dynamic)
            .file_name()
            .ok_or_else(|| ContractError::Transport(format!("the dynamic segment {dynamic}")))?;
        let path = std::path::Path::new(POSIX_SHM_DIR).join(file);
        let inode = std::fs::metadata(&path)
            .map_err(|error| {
                ContractError::Transport(format!("the dynamic segment {}: {error}", path.display()))
            })?
            .ino();
        self.segments
            .push((factory.name().as_str().to_owned(), path, inode));
        Ok(factory)
    }

    /// The first watched service whose dynamic segment no longer exists, or
    /// exists as another file than the one opened.
    pub(crate) fn removed(&self) -> Option<&str> {
        self.segments
            .iter()
            .find(|(_, path, inode)| {
                std::fs::metadata(path).map_or(true, |metadata| metadata.ino() != *inode)
            })
            .map(|(name, _, _)| name.as_str())
    }
}

/// Give up `ports` without dropping them.
///
/// A participant that drops the last registration of a service removes the
/// service's files by name, and once the daemon has opened its services
/// again those names are the new services'. Kept registered, the abandoned
/// ports leave every other participant of the removed services a co-owner,
/// so no one removes anything by name on leaving them. What is kept is the
/// mapping of the removed segments and one node entry, once per removal.
pub(crate) fn abandon<T>(ports: T) {
    std::mem::forget(ports);
}
