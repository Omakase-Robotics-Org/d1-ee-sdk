//! A provider process under test.

use std::process::{Child, Command, ExitStatus, Stdio};

use d1fw_ee_shm_contract::root::ShmRoot;

/// A provider process, killed with SIGKILL when dropped.
#[derive(Debug)]
pub struct ProviderProcess(Child);

impl ProviderProcess {
    /// Start `command` with `--root` and `--prefix` of `root` appended, its
    /// standard output discarded and its standard error inherited.
    ///
    /// # Panics
    ///
    /// When the command cannot be started.
    #[must_use]
    pub fn start(mut command: Command, root: &ShmRoot) -> Self {
        command
            .args(["--root", root.root_path(), "--prefix", root.prefix()])
            .stdout(Stdio::null())
            .stderr(Stdio::inherit());
        let child = command
            .spawn()
            .unwrap_or_else(|error| panic!("start {command:?}: {error}"));
        Self(child)
    }

    /// The process id.
    #[must_use]
    pub fn id(&self) -> u32 {
        self.0.id()
    }

    /// SIGKILL, and reap.
    pub fn kill(&mut self) {
        let _ = self.0.kill();
        let _ = self.0.wait();
    }

    /// SIGTERM, and the exit status.
    ///
    /// # Panics
    ///
    /// When `kill` cannot be run or the process cannot be waited for.
    pub fn terminate(mut self) -> ExitStatus {
        let pid = self.0.id().to_string();
        let sent = Command::new("kill")
            .args(["-TERM", &pid])
            .status()
            .expect("run kill");
        assert!(sent.success(), "kill -TERM {pid}");
        let status = self.0.wait().expect("wait for the provider");
        std::mem::forget(self);
        status
    }
}

impl Drop for ProviderProcess {
    fn drop(&mut self) {
        self.kill();
    }
}
