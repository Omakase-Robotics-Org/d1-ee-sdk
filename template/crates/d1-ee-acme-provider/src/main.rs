//! `d1-ee-acme-provider`: serves an ACME G1 gripper on one arm side to
//! `d1-firmwared` (`end_effector.shm/1`).

use std::process::ExitCode;

fn main() -> ExitCode {
    d1_ee_acme_provider::app::main()
}
