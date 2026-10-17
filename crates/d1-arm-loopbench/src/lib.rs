//! `d1-arm-loopbench`: a client of the D1 arm contracts that measures the
//! native-rate command loop end to end.
//!
//! It takes the arm lease over the daemon's REST port, reads
//! `arm_feedback.shm/1`, and streams joint targets over `arm_command.shm/1`
//! at a chosen rate for a chosen time: a small sine around the pose the arms
//! hold when it starts, on the joints and arms it is told. It then prints
//! what the loop achieved:
//!
//! - the rate of commands sent and of commands the daemon wrote;
//! - the daemon-added latency, from the daemon taking a command to its write
//!   to the arm controller returning (`command_status.sent_mono_ns -
//!   rx_mono_ns`), p50, p99 and maximum;
//! - the client-to-written latency (`sent_mono_ns - client_mono_ns`), and the
//!   end-to-end latency to the first feedback record whose accepted joint
//!   command is the written target;
//! - the controller's frame-miss counters (`frame_miss_count` and the rise of
//!   `max_frame_miss_count` over the run) and the feedback rate and gaps;
//! - every refusal, by reason;
//! - the daemon's own record of the path, when the daemon offers it
//!   (`GET /v1/arm/command_path`, d1-firmwared 0.12 and later): where the
//!   time of each command written in the last second went inside the daemon
//!   (per stage, p50 / p99 / max), and the controller's uptake of the writes
//!   (the echo lag, the consumption and feedback rates, the targets in
//!   flight, saturation and pacing).
//!
//! With `--feedback-only` it only reads `arm_feedback.shm/1`, spinning on
//! it, and reports the feedback rate, its gaps and its publish latency.
//!
//! The same binary runs against the simulated daemon (`d1-firmwared run
//! --backend sim`) and against a robot. With the `--max-*`/`--min-*` gates
//! it exits 1 when one is not met. It runs as the daemon's account, which
//! owns the shared memory.

mod http;

use std::{
    collections::{BTreeMap, VecDeque},
    process::ExitCode,
    sync::{
        Arc,
        atomic::{AtomicBool, Ordering},
    },
    time::{Duration, Instant},
};

use clap::Parser;
use d1fw_arm_shm_contract::{
    JOINTS, SIDES,
    model::{self, new_instance_id},
    names::{self, ArmNames},
    ports,
    root::ShmRoot,
    wire::{self, Command, CommandStatus, Event, Feedback},
};
use iceoryx2::prelude::{NodeBuilder, NodeName, ipc};
use serde_json::{Value, json};

/// The command line.
#[derive(Debug, Clone, Parser)]
#[command(
    version,
    about = "Stream joint commands over arm_command.shm/1 and report the achieved rate, latency \
             and the controller's missed frames"
)]
pub struct Args {
    /// Commands per second.
    #[arg(long, default_value_t = 200.0)]
    pub rate_hz: f64,
    /// How long to stream, seconds.
    #[arg(long, default_value_t = 60.0)]
    pub duration_s: f64,
    /// Which arms to command: `ab`, `a` or `b`.
    #[arg(long, default_value = "ab")]
    pub sides: String,
    /// The joints the sine moves, 1-based, comma-separated; the others hold.
    #[arg(long, default_value = "1,2,3,4,5,6,7")]
    pub joints: String,
    /// The sine's amplitude, degrees.
    #[arg(long, default_value_t = 1.0)]
    pub amplitude_deg: f64,
    /// The sine's period, seconds.
    #[arg(long, default_value_t = 4.0)]
    pub period_s: f64,
    /// The daemon's REST base URL, for the lease.
    #[arg(long, default_value = "http://127.0.0.1:4750")]
    pub rest: String,
    /// The lease holder's name.
    #[arg(long, default_value = "d1-arm-loopbench")]
    pub holder: String,
    /// The lease class: `ambient`, `policy` or `operator`.
    #[arg(long, default_value = "operator")]
    pub class: String,
    /// A lease token to use instead of taking the lease over REST.
    #[arg(long)]
    pub lease_token: Option<String>,
    /// The shared-memory root.
    #[arg(long, default_value = d1fw_arm_shm_contract::root::DEFAULT_ROOT_PATH)]
    pub root: String,
    /// The shared-memory prefix.
    #[arg(long, default_value = d1fw_arm_shm_contract::root::DEFAULT_PREFIX)]
    pub prefix: String,
    /// The base of the arm service names.
    #[arg(long, default_value = names::DEFAULT_BASE)]
    pub base: String,
    /// Each command's age limit, milliseconds (0: the contract's default).
    #[arg(long, default_value_t = 0)]
    pub max_age_ms: u64,
    /// Write the report as JSON to this file too.
    #[arg(long)]
    pub json: Option<std::path::PathBuf>,
    /// Gate: the lowest acceptable rate of written commands, per second.
    #[arg(long)]
    pub min_rate_hz: Option<f64>,
    /// Gate: the highest acceptable daemon-added latency p99, microseconds.
    #[arg(long)]
    pub max_p99_us: Option<u64>,
    /// Gate: the highest acceptable daemon-added latency, microseconds.
    #[arg(long)]
    pub max_max_us: Option<u64>,
    /// Gate: the highest acceptable `frame_miss_count` seen on any arm.
    #[arg(long)]
    pub max_frame_miss: Option<i64>,
    /// Gate: the most refused commands acceptable.
    #[arg(long)]
    pub max_refused: Option<u64>,
    /// Only read `arm_feedback.shm/1` for `--duration-s`, spinning on it,
    /// and report its rate, gaps and publish latency: no lease, no command.
    #[arg(long)]
    pub feedback_only: bool,
    /// Gate: the lowest acceptable feedback rate, records per second.
    #[arg(long)]
    pub min_feedback_hz: Option<f64>,
    /// Gate: the highest acceptable feedback publish latency p99 (from the
    /// daemon receiving a record to this reader holding it), microseconds.
    #[arg(long)]
    pub max_feedback_p99_us: Option<u64>,
    /// Gate: the most feedback records lost (gaps in `seq`).
    #[arg(long)]
    pub max_feedback_gaps: Option<u64>,
}

/// Run the bench with the process's command line and print the report.
#[must_use]
pub fn main() -> ExitCode {
    let args = Args::parse();
    match run(&args) {
        Ok(report) => {
            print!("{}", report.text());
            if let Some(path) = &args.json
                && let Err(error) = std::fs::write(path, format!("{:#}\n", report.json()))
            {
                eprintln!("d1-arm-loopbench: write {}: {error}", path.display());
                return ExitCode::from(2);
            }
            let failures = report.gate_failures(&args);
            for failure in &failures {
                println!("FAIL {failure}");
            }
            if failures.is_empty() {
                println!("PASS");
                ExitCode::SUCCESS
            } else {
                ExitCode::from(1)
            }
        }
        Err(error) => {
            eprintln!("d1-arm-loopbench: {error}");
            ExitCode::from(2)
        }
    }
}

/// Percentiles of a sample of microsecond figures.
#[derive(Debug, Clone, Copy, Default, PartialEq)]
pub struct Spread {
    /// How many samples.
    pub count: usize,
    /// Median, µs.
    pub p50: u64,
    /// 99th percentile, µs.
    pub p99: u64,
    /// Maximum, µs.
    pub max: u64,
}

impl Spread {
    fn of(mut values: Vec<u64>) -> Self {
        if values.is_empty() {
            return Self::default();
        }
        values.sort_unstable();
        let at = |q: f64| values[((values.len() - 1) as f64 * q).round() as usize];
        Self {
            count: values.len(),
            p50: at(0.50),
            p99: at(0.99),
            max: *values.last().expect("not empty"),
        }
    }

    fn json(self) -> Value {
        json!({"count": self.count, "p50_us": self.p50, "p99_us": self.p99, "max_us": self.max})
    }
}

/// What one run measured.
#[derive(Debug, Clone, Default)]
pub struct Report {
    /// Seconds streamed.
    pub elapsed_s: f64,
    /// The requested rate.
    pub target_hz: f64,
    /// Commands published.
    pub sent: u64,
    /// Commands the daemon reported written.
    pub accepted: u64,
    /// Commands the daemon reported refused, by `RESULT_*` code.
    pub refused: BTreeMap<u32, u64>,
    /// The first refusal's message for each code.
    pub refusal_messages: BTreeMap<u32, String>,
    /// Commands the daemon dropped because a newer one had arrived.
    pub superseded: u64,
    /// Command statuses this client did not receive (gaps in their `seq`).
    pub statuses_lost: u64,
    /// Daemon-added latency: taken to written.
    pub daemon: Spread,
    /// Client publish to written.
    pub client_to_written: Spread,
    /// Client publish to the first feedback record carrying the target.
    pub end_to_end: Spread,
    /// Feedback records received, and the gaps in their `seq`.
    pub feedback_records: u64,
    /// Feedback records lost.
    pub feedback_gaps: u64,
    /// Highest `frame_miss_count` seen per arm.
    pub frame_miss_max: [i64; SIDES],
    /// Rise of `max_frame_miss_count` over the run per arm.
    pub frame_miss_high_water_rise: [i64; SIDES],
    /// Highest `sys_cyc_miss_count` seen per arm.
    pub sys_cyc_miss_max: [i64; SIDES],
    /// `EVENT_STREAM_STOPPED` events seen.
    pub stream_stops: u64,
    /// Feedback publish latency: the daemon receiving a record to this
    /// reader holding it (measured with `--feedback-only`, where the reader
    /// spins).
    pub feedback_latency: Spread,
    /// The daemon's record of the path (`GET /v1/arm/command_path`), read
    /// after the stream; `None` when the daemon does not offer it or the
    /// REST port was not asked.
    pub daemon_command_path: Option<Value>,
}

impl Report {
    /// Written commands per second.
    #[must_use]
    pub fn achieved_hz(&self) -> f64 {
        if self.elapsed_s > 0.0 {
            self.accepted as f64 / self.elapsed_s
        } else {
            0.0
        }
    }

    fn refused_total(&self) -> u64 {
        self.refused.values().sum()
    }

    /// The report as text, one figure per line.
    #[must_use]
    pub fn text(&self) -> String {
        let mut out = String::new();
        let line = |out: &mut String, text: String| {
            out.push_str(&text);
            out.push('\n');
        };
        line(
            &mut out,
            format!(
                "streamed {:.1} s at a target of {:.1} Hz: sent {} ({:.1}/s), written {} ({:.1}/s), refused {}, superseded {}, statuses lost {}",
                self.elapsed_s,
                self.target_hz,
                self.sent,
                self.sent as f64 / self.elapsed_s.max(1e-9),
                self.accepted,
                self.achieved_hz(),
                self.refused_total(),
                self.superseded,
                self.statuses_lost,
            ),
        );
        for (code, count) in &self.refused {
            line(
                &mut out,
                format!(
                    "  refused with result {code}: {count} (first: {})",
                    self.refusal_messages.get(code).map_or("", String::as_str)
                ),
            );
        }
        for (name, spread) in [
            ("daemon-added latency (taken -> written)", self.daemon),
            ("client publish -> written", self.client_to_written),
            (
                "client publish -> feedback carries the target",
                self.end_to_end,
            ),
        ] {
            line(
                &mut out,
                format!(
                    "{name}: p50 {} us, p99 {} us, max {} us ({} samples)",
                    spread.p50, spread.p99, spread.max, spread.count
                ),
            );
        }
        line(
            &mut out,
            format!(
                "feedback: {} records ({:.1}/s), {} lost; frame_miss_count max a={} b={}; max_frame_miss_count rise a={} b={}; sys_cyc_miss_count max a={} b={}; stream stops {}",
                self.feedback_records,
                self.feedback_records as f64 / self.elapsed_s.max(1e-9),
                self.feedback_gaps,
                self.frame_miss_max[0],
                self.frame_miss_max[1],
                self.frame_miss_high_water_rise[0],
                self.frame_miss_high_water_rise[1],
                self.sys_cyc_miss_max[0],
                self.sys_cyc_miss_max[1],
                self.stream_stops,
            ),
        );
        if self.feedback_latency.count > 0 {
            line(
                &mut out,
                format!(
                    "feedback publish latency (received by the daemon -> held by this reader): p50 {} us, p99 {} us, max {} us ({} samples)",
                    self.feedback_latency.p50,
                    self.feedback_latency.p99,
                    self.feedback_latency.max,
                    self.feedback_latency.count
                ),
            );
        }
        if let Some(record) = &self.daemon_command_path {
            for text in daemon_record_lines(record) {
                line(&mut out, text);
            }
        }
        out
    }

    /// The report as JSON.
    #[must_use]
    pub fn json(&self) -> Value {
        json!({
            "elapsed_s": self.elapsed_s,
            "target_hz": self.target_hz,
            "sent": self.sent,
            "accepted": self.accepted,
            "achieved_hz": self.achieved_hz(),
            "refused": self.refused.iter().map(|(code, count)| (code.to_string(), json!(count))).collect::<serde_json::Map<_, _>>(),
            "refusal_messages": self.refusal_messages.iter().map(|(code, text)| (code.to_string(), json!(text))).collect::<serde_json::Map<_, _>>(),
            "superseded": self.superseded,
            "statuses_lost": self.statuses_lost,
            "daemon_added_latency": self.daemon.json(),
            "client_to_written_latency": self.client_to_written.json(),
            "end_to_end_latency": self.end_to_end.json(),
            "feedback_records": self.feedback_records,
            "feedback_gaps": self.feedback_gaps,
            "frame_miss_max": self.frame_miss_max,
            "frame_miss_high_water_rise": self.frame_miss_high_water_rise,
            "sys_cyc_miss_max": self.sys_cyc_miss_max,
            "stream_stops": self.stream_stops,
            "feedback_latency": self.feedback_latency.json(),
            "daemon_command_path": self.daemon_command_path.clone().unwrap_or(Value::Null),
        })
    }

    /// The gates `args` set that this report does not meet.
    #[must_use]
    pub fn gate_failures(&self, args: &Args) -> Vec<String> {
        let mut failures = Vec::new();
        if let Some(min) = args.min_rate_hz
            && self.achieved_hz() < min
        {
            failures.push(format!("written {:.1}/s < {min}/s", self.achieved_hz()));
        }
        if let Some(max) = args.max_p99_us
            && self.daemon.p99 > max
        {
            failures.push(format!(
                "daemon-added p99 {} us > {max} us",
                self.daemon.p99
            ));
        }
        if let Some(max) = args.max_max_us
            && self.daemon.max > max
        {
            failures.push(format!(
                "daemon-added max {} us > {max} us",
                self.daemon.max
            ));
        }
        if let Some(max) = args.max_frame_miss
            && self.frame_miss_max.iter().any(|seen| *seen > max)
        {
            failures.push(format!(
                "frame_miss_count reached {:?} > {max}",
                self.frame_miss_max
            ));
        }
        if let Some(max) = args.max_refused
            && self.refused_total() > max
        {
            failures.push(format!("{} commands refused > {max}", self.refused_total()));
        }
        let feedback_hz = self.feedback_records as f64 / self.elapsed_s.max(1e-9);
        if let Some(min) = args.min_feedback_hz
            && feedback_hz < min
        {
            failures.push(format!("feedback {feedback_hz:.1}/s < {min}/s"));
        }
        if let Some(max) = args.max_feedback_p99_us
            && self.feedback_latency.p99 > max
        {
            failures.push(format!(
                "feedback publish latency p99 {} us > {max} us",
                self.feedback_latency.p99
            ));
        }
        if let Some(max) = args.max_feedback_gaps
            && self.feedback_gaps > max
        {
            failures.push(format!(
                "{} feedback records lost > {max}",
                self.feedback_gaps
            ));
        }
        if !args.feedback_only && self.daemon.count == 0 {
            failures.push("no command was written".to_owned());
        }
        failures
    }
}

/// The stage names of the daemon's record, in path order, and `total`.
const DAEMON_STAGES: [&str; 8] = [
    "contract",
    "rate",
    "lease",
    "facade_entry",
    "guard",
    "backend",
    "written",
    "total",
];

/// The daemon's record as report lines: the stage breakdown of the last
/// second and the controller's uptake.
fn daemon_record_lines(record: &Value) -> Vec<String> {
    let stages = &record["stages"];
    let stream = &record["stream"];
    let spread = |stage: &str| {
        let spread = &stages[stage];
        format!(
            "{stage} {}/{}/{}",
            spread["p50_us"], spread["p99_us"], spread["max_us"]
        )
    };
    vec![
        format!(
            "daemon stages over the last {:.2} s ({} commands, p50/p99/max us): {}",
            stages["window_s"].as_f64().unwrap_or(0.0),
            stages["count"],
            DAEMON_STAGES
                .iter()
                .map(|stage| spread(stage))
                .collect::<Vec<_>>()
                .join(", ")
        ),
        format!(
            "daemon stream: echo lag {} us, consumption {:.1}/s, advance {:.1}/s, feedback {:.1}/s, in flight {}, saturated {}, paced {}, onsets {}, superseded {} (paced {})",
            stream["echo_lag_us"],
            stream["consumption_hz"].as_f64().unwrap_or(0.0),
            stream["advance_hz"].as_f64().unwrap_or(0.0),
            stream["feedback_hz"].as_f64().unwrap_or(0.0),
            stream["in_flight"],
            stream["saturated"],
            stream["paced"],
            stream["saturation_onsets"],
            record["superseded"],
            stream["paced_superseded"],
        ),
    ]
}

/// The daemon's record of the path, or `None` when the daemon does not
/// offer the route (a daemon before it) or does not answer.
fn daemon_command_path(rest: &str) -> Option<Value> {
    match http::request(rest, "GET", "/v1/arm/command_path", &Value::Null) {
        Ok((200, body)) => body.get("data").cloned().filter(|data| data.is_object()),
        _ => None,
    }
}

fn parse_sides(text: &str) -> Result<[bool; SIDES], String> {
    match text {
        "ab" | "ba" => Ok([true, true]),
        "a" => Ok([true, false]),
        "b" => Ok([false, true]),
        other => Err(format!("--sides {other:?}: give ab, a or b")),
    }
}

fn parse_joints(text: &str) -> Result<[bool; JOINTS], String> {
    let mut joints = [false; JOINTS];
    for item in text
        .split(',')
        .map(str::trim)
        .filter(|item| !item.is_empty())
    {
        let joint: usize = item
            .parse()
            .map_err(|_| format!("--joints: {item:?} is not a joint number"))?;
        if !(1..=JOINTS).contains(&joint) {
            return Err(format!("--joints: {joint} is not 1 to {JOINTS}"));
        }
        joints[joint - 1] = true;
    }
    Ok(joints)
}

/// The lease the bench holds, renewed by a thread of its own until dropped.
struct Lease {
    token: u64,
    stop: Arc<AtomicBool>,
    renewer: Option<std::thread::JoinHandle<()>>,
    release: Option<(String, String)>,
}

impl Lease {
    fn take(args: &Args) -> Result<Self, String> {
        if let Some(token) = &args.lease_token {
            return Ok(Self {
                token: model::parse_lease_token(token).map_err(|error| error.to_string())?,
                stop: Arc::new(AtomicBool::new(false)),
                renewer: None,
                release: None,
            });
        }
        let request = json!({"holder": args.holder, "ttl_s": 30, "class": args.class});
        let (status, body) = http::request(&args.rest, "POST", "/v1/arm/lease", &request)?;
        let token = body["data"]["token"]
            .as_str()
            .ok_or_else(|| format!("POST /v1/arm/lease answered {status}: {body}"))?;
        let token = model::parse_lease_token(token).map_err(|error| error.to_string())?;
        let stop = Arc::new(AtomicBool::new(false));
        let renewer = {
            let stop = Arc::clone(&stop);
            let rest = args.rest.clone();
            std::thread::spawn(move || {
                let mut last = Instant::now();
                while !stop.load(Ordering::Acquire) {
                    std::thread::sleep(Duration::from_millis(100));
                    if last.elapsed() >= Duration::from_secs(10) {
                        last = Instant::now();
                        if let Err(error) = http::request(&rest, "POST", "/v1/arm/lease", &request)
                        {
                            eprintln!("d1-arm-loopbench: lease renewal failed: {error}");
                        }
                    }
                }
            })
        };
        Ok(Self {
            token,
            stop,
            renewer: Some(renewer),
            release: Some((args.rest.clone(), args.holder.clone())),
        })
    }
}

impl Drop for Lease {
    fn drop(&mut self) {
        self.stop.store(true, Ordering::Release);
        if let Some(renewer) = self.renewer.take() {
            let _ = renewer.join();
        }
        if let Some((rest, holder)) = self.release.take() {
            let _ = http::request(&rest, "DELETE", "/v1/arm/lease", &json!({"holder": holder}));
        }
    }
}

/// A written command waiting for the feedback record that carries it.
struct Pending {
    target: [[f32; JOINTS]; SIDES],
    sides: [bool; SIDES],
    client_mono_ns: u64,
}

fn carries(record: &Feedback, pending: &Pending) -> bool {
    (0..SIDES).filter(|side| pending.sides[*side]).all(|side| {
        record.sides[side]
            .accepted_cmd
            .iter()
            .zip(pending.target[side])
            .all(|(held, sent)| (held - sent).abs() <= 1e-3)
    })
}

/// Stream for `args.duration_s` and measure.
///
/// # Errors
///
/// A bad argument, a lease that cannot be taken, shared memory that cannot
/// be opened, or no feedback within 5 s.
#[allow(clippy::too_many_lines)]
pub fn run(args: &Args) -> Result<Report, String> {
    if !(args.rate_hz > 0.0 && args.rate_hz <= 10_000.0) {
        return Err(format!("--rate-hz {} is not in (0, 10000]", args.rate_hz));
    }
    let sides = parse_sides(&args.sides)?;
    let joints = parse_joints(&args.joints)?;
    let root = ShmRoot::new(&args.root, &args.prefix).map_err(|error| error.to_string())?;
    let names = ArmNames::new(&args.base).map_err(|error| error.to_string())?;
    let node = NodeBuilder::new()
        .config(&root.config().map_err(|error| error.to_string())?)
        .name(&NodeName::new("d1-arm-loopbench").map_err(|error| format!("{error:?}"))?)
        .create::<ipc::Service>()
        .map_err(|error| format!("create the iceoryx2 node under {}: {error:?}", args.root))?;
    let contract = |error: model::ContractError| error.to_string();
    let feedback = ports::open_topic::<ipc::Service, Feedback>(
        &node,
        &names.name(names::FEEDBACK),
        ports::FEEDBACK_TOPIC,
    )
    .map_err(contract)?
    .subscriber_builder()
    .create()
    .map_err(|error| format!("feedback subscriber: {error:?}"))?;
    let statuses = ports::open_topic::<ipc::Service, CommandStatus>(
        &node,
        &names.name(names::COMMAND_STATUS),
        ports::COMMAND_STATUS_TOPIC,
    )
    .map_err(contract)?
    .subscriber_builder()
    .create()
    .map_err(|error| format!("command status subscriber: {error:?}"))?;
    let events = ports::open_topic::<ipc::Service, Event>(
        &node,
        &names.name(names::EVENTS),
        ports::EVENTS_TOPIC,
    )
    .map_err(contract)?
    .subscriber_builder()
    .create()
    .map_err(|error| format!("events subscriber: {error:?}"))?;
    let commands = ports::open_topic::<ipc::Service, Command>(
        &node,
        &names.name(names::COMMAND),
        ports::COMMAND_TOPIC,
    )
    .map_err(contract)?
    .publisher_builder()
    .create()
    .map_err(|error| format!("command publisher: {error:?}"))?;
    let notifier = ports::command_notifier(&node, &names.name(names::COMMAND)).map_err(contract)?;

    // The pose to move around: what the arms measure now.
    let deadline = Instant::now() + Duration::from_secs(5);
    let first = loop {
        if let Some(sample) = feedback
            .receive()
            .map_err(|error| format!("feedback: {error:?}"))?
        {
            break *sample;
        }
        if Instant::now() > deadline {
            return Err(
                "no arm_feedback.shm/1 record within 5 s: is the daemon running with \
                 [arm] shm_publish, under this root and prefix, as this account?"
                    .to_owned(),
            );
        }
        std::thread::sleep(Duration::from_millis(1));
    };
    let centre: [[f64; JOINTS]; SIDES] =
        std::array::from_fn(|side| first.sides[side].pos.map(f64::from));
    let initial_high_water: [i64; SIDES] =
        std::array::from_fn(|side| i64::from(first.sides[side].max_frame_miss_count));

    if args.feedback_only {
        return Ok(read_feedback(args, &feedback, &first, initial_high_water));
    }
    let lease = Lease::take(args)?;
    // Events before this point are not this run's.
    while events
        .receive()
        .map_err(|error| format!("{error:?}"))?
        .is_some()
    {}
    while statuses
        .receive()
        .map_err(|error| format!("{error:?}"))?
        .is_some()
    {}

    let instance_id = new_instance_id();
    let period = Duration::from_secs_f64(1.0 / args.rate_hz);
    let duration = Duration::from_secs_f64(args.duration_s);
    let mut report = Report {
        target_hz: args.rate_hz,
        ..Report::default()
    };
    let mut daemon = Vec::new();
    let mut client_to_written = Vec::new();
    let mut end_to_end = Vec::new();
    let mut pending: VecDeque<Pending> = VecDeque::new();
    let mut last_feedback_seq = first.header.seq;
    let mut last_status_seq: Option<u64> = None;
    let mut high_water = initial_high_water;
    let mask = (if sides[0] { wire::SIDE_MASK_A } else { 0 })
        | (if sides[1] { wire::SIDE_MASK_B } else { 0 });

    let mut drain = |report: &mut Report, pending: &mut VecDeque<Pending>| -> Result<(), String> {
        while let Some(status) = statuses
            .receive()
            .map_err(|error| format!("command status: {error:?}"))?
        {
            if status.client_instance_id != instance_id {
                continue;
            }
            if let Some(last) = last_status_seq {
                report.statuses_lost += status.header.seq.saturating_sub(last + 1);
            }
            last_status_seq = Some(status.header.seq);
            report.superseded = status.superseded;
            if status.result == wire::RESULT_ACCEPTED {
                report.accepted += 1;
                daemon.push(status.sent_mono_ns.saturating_sub(status.rx_mono_ns) / 1_000);
                client_to_written
                    .push(status.sent_mono_ns.saturating_sub(status.client_mono_ns) / 1_000);
                pending.push_back(Pending {
                    target: status
                        .written_deg
                        .map(|side| side.map(|value| value as f32)),
                    sides,
                    client_mono_ns: status.client_mono_ns,
                });
                if pending.len() > 1_000 {
                    pending.pop_front();
                }
            } else {
                *report.refused.entry(status.result).or_default() += 1;
                report
                    .refusal_messages
                    .entry(status.result)
                    .or_insert_with(|| wire::read_text(&status.message).unwrap_or("").to_owned());
            }
        }
        while let Some(record) = feedback
            .receive()
            .map_err(|error| format!("feedback: {error:?}"))?
        {
            report.feedback_records += 1;
            report.feedback_gaps += record.header.seq.saturating_sub(last_feedback_seq + 1);
            last_feedback_seq = record.header.seq;
            for (side, arm) in record.sides.iter().enumerate() {
                report.frame_miss_max[side] =
                    report.frame_miss_max[side].max(i64::from(arm.frame_miss_count));
                report.sys_cyc_miss_max[side] =
                    report.sys_cyc_miss_max[side].max(i64::from(arm.sys_cyc_miss_count));
                high_water[side] = high_water[side].max(i64::from(arm.max_frame_miss_count));
            }
            if let Some(found) = pending.iter().position(|entry| carries(&record, entry)) {
                let entry = &pending[found];
                end_to_end.push(record.rx_mono_ns.saturating_sub(entry.client_mono_ns) / 1_000);
                pending.drain(..=found);
            }
        }
        while let Some(event) = events.receive().map_err(|error| format!("{error:?}"))? {
            if event.kind == wire::EVENT_STREAM_STOPPED {
                report.stream_stops += 1;
            }
        }
        Ok(())
    };

    let started = Instant::now();
    let mut next = started;
    let mut seq = 0_u64;
    while started.elapsed() < duration {
        // Sleep to just before the tick, then spin onto it.
        let now = Instant::now();
        if next > now {
            let wait = next - now;
            if wait > Duration::from_micros(300) {
                std::thread::sleep(wait - Duration::from_micros(200));
            }
            while Instant::now() < next {
                std::hint::spin_loop();
            }
        }
        let t = next.duration_since(started).as_secs_f64();
        let offset = args.amplitude_deg * (2.0 * std::f64::consts::PI * t / args.period_s).sin();
        seq += 1;
        let mut command = Command::zeroed();
        command.header = model::header(instance_id, seq);
        command.lease_token = lease.token;
        command.max_age_ns = args.max_age_ms * 1_000_000;
        command.sides = mask;
        command.target_deg = std::array::from_fn(|side| {
            std::array::from_fn(|joint| {
                centre[side][joint] + if joints[joint] { offset } else { 0.0 }
            })
        });
        commands
            .send_copy(command)
            .map_err(|error| format!("send a command: {error:?}"))?;
        let _ = notifier.notify();
        report.sent += 1;
        drain(&mut report, &mut pending)?;
        next += period;
        // A tick missed by more than a period is skipped, not repaid.
        if Instant::now() > next + period {
            next = Instant::now();
        }
    }
    report.elapsed_s = started.elapsed().as_secs_f64();
    let settle = Instant::now() + Duration::from_millis(200);
    while Instant::now() < settle {
        drain(&mut report, &mut pending)?;
        std::thread::sleep(Duration::from_millis(1));
    }
    drop(lease);
    report.daemon_command_path = daemon_command_path(&args.rest);
    report.daemon = Spread::of(daemon);
    report.client_to_written = Spread::of(client_to_written);
    report.end_to_end = Spread::of(end_to_end);
    report.frame_miss_high_water_rise =
        std::array::from_fn(|side| high_water[side] - initial_high_water[side]);
    Ok(report)
}

/// `--feedback-only`: spin on the feedback for `args.duration_s`.
fn read_feedback(
    args: &Args,
    feedback: &iceoryx2::port::subscriber::Subscriber<ipc::Service, Feedback, ()>,
    first: &Feedback,
    initial_high_water: [i64; SIDES],
) -> Report {
    let mut report = Report::default();
    let mut latencies = Vec::new();
    let mut last_seq = first.header.seq;
    let mut high_water = initial_high_water;
    let duration = Duration::from_secs_f64(args.duration_s);
    let started = Instant::now();
    while started.elapsed() < duration {
        match feedback.receive() {
            Ok(Some(record)) => {
                let now = d1fw_arm_shm_contract::clock::monotonic_ns();
                latencies.push(now.saturating_sub(record.rx_mono_ns) / 1_000);
                report.feedback_records += 1;
                report.feedback_gaps += record.header.seq.saturating_sub(last_seq + 1);
                last_seq = record.header.seq;
                for (side, arm) in record.sides.iter().enumerate() {
                    report.frame_miss_max[side] =
                        report.frame_miss_max[side].max(i64::from(arm.frame_miss_count));
                    report.sys_cyc_miss_max[side] =
                        report.sys_cyc_miss_max[side].max(i64::from(arm.sys_cyc_miss_count));
                    high_water[side] = high_water[side].max(i64::from(arm.max_frame_miss_count));
                }
            }
            Ok(None) => std::thread::yield_now(),
            Err(_) => break,
        }
    }
    report.elapsed_s = started.elapsed().as_secs_f64();
    report.feedback_latency = Spread::of(latencies);
    report.frame_miss_high_water_rise =
        std::array::from_fn(|side| high_water[side] - initial_high_water[side]);
    report
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn spreads_are_nearest_rank() {
        let spread = Spread::of((1..=100).collect());
        assert_eq!((spread.p50, spread.p99, spread.max), (51, 99, 100));
        assert_eq!(Spread::of(Vec::new()), Spread::default());
    }

    #[test]
    fn sides_and_joints_parse() {
        assert_eq!(parse_sides("ab").unwrap(), [true, true]);
        assert_eq!(parse_sides("b").unwrap(), [false, true]);
        assert!(parse_sides("c").is_err());
        assert_eq!(
            parse_joints("1, 7").unwrap(),
            [true, false, false, false, false, false, true]
        );
        assert!(parse_joints("8").is_err());
    }

    #[test]
    fn the_daemons_record_is_reported_when_present() {
        let record = json!({
            "superseded": 4,
            "stream": {"echo_lag_us": 1394, "consumption_hz": 921.6, "advance_hz": 718.0,
                       "feedback_hz": 925.9, "in_flight": 0, "saturated": false, "paced": true,
                       "saturation_onsets": 2, "paced_superseded": 3},
            "stages": {"window_s": 1.0, "count": 812,
                       "contract": {"p50_us": 2, "p99_us": 3, "max_us": 3},
                       "rate": {"p50_us": 0, "p99_us": 0, "max_us": 0},
                       "lease": {"p50_us": 2, "p99_us": 2, "max_us": 2},
                       "facade_entry": {"p50_us": 30, "p99_us": 32, "max_us": 34},
                       "guard": {"p50_us": 289, "p99_us": 348, "max_us": 916},
                       "backend": {"p50_us": 24, "p99_us": 27, "max_us": 48},
                       "written": {"p50_us": 5, "p99_us": 6, "max_us": 13},
                       "total": {"p50_us": 352, "p99_us": 399, "max_us": 1013}}
        });
        let report = Report {
            daemon_command_path: Some(record.clone()),
            ..Report::default()
        };
        let text = report.text();
        assert!(text.contains("guard 289/348/916"), "{text}");
        assert!(text.contains("total 352/399/1013"), "{text}");
        assert!(
            text.contains("echo lag 1394 us, consumption 921.6/s"),
            "{text}"
        );
        assert!(text.contains("onsets 2, superseded 4 (paced 3)"), "{text}");
        assert_eq!(report.json()["daemon_command_path"], record);
        assert!(Report::default().json()["daemon_command_path"].is_null());
        assert!(!Report::default().text().contains("daemon stages"));
    }

    #[test]
    fn gates_fail_by_name() {
        let report = Report {
            elapsed_s: 10.0,
            accepted: 1_990,
            daemon: Spread {
                count: 1_990,
                p50: 50,
                p99: 600,
                max: 900,
            },
            frame_miss_max: [3, 25],
            ..Report::default()
        };
        let args = Args::parse_from([
            "d1-arm-loopbench",
            "--min-rate-hz",
            "199",
            "--max-p99-us",
            "500",
            "--max-max-us",
            "2000",
            "--max-frame-miss",
            "19",
            "--max-refused",
            "0",
        ]);
        let failures = report.gate_failures(&args);
        assert_eq!(failures.len(), 2, "{failures:?}");
        assert!(failures[0].contains("p99 600"));
        assert!(failures[1].contains("frame_miss_count"));
    }
}
