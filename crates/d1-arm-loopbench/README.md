# d1-arm-loopbench

A client of the D1 arm contracts that measures the native-rate command loop
end to end, against the simulated daemon or a robot.

It takes the arm lease over the daemon's REST port (`--rest`, `--holder`,
`--class`, or `--lease-token`), reads `arm_feedback.shm/1`, and streams
`arm_command.shm/1` at `--rate-hz` for `--duration-s`: a sine of
`--amplitude-deg` and `--period-s` on `--joints` of `--sides`, around the
pose the arms measure when it starts. It sleeps to just before each tick and
spins onto it, notifies the `command` event service after each command, and
drains `command_status`, the feedback and the events every tick. Then it
prints:

- commands sent and written per second, refusals by `RESULT_*` with the first
  message of each, superseded commands, lost statuses;
- the daemon-added latency (`sent_mono_ns - rx_mono_ns`), the
  client-to-written latency and the end-to-end latency (to the first
  feedback record whose `accepted_cmd` is the written target), p50, p99,
  max;
- the feedback rate and gaps, the highest `frame_miss_count` and
  `sys_cyc_miss_count` per arm, the rise of `max_frame_miss_count`, and the
  stream stops.

With `--feedback-only` it takes no lease and sends nothing: it spins on
`arm_feedback.shm/1` for `--duration-s` and reports the feedback rate, the
gaps in `seq`, the publish latency (from the daemon receiving a record to
this reader holding it) and the frame-miss counters, gated by
`--min-feedback-hz`, `--max-feedback-p99-us` and `--max-feedback-gaps`.

`--json <file>` writes the same report as JSON. The gates
`--min-rate-hz`, `--max-p99-us`, `--max-max-us`, `--max-frame-miss` and
`--max-refused` make it print `FAIL …` and exit 1 when one is not met
(`PASS` and 0 otherwise; 2 when it cannot run). It runs as the daemon's
account, and puts no arm in any mode: the arms must already act on joint
targets.
