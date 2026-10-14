# d1fw-canfd

Variable-length CANFD frames and the synchronous bus seam the dexterous
hands speak.

## Why this exists separately

The Damiao gripper's motors use classic 8-byte CAN frames, which
`d1fw-proto-dmcan` models as a fixed `[u8; 8]`. Both dexterous hands need
more room than that: one LinkerHand O30 joint-vector write is a 28-byte
payload, and a Leadshine DH116S feedback frame is 62 bytes.

They therefore get their own frame type rather than a widened gripper one.
The gripper's fixed-width frame is load-bearing inside its state machine,
and adding hand support is not allowed to change gripper behaviour.

## Surface

- `CanFdFrame` — one frame reduced to what the hand protocols need: a
  standard CAN identifier and a payload of at most `MAX_PAYLOAD` bytes.
- `fd_padded_length` / `CanFdFrame::padded` — CANFD payloads may only take
  the lengths in `DLC_LENGTHS`, so a frame is zero-padded up to the next
  legal size before it goes out.
- `CanFdBus` — the synchronous transport seam. `send_as` sends a frame with
  its `FrameKind` (motion, read or stop), which a gating transport (the
  flange relay's provider side) needs; a plain transport ignores it.
- `CanFdError` — frame validation, transport failures and a gating
  transport's refusal.

## Scope

This crate performs no I/O. `CanFdBus` is implemented by d1-firmwared's arm
end-module passthrough and its simulated flange channel, by the flange
relay's provider side (`d1fw-ee-hand`'s `FlangeChannel`), by the SDK's
recording channel (`d1fw-ee-harness`'s `Recorder`) and by every simulated
device a provider's tests put on a channel (`d1fw-ee-hand`'s `SimBus`).
