# d1fw-ee-manifest

The provider manifest `omakase-ee-provider/1`, which describes one release
of an end-effector provider to the robot that runs it:

- `SCHEMA_JSON`: the JSON Schema (draft 2020-12) committed as
  `schema/omakase-ee-provider-1.schema.json`;
- `ProviderManifest`: the document as a Rust type. `parse` checks the
  schema, then the rules a schema cannot state: the `sdk.d1_ee_sdk` range
  holds `conformance.version`; `{side}` is the only placeholder of
  `launch.command` and `launch.config_file` and appears in both; the program
  is a file under `/opt/<package>/bin/`; a `registry` package's key names
  this release's `.deb`; each posture has one value per joint; models are
  distinct; `config_schema` is a self-contained draft 2020-12 schema. A
  provider that reads no configuration file names none of
  `launch.config_dir`, `launch.config_file` and `config_schema`;
- `version_range`: the range grammar of `sdk.d1_ee_sdk` and a fit's
  `provider.version` (comparators `>=`, `>`, `<=`, `<`, `=` or none for `=`,
  plain `MAJOR.MINOR.PATCH` operands, joined by commas; `RANGE_PATTERN`), and
  `COMMAND_PATTERN`, the expression of `launch.command`; d1-provisioning's
  reader uses both verbatim;
- `reader_warnings`: valid JSON that d1-provisioning's on-robot reader
  refuses (`\u`, `\b`, `\f` escapes; empty or dotted keys; a key twice);
- `ProviderManifest::check_descriptor`: the manifest against the descriptor
  a provider announced (`GET /v1/end_effectors/{side}`, or a bare
  descriptor): the model is listed, the driver and the joint count agree, the
  guard envelope id is listed, and each posture resolves to joint targets;
- the command `d1fw-ee-manifest`:

  ```sh
  d1fw-ee-manifest check manifest.json [--descriptor descriptor.json]
  d1fw-ee-manifest schema
  ```

  `check` exits 0 for a valid manifest (and an agreeing descriptor), 1 for
  an invalid one or a disagreeing descriptor, 2 for a file it cannot read;
  a reader warning is printed and leaves the status as it is.

The crate needs no network and does not link the shared-memory transport:
it runs on a developer's machine, in a release job and on a robot alike.
`tests/fixtures/d1-ee-ag.manifest.json` is the manifest of d1-ee-ag 0.1.1;
`tests/fixtures/published/` holds the other documents the hub serves
(d1-firmware.end-effector.damiao.motor 0.1.2,
d1-firmware.end-effector.robotera.xhand-lite 0.1.0, the fit of d1-3
revision 2), the descriptors they are held to, and the manifests of
d1-firmware.end-effector.leadshine.dh116 0.1.1 and
d1-firmware.end-effector.linkerbot.o30 0.1.1, which read no configuration
file;
`tests/fixtures/rules/` the accept and refuse cases d1-provisioning's tests
share.
The developer path is `docs/third-party-providers.md`.
