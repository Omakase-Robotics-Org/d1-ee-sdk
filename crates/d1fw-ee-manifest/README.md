# d1fw-ee-manifest

The provider manifest `omakase-ee-provider/1`, which describes one release
of an end-effector provider to the robot that runs it:

- `SCHEMA_JSON`: the JSON Schema (draft 2020-12) committed as
  `schema/omakase-ee-provider-1.schema.json`;
- `ProviderManifest`: the document as a Rust type. `parse` checks the
  schema, then the rules a schema cannot state: `sdk.d1_ee_sdk` is a Cargo
  SemVer range holding `conformance.version`; `{side}` is the only
  placeholder of `launch.command` and `launch.config_file` and appears in
  both; the program is a file under `/opt/<package>/bin/`; a `registry`
  package's key names this release's `.deb`; each posture has one value per
  joint; models are distinct; `config_schema` is a self-contained draft
  2020-12 schema;
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
  an invalid one or a disagreeing descriptor, 2 for a file it cannot read.

The crate needs no network and does not link the shared-memory transport:
it runs on a developer's machine, in a release job and on a robot alike.
`tests/fixtures/d1-ee-ag.manifest.json` is the manifest of d1-ee-ag 0.1.1.
The developer path is `docs/third-party-providers.md`.
