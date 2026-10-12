# Upstream provenance

Agnes Harness source is vendored without modifying its tracked source files.

- Upstream: https://github.com/AgnesAI-Labs/agnes-harness
- Pinned commit: fe90962a6b8561d8cae7e5d451905604fcf475cf
- Location: third_party/agnes-harness
- Full upstream tracked file set is included, with file modes and symlinks preserved.
- Upstream .git history, node_modules and machine-specific build output are not included.
- Preserve and comply with the upstream LICENSE and notices. The repository includes upstream public test fixtures containing intentionally fake tokens and localhost-only TLS test material; these are not project authentication secrets.
- Project integrations live outside upstream tracked sources (scripts, services, packages and demo).

Publishing does not modify the original nested Git repository in the running server workspace. The outer repository is a source snapshot, not a forked upstream history.
