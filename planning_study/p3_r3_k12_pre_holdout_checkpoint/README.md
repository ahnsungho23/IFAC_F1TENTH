# R3-K12 pre-final-holdout checkpoint

This checkpoint freezes the complete seen-data research chain immediately
before the first `FINAL_HOLDOUT_UNSEEN` evaluation. It does not contain a
holdout result and does not authorize a holdout rerun.

## Frozen authorities

| authority | SHA-256 |
|---|---|
| `R3_LEXICOGRAPHIC_COVERAGE_RESERVE_K12` | `7b861e8c7e23ae168413885ea0dc2d09769046ff48a562700b658e084d116fcc` |
| Reference Oracle v2 spec | `62b63b8ab05d5a73565398141bd05a9db4a102541855fb2e3634d85b18a6bffe` |
| Evaluation contract | `226b1b44a9bea6adf26715658f36ae8e7b2c322e030f363270728f9e044b1dad` |
| Dataset split manifest | `c57cfe8e57dfca4bb318d30047e8f7215f6994e88a39babfbdbb10ce7637b2f4` |
| Exact Oracle v2 harness binary | `8b23f2b6df54c2e93794ee087f3ebd7a14f9921544af4f97eef755535e045e7e` |

The selected policy is
[`selected_v2_method_spec.json`](../p3_geometry_factor_ranking_v2/selected_v2_method_spec.json).
The finite oracle and metric contracts are in
[`p3_reference_oracle_v2`](../p3_reference_oracle_v2/), and the split authority
is [`dataset_split_manifest.csv`](../p3_mapping_research_corpus_v1/dataset_split_manifest.csv).

## Included history

The checkpoint versions the pilot, mapping diagnosis/corpus, transition study,
factorized v1 design, geometry-conditioned v1 method and seen validation,
Reference Oracle v2 repair, validation diagnosis, and selected ranking v2
method. Exact `.event` snapshots are included because they are evaluator inputs,
not rosbag copies. Compact summaries, plots, frozen specifications, scripts,
execution manifests, and existing SHA sidecars are retained.

Research instrumentation v2 source and parity tests were already committed in
parent baseline `80ae205fd470f537bd1e5449fb906c5548f6653a` and remain part of this
tag. Its production `src/local_planning` tree is unchanged by this checkpoint:
tree `d7b078199a64b39bfc4346d1f642837c9ea40e9f`, config tree
`578d1ba804734ee9bb2744553794451f6e3ad78e`.

## Deliberately excluded generated dumps

Seven deterministic candidate-level dumps totaling 42,128,049 bytes are not
versioned. They are regenerable from the included scripts, exact input
snapshots, frozen specs, and detached harness. Their compact summaries remain
versioned. Exact paths and byte sizes are recorded in
[`checkpoint_manifest.json`](checkpoint_manifest.json) and ignored explicitly
by the repository `.gitignore`.

No rosbag, MCAP/DB3 copy, build/install/log directory, runtime cache, compiled
binary, or `/tmp` artifact is committed. The detached harness source and build
recipe are versioned in [`p3_reference_oracle_v2/harness`](../p3_reference_oracle_v2/harness/).

## Holdout boundary

The checkpoint creation procedure hashes the split manifest as an opaque frozen
artifact but does not parse or load its rows. `FINAL_HOLDOUT_UNSEEN` rows loaded
and evaluations executed remain zero. No final-holdout command was run.
