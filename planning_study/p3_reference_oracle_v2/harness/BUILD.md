# Reference Oracle v2 detached harness build

This directory freezes the audit-only source needed to rebuild the exact
Reference Oracle v2 evaluator without changing the production checkout.
The adapter patch adds detached public wrappers and a CMake target only in a
temporary worktree. It must never be applied to the production branch.

## Provenance

| artifact | SHA-256 | role |
|---|---|---|
| `p3_family_oracle_harness_v1.cpp` | `d7050e61a1f2c582fa3a08fc5f94cad4c8f378b4bc2a5ce1b11162174180e69a` | predecessor source recorded in the original Reference Oracle v2 README |
| `p3_family_oracle_harness_v2.cpp` | `c3aa74b1aa76ca87899fbe3805fe8fa972b7d6168ff1d9fff360914d48d38594` | exact source used for the frozen Oracle v2 binary |
| `local_planning_oracle_adapters.patch` | `53ae5d741d4d8efbbd7b795e3ed6e32db28b0097351e4be2f74d5feef3f5edd9` | detached adapters and CMake target |
| frozen Oracle v2 binary | `8b23f2b6df54c2e93794ee087f3ebd7a14f9921544af4f97eef755535e045e7e` | exact validator binary used by the study; binary is not versioned |

The old README's `d705...` source hash refers to the v1 predecessor. The
instrumentation-aware binary `8b23...` was built from the v2 source `c3aa...`.
This checkpoint records both instead of rewriting historical results.

## Rebuild recipe

The frozen build used ROS 2 Jazzy, GCC 13.3.0, CMake 3.28.3, Unix Makefiles,
and `CMAKE_BUILD_TYPE=Release` on Linux x86_64. Replace `REPO` and
`F110_OVERLAY` with local absolute paths.

```zsh
REPO=/path/to/2026_IFAC
F110_OVERLAY=/path/to/f110_msgs_overlay/install/setup.zsh
AUDIT_ROOT=$(mktemp -d /tmp/p3-oracle-v2-rebuild.XXXXXX)

git -C "$REPO" worktree add --detach "$AUDIT_ROOT/source" p3_r3_k12_pre_holdout
git -C "$AUDIT_ROOT/source" apply \
  planning_study/p3_reference_oracle_v2/harness/local_planning_oracle_adapters.patch
cp "$AUDIT_ROOT/source/planning_study/p3_reference_oracle_v2/harness/p3_family_oracle_harness_v2.cpp" \
  "$AUDIT_ROOT/source/src/local_planning/test/p3_family_oracle_harness.cpp"

source /opt/ros/jazzy/setup.zsh
source "$F110_OVERLAY"
cmake -S "$AUDIT_ROOT/source/src/local_planning" \
  -B "$AUDIT_ROOT/build" -DCMAKE_BUILD_TYPE=Release
cmake --build "$AUDIT_ROOT/build" --target p3_family_oracle_harness --parallel
sha256sum "$AUDIT_ROOT/build/p3_family_oracle_harness"
```

The expected final line is the frozen binary SHA `8b23f2b6...`. Toolchain or
dependency drift can change binary bytes even when evaluator behavior is the
same; the versioned source, adapter patch, frozen parameter literals, and
candidate-digest parity are the semantic reproduction authorities.
