# Immutable workload freeze — operational revision 3, protocol revision 2

## Revision lineage

Revision 0 froze `W1_SCE018_STANDALONE` and blocked W2/W3. Revision 1 resolves those blockers by
using SCE018 itself as the immutable W2/W3 planner input. It does not relabel the historical SHADOW
run as W2 and does not substitute the missing historical S01 path. Workload selection is based on
causal comparability and reproducibility, never observed latency.

Binary provenance revision 2 changes no workload contract. It replaces only the disallowed
non-Release executables with a clean Release overlay built from source commit
`dc33b875a938fdb7730d35fe61de43ee863b04c0`.

Execution attempt 0 stopped before timing. Protocol revision 2 changes no workload bytes or
execution scale; it freezes the previously missing decision/quantile rules and actual-affinity
evidence. W1/W2 timing scopes were proven non-equivalent, so W1 remains an algorithm-isolated
diagnostic and cannot be subtracted from W2 as pure ROS overhead.

The revision-2 execution attempt then stopped before timing because the directly invoked W1 runner
was stored with Git mode `100644`. Revision 3 is operational only: it records mode `100755`, adds a
filesystem/frozen-tree executable preflight, and routes future outputs to `executions/r3/`. The
scientific identities and contracts below are unchanged.

## Common production identity

| Item | Frozen value |
|---|---|
| algorithm checkpoint | `55c61e54540fc07d57e6041655e0e59ddb8e17e8` |
| revision-1 authoring HEAD | `99dcb24664a43f28bd04f7889071e6b966a76097` |
| method | `LEX8_GLOBAL_DISJOINT_COVERAGE4` |
| method SHA-256 | `670f39a23479bcdcc1db0829a895257443fec8ee2f78f186bc1beb224090b776` |
| reference-v3 SHA-256 | `965f6ce65b7ce5b1c6426a0975c1dfafe779e89eb20308bb09a11a1b4a22e780` |
| pair/lexicographic/coverage | `128/8/4` |
| reconstruction/validator maxima | `12/12` |
| planning period / production mode | `25 ms` / `TEST_ACTIVE` |
| frozen-contract file SHA-256 | `21e653cf9b063c9c60843c6ebaeb06fda918046bfe6d78a7d4ba6b957bdddd40` |
| planner configuration SHA-256 | `4fe480351a80135ff2a6e4592f661ff8a5670c032e12554d85065339d16ea960` |
| rejected normal harness SHA-256 | `5428854081403031bd7fd3d037eae11258b0fb9bd6e48d3f9a59ebbfc49a6ede` (`NON_RELEASE`) |
| rejected normal ROS node SHA-256 | `3059d3c51d563fc2f4102289508a0a7578ddc059543d97022d451744f295d595` (`NON_RELEASE`) |
| frozen W1 Release harness SHA-256 | `49503d48d96cf408ad47691a69b683e5f2a0c02947a73283994f57b2697c0fd2` |
| frozen W2/W3 installed Release node SHA-256 | `54019a86e13f4dc25771628f7a3d385be8e2c6ce2a657448b3a5939823ab878a` |
| planner/harness optimization | `-O3 -DNDEBUG` (`RELEASE_EQUIVALENT`) |
| replay source SHA-256 | `ab014c1e56fd03189327d418e63c88c3295b150936037c7dc65e6d1905b8251b` |
| replay Release binary SHA-256 | `49c5fd04c105883c744f470e810fb0811fa7d36cd7094c069b2e264c3ba8cafb` |
| Release-overlay builder SHA-256 | `75d41200c313060b4df4b56ac461d098d2f4787416dd450166adc3aa5ae94c38` |
| revision-1 W2 qualification runner SHA-256 | `3dbcc9130333231f1a21bc446df4ce7a8d0aced5d7c460afb574906ca8ed6830` |
| revision-1 W3 qualification runner SHA-256 | `5fd7889b6bf1f58c2d00474a3fd863280f6be8fd0d7ae48f8f36de5dfae0d85b` |
| revision-2 W1 qualification runner SHA-256 | `e25e1d19cda2c32766d91808eec331e88eeae3bc69695738553a36403963f07f` |
| revision-2 W2 qualification runner SHA-256 | `e2f7c03b9420a66a398b08562f031f63494fb99a1fb213124bd38cbbffd55202` |
| revision-2 W3 qualification runner SHA-256 | `457fee8b054ae2c765cb5e8d183f5f3a0bcd7d04a138b549ae116356b214e526` |
| revision-2 analysis SHA-256 | `92321d50542004236de9154a9098e5e0ebc4897d3c61531d7fd4c73160e455db` |

The normal workspace remains untouched. Binary provenance revision 2 builds only `local_planning` in
`release_overlay/_build`, installs it in `release_overlay/_install`, and records logs in
`release_overlay/_log`; all three generated directories are ignored. Full compiler evidence,
absolute paths, Build IDs, and hashes are in `binary_provenance.md`.

## Canonical SCE018 and ROS projection

Canonical file:
`planning_study/p3_geometry_conditioned_method_v1/success_control_inputs/SCE018.event`, SHA-256
`572adb59ea24f3f06bed7502eb870e57a33a5416e8630106b67bc2b1d0b405bf`. It contains 185
reference waypoints, ego `(s,d,speed)=(11.801691151003425,0.074708912484302642,0)`, source stamp
`1787619957628360127`, and obstacle id 231 with
`s=[20.857921486561061,21.002442409473012]` and
`d=[-0.39155248802935838,-0.072975715874275343]`.

The external parser performs this complete mapping:

| Event data | Public ROS field | Production state |
|---|---|---|
| every `W` row: id, s, d, x, y, d_right, d_left, psi, kappa, vx, ax | `/gqsc_runtime/global_waypoints`, `f110_msgs/WpntArray.wpnts[*]` matching fields; header frame `map`, transient/reliable | validated global reference; generation 1; the exact 185-point path used by the planner |
| `EVENT` ego s/d/speed | `/gqsc_runtime/frenet_odom`, `nav_msgs/Odometry.pose.position.x/y`, `twist.linear.x`; frame `map`, reliable | `ego_s`, `ego_d`, `ego_speed_mps` in the planning snapshot |
| every `O` row: id, s_center/start/end, d_right/left, size, s_var, d_var, is_static, is_visible | `/gqsc_runtime/confirmed_static_obs`, matching `f110_msgs/Obstacle` fields; `d_center=(d_right+d_left)/2`; other message fields stay their defined zero/false defaults | accepted static-obstacle snapshot and exact diagnostic geometry |
| canonical source stamp plus deterministic offsets | obstacle header stamp | `source_stamp_ns`; accepted-message `obstacle_sequence`; source-restart-driven `source_epoch` |
| fixed `STATE_GLOBAL` | `/gqsc_runtime/state`, transient/reliable | public state-machine dependency without a private planner call |
| callback publications | existing `/gqsc_runtime/p3_cycle` and `/gqsc_runtime/live_profile` remaps | callback/result join and production evaluator/result state |

The reference and obstacle frames are `map`. No frame conversion or geometry normalization is
needed, so exact digest parity is valid. Lifecycle identity is not injected: `source_epoch`,
`obstacle_sequence`, callback sequence, and reference generation remain production-owned values.

## W1: `W1_SCE018_STANDALONE`

Status: `FROZEN_READY`. Runtime is the frozen Release-overlay standalone integration harness.
Expected values are
pair/reconstruction/validator `128/12/12`, hard/usable `9/9`, outcome `FRESH_SELECTED`, failure
`NONE`, and selected digest `c7b2c19bf2af9350`. Event parsing and planner construction are outside
the primary timed interval. Exact commands are in `execution_protocol.md`.

## W2: `W2_SCE018_ROS_NODE_TEST_ACTIVE`

Status: `FROZEN_READY`. Runtime is the Release-overlay installed `local_planner_node` and executor,
with no
simulator. The only planner inputs are the private remaps in the table above. The external tool
parses the canonical event, publishes production messages, uses the existing public source-restart
lifecycle, and joins the two existing production output streams by `callback_sequence`. A
four-callback redacted smoke passed; the future exact runner fixes 20 warm-up plus 200 measurement
callbacks but was not executed in this task.

## W3: `W3_SCE018_FULL_STACK_CONTENTION`

Status: `FROZEN_READY`. W3 uses the exact same installed Release node as W2. The qualified planner
and replay contract are byte/logically identical to W2. Concurrent real-project processes are
simulator bridge/map/RViz/robot state, kinematic
localization, global planning/frenet conversion, obstacle detector, state machine, and the control
stack. The qualification planner is the sole local planner. Its inputs and outputs are private
remaps; the normal detector output and simulator odometry remain active only as system load.

Frozen W3 environment identities include:

- simulator config `49f9e5c1ae2d1d34c7fede4189e98a65f8d2977ae5ace0562735ffd28d9bca2f`
  and launch `a3f7f679b19d8598feea4a34944b5667a222109c78cc126e4e540f8a26ca0f86`;
- `ifac_track` PNG/YAML/Kissmap hashes `1f1fee4e...`, `76250116...`, `65620cf4...`;
- current global path hash
  `b9cdd21fb7067ebd32b9ce2f4d2ffbd8affc96c5d2193eaba02892a772a16fad`;
- global/localization/detector/state/control launch hashes `044fa7d6...`, `866bb0c1...`,
  `f4b83044...`, `4a978272...`, `99a775ca...`;
- initialization pose `(x,y,z,w)=(-0.427,0.456,0.3651,0.9310)` and `use_sim_time=false`.

The old S01 row remains revision-0 history only. It is not a revision-1 or revision-2 workload.
