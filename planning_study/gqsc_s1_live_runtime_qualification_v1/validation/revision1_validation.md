# Revision-1 non-performance validation

Date: 2026-09-02. Branch `research`; authoring HEAD
`99dcb24664a43f28bd04f7889071e6b966a76097`. These are wiring/parity artifacts, not latency
results. Both drivers used `smoke_mode=true`; numeric `O_total` and raw timed JSON were discarded.

## W2

Command: `tools/run_w2_smoke.zsh`. Result: four uniquely joined callbacks, four source epochs,
parity PASS. The graph contains the actual installed `local_planner_node` and the external replay
driver, with one driver publisher and one planner subscriber on each qualified input.

- `w2_joined_smoke.jsonl` SHA-256:
  `6342393165476f3d7b32482d9ede867061045b4a0eff6386a972c77cf29e5f4b`
- `w2_ros_graph.txt` SHA-256:
  `96f58425bf21fe930d834ad26da14da492a1d42af5189b5f6d2711e13074edba`

## W3

Command: `tools/run_w3_smoke.zsh`. Result: four uniquely joined callbacks, four source epochs,
parity PASS. Ready-node evidence includes simulator bridge/map/RViz/robot state, kinematic
localization, global trajectory/Frenet conversion, obstacle detector, the single qualification
local planner, state machine, and all control nodes. Qualified input topics each have exactly the
driver publisher and planner subscriber. Normal `/confirmed_static_obs` remains published by
`obstacle_detector`; normal `/ego_racecar/odom` remains published by `bridge` and is consumed by the
normal stack.

- `w3_joined_smoke.jsonl` SHA-256:
  `1e37cbcfd44db4cb9fe93fa152603fdd0d2a318a754113d65c65745fbcd32610`
- `w3_ros_graph.txt` SHA-256:
  `bc612bf449dff3fa1a62cfdcd70ad37d84c017d73834974e430fab374ea445e2`

Every retained joined row asserts `TEST_ACTIVE`, one fresh R3 invocation, `128/12/12`, candidate
count 12, the frozen candidate identity/provenance/digest, hard-valid selected lifecycle, and no
safe stop. No p95/p99 or other latency statistic was computed.

## CPU topology inspection only

Host: Intel i7-14650HX, one NUMA node/socket, logical CPUs 0–23. CPU 8 is core 4, maximum 5200 MHz,
and sibling set `8-9`. Kernel command line has neither `isolcpus` nor `nohz_full`. Revision 1 keeps
CPU 8 as the predeclared P-core affinity target but excludes both CPU 8 and sibling CPU 9 from all
non-planner experiment processes in A and B. No taskset A/B qualification was executed.
