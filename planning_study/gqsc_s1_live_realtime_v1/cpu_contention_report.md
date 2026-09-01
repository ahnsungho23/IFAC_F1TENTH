# CPU and contention report

## Host topology

The profiling host reports an Intel Core i7-14650HX with 24 online logical CPUs. Sysfs maximum
frequency distinguishes the hybrid topology:

- logical CPUs 0-15: 5.0 or 5.2 GHz maximum;
- logical CPUs 16-23: 3.7 GHz maximum.

Idle/current frequencies observed during the audit were commonly around 0.8-1.0 GHz, so both
core class and frequency residency can affect a short 17-31 ms burst.

## Controlled affinity diagnostic

`taskset` was used only for two diagnostic harness runs of the identical SCE018 query:

| placement | p50 | p95 | p99 | max |
|---|---:|---:|---:|---:|
| CPU 8, performance class | 21.288 | 22.008 | 22.418 | 22.769 |
| CPU 22, efficiency class | 30.635 | 30.952 | 31.089 | 31.233 |

The counts, ordered candidates, path digests, validator verdicts, and selection were unchanged.
The major stages slowed together: pair proxy median 2.947 -> 5.017 ms, reconstruction
5.088 -> 7.102 ms, and exact validation 11.816 -> 16.799 ms.

During the ROS-node-isolated run, a read-only process snapshot placed `local_planner_node` on
logical CPU 22. Its callback p95 was 30.824 ms without a running simulator. This is direct evidence
that heterogeneous-core scheduling alone can reproduce the problematic tail.

## Concurrent host load

A read-only process snapshot during diagnosis showed substantial desktop load, led by roughly:

- VS Code GPU process: 34% CPU;
- Chrome renderer: 12%;
- C/C++ language service: 10%;
- VS Code renderer: 9%;
- Chrome GPU process: 8.5%;
- Xorg: 6.6%;
- GNOME Shell: 5.8%.

The load average was approximately 3.09. A full closed loop additionally runs gym physics/scan,
KICP, global/Frenet planning, state machine, controller, and ROS executor traffic. These values are
a point-in-time diagnostic, not a stable benchmark contract.

## Attribution

- ROS conversion, publication, and mutex wait are sub-millisecond in the measured callbacks.
- Research instrumentation changed allowed-seen evaluator p95 by about 0.10 ms.
- Full-bound pair/reconstruction/validation work accounts for most evaluator time and scales with
  the core class even when its work counts and output are identical.
- The live gap must therefore not be attributed wholly to GQSC algorithm complexity.

No CPU affinity, realtime priority, scheduler policy, power governor, or OS setting was changed as
a deployment modification.

