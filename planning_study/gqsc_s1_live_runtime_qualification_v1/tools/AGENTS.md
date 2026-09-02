# Runtime qualification tool rules

- These tools are external research drivers. They must not link to private `local_planning`
  planner classes or duplicate planner decisions.
- Publish only existing ROS message types and use only public planner topics.
- Keep SCE018 geometry byte-derived from the canonical event file; do not hand-maintain a second
  numeric copy.
- Smoke mode must redact all timing values. It may prove field presence and joins, but must not
  create a latency result table.
- A source-stamp regression may only exercise the production node's existing source-restart
  recovery. Never change `p3_mode`, safety guards, candidate policy, or bounded counts.
- Qualification output must retain raw profile/diagnostic messages; smoke output must not.

