# vendor/ — TUM trajectory optimization modules

Vendored so `forza_trajectory_gui.py` runs with no ROS 2 workspace, no colcon
build and no separately installed `trajectory_planning_helpers`.

## Provenance

| Item | Value |
|---|---|
| Upstream | https://github.com/TUMFTM/trajectory_planning_helpers |
| | https://github.com/TUMFTM/global_racetrajectory_optimization |
| Pinned commit | `b1b38ecfefec7200d5bcb243f115af1d795b3766` |
| License | **LGPL-3.0** — see `LICENSE` (verbatim copy of upstream's) |

LGPL-3.0 incorporates the GPL-3.0 by reference; upstream ships only the LGPL
supplement, so this copy matches it. GPL-3.0 text: https://www.gnu.org/licenses/gpl-3.0.txt

The same commit is recorded as `OPTIMIZER_COMMIT` in `../forza_trajectory_gui.py`
and written into every run's `metadata.json` under `forza_source`.

## Scope — transitive closure only

`trajectory_planning_helpers/` holds **19 of upstream's 31 modules**: the
transitive closure of what `forza_trajectory_gui.py` calls. `helper_funcs_glob/`
holds **2 of 8**: `prep_track.py` and `interp_track.py`.

Deliberately not vendored:

- `inputs/frictionmaps/` (~11.1 MB) — Berlin/Modena variable-friction maps, used
  only by the mintime optimizer, which this generator never runs.
- `helper_funcs_glob`'s `result_plots.py` — pulls in `matplotlib` + `mpl_toolkits`.
- The 12 unused `trajectory_planning_helpers` modules: `angle3pt`,
  `calc_normal_vectors`, `calc_normal_vectors_ahead`, `calc_tangent_vectors`,
  `calc_vel_profile_brake`, `get_rel_path_part`, `import_veh_dyn_info_2`,
  `nonreg_sampling`, `opt_shortest_path`, `path_matching_global`,
  `path_matching_local`, `progressbar`.

## Modifications to upstream

**One file:** `trajectory_planning_helpers/__init__.py`. Upstream imports all 31
sibling modules; the imports for the 12 unvendored ones were removed. Everything
else is byte-identical to upstream.

It could not simply be deleted: `create_raceline`, `iqp_handler`,
`spline_approximation` and `prep_track` do `import trajectory_planning_helpers as
tph` and then reach for `tph.<submodule>`, which only resolves because this file
populates the package namespace.

`helper_funcs_glob/` intentionally has **no** `__init__.py`. Upstream's imports
by full package path (`global_racetrajectory_optimization.helper_funcs_glob.src`),
which does not exist here; `forza_trajectory_gui.py` puts this directory on
`sys.path` and imports `prep_track` / `interp_track` as top-level modules, so
upstream's package `__init__.py` never ran in the first place.
