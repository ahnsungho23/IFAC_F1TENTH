# Vendored from TUM `trajectory_planning_helpers` (LGPL-3.0), commit b1b38ecf.
#
# MODIFIED vs upstream: this file originally imported all 31 sibling modules.
# The Forza generator needs only the 19 modules vendored here (transitive
# closure of forza_trajectory_gui.py), so the imports for the 12 unvendored
# modules were removed. No other vendored file is modified. See ../README.md.
#
# The import order below is upstream's, minus the removed entries. Modules that
# do `import trajectory_planning_helpers as tph` and then reach for
# `tph.<submodule>` (create_raceline, iqp_handler, spline_approximation,
# prep_track) rely on this file populating the package namespace, so it cannot
# simply be emptied.
import trajectory_planning_helpers.interp_splines
import trajectory_planning_helpers.calc_spline_lengths
import trajectory_planning_helpers.calc_splines
import trajectory_planning_helpers.normalize_psi
import trajectory_planning_helpers.calc_head_curv_an
import trajectory_planning_helpers.calc_head_curv_num
import trajectory_planning_helpers.calc_t_profile
import trajectory_planning_helpers.import_veh_dyn_info
import trajectory_planning_helpers.calc_ax_profile
import trajectory_planning_helpers.calc_vel_profile
import trajectory_planning_helpers.spline_approximation
import trajectory_planning_helpers.side_of_line
import trajectory_planning_helpers.conv_filt
import trajectory_planning_helpers.create_raceline
import trajectory_planning_helpers.iqp_handler
# quadprog-linked module; make optional to avoid import-time hard failure
try:
    import trajectory_planning_helpers.opt_min_curv
except Exception:
    pass
import trajectory_planning_helpers.interp_track_widths
import trajectory_planning_helpers.check_normals_crossing
import trajectory_planning_helpers.interp_track
