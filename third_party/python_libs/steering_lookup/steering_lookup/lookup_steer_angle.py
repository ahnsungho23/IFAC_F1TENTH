from pathlib import Path

import numpy as np

try:
    import ament_index_python.packages as ament_pkg
except ImportError:
    ament_pkg = None


def find_nearest(array, value):
    """Return the nearest array value and its index."""
    idx = np.abs(array - value).argmin()
    return array[idx], idx


def find_closest_neighbors(array, value):
    """Return the two array entries nearest to a lookup value."""
    nan_indices = np.argwhere(np.isnan(array))
    if len(nan_indices) > 0:
        array = array[:nan_indices[0][0]]

    closest, closest_idx = find_nearest(array, value)
    if closest_idx == 0:
        return array[0], 0, array[0], 0
    if closest_idx == len(array) - 1:
        return array[closest_idx], closest_idx, array[closest_idx], closest_idx

    neighbor_indices = [closest_idx - 1, closest_idx + 1]
    second_closest, relative_idx = find_nearest(array[neighbor_indices], value)
    second_idx = neighbor_indices[relative_idx]
    return closest, closest_idx, second_closest, second_idx


class LookupSteerAngle:
    """Look up a steering angle from acceleration and velocity samples."""

    def __init__(self, model_name, logger=None):
        try:
            if ament_pkg is None:
                raise LookupError
            package_share = Path(
                ament_pkg.get_package_share_directory('steering_lookup')
            )
        except Exception:
            package_share = Path(__file__).resolve().parents[1]

        lookup_path = package_share / 'cfg' / f'{model_name}_lookup_table.csv'
        self.lu = np.loadtxt(lookup_path, delimiter=',')
        self.logger = logger

    def lookup_steer_angle(self, accel, vel):
        """Interpolate the steering angle for lateral acceleration and speed."""
        sign_accel = 1.0 if accel > 0.0 else -1.0
        accel = abs(accel)
        lookup_velocities = self.lu[0, 1:]
        lookup_steers = self.lu[1:, 0]

        _, velocity_idx = find_nearest(lookup_velocities, vel)
        closest, closest_idx, second, second_idx = find_closest_neighbors(
            self.lu[1:, velocity_idx + 1], accel
        )

        if closest_idx == second_idx:
            steer_angle = lookup_steers[closest_idx]
        else:
            steer_angle = np.interp(
                accel,
                [closest, second],
                [lookup_steers[closest_idx], lookup_steers[second_idx]],
            )
        return steer_angle * sign_accel


if __name__ == '__main__':
    lookup = LookupSteerAngle('NUC6_glc_pacejka', print)
    print(lookup.lookup_steer_angle(9, 7))
