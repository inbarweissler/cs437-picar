import sys
sys.path.append('/home/pi/cs437-picar')
import numpy as np
import picar_4wd as fc
import matplotlib.pyplot as plt

class ScanningGrid:
    max_dist_cm = 30
    grid_res_cm = 1  # each cell in the grid represents 1cm
    grid_len_x = int(max_dist_cm*2/grid_res_cm)
    grid_len_y = int(max_dist_cm/grid_res_cm)
    x0_cm = -max_dist_cm
    y0_cm = 0

    def __init__(self):
        self.grid = np.zeros((self.grid_len_x, self.grid_len_y))
        pass

    def update_grid(self, obj_dist, obj_angle):
        x_cm = int(obj_dist * np.sin(np.radians(obj_angle)))
        y_cm = int(obj_dist * np.cos(np.radians(obj_angle)))

        if -self.max_dist_cm < x_cm < self.max_dist_cm-1 and y_cm < self.max_dist_cm-1:
            x_ind = int((x_cm-self.x0_cm)/self.grid_res_cm)
            y_ind = int((y_cm-self.y0_cm)/self.grid_res_cm)

            self.grid[x_ind, y_ind] = 1


if __name__ == '__main__':
    scan_res = 10  # deg
    scan_span = 60  # deg, +-90 deg from center
    scanning_grid = ScanningGrid()
    num_scans = 5  # to reduce noise impact

    for angle_deg in range(-scan_span, scan_span):
        fc.servo.set_angle(angle_deg)
        dist_list = [fc.us.get_distance() for i in range(num_scans)]
        if all(dist is None for dist in dist_list):
            dist_cm = None
        else:
            dist_cm = np.mean([dist for dist in dist_list if dist is not None])
            scanning_grid.update_grid(dist_cm, -1*angle_deg)  # angle is represented opposite to x_axis

    plt.pcolor(scanning_grid.grid.T)
    plt.show()
    # TODO: add interpolation
