import sys
sys.path.append('/home/pi/cs437-picar')
import mapping
from mapping import Mapper, MapCoordinate

if __name__ == '__main__':
    mapping.MAX_DISTANCE_STD_TH = 0.5
    mapper = Mapper(scan_res=5, num_scans=50)
    car_position = MapCoordinate(0, 0)
    car_heading = 0
    mapper.update_grid(car_position, car_heading, with_clearing=False)
    mapper.plot_map()
