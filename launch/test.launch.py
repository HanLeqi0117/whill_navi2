from whill_navi2.whill_navi2_node import Waypoint, DataPath
import typing
from geometry_msgs.msg import Pose, Point
            
if __name__ == '__main__':
    data_path = DataPath()
    read_file = data_path.get_rewapypoint_path()[0]
    waypoints_list_ = list[Waypoint]()
    wp_tmp = Waypoint

    waypoints_list_.append(wp_tmp)
    print(waypoints_list_)

    with open(read_file, 'r') as f:
        wp_tmp = Waypoint
        for line in f.readlines():
            if line.count(',') == 7:
                line.strip('\n')
                token_list = line.split(',')
                float_list = []
                for token in token_list:
                    float_list.append(float(token))
                wp_tmp.mode = int(token_list[7])
