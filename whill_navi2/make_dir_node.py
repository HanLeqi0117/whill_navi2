import rclpy
from rclpy.node import Node
from ament_index_python.packages import get_package_share_directory
import os
import yaml

#
# データ保存用ディレクトリ作成用Node
# Path_Tree
# 例：
# $HOME/$WORKSPACE
# └── full_data
#     └── date_today
#         └── nakanoshima
#             ├── branchpoint
#             ├── data_gather_bag
#             ├── finalwaypoint
#             ├── map
#             ├── production_bag
#             ├── remap
#             ├── rewaypoint
#             └── waypoint
#

class MakeDir(Node):
    def __init__(self):
        super().__init__("make_dir_node")

        # Intialize variables
        self._prefix_path_ = str()
        self._suffix_paths_dict_ = {str : str}
        self._suffix_path_key_list_ = [str]
        
        # Declare Parameters
        self._ws_param_ = self.declare_parameter("ws_dir", "whill2_ws")
        self._place_param_ = self.declare_parameter("place_dir", "nakanoshima")
        self._date_param_ = self.declare_parameter("date_dir", "date_today")
        self._map_dir_param_ = self.declare_parameter("map_dir", "map")
        self._bag_dir_param_ = self.declare_parameter("bag_dir", "bag")
        self._waypoint_dir_param_ = self.declare_parameter("waypoint_dir", "waypoint")
        self._branchpoint_dir_param_ = self.declare_parameter("branchpoint_dir", "branchpoint")

        map_dir = self._map_dir_param_.get_parameter_value().string_value
        bag_dir = self._bag_dir_param_.get_parameter_value().string_value
        waypoint_dir = self._waypoint_dir_param_.get_parameter_value().string_value
        branchpoint_dir = self._branchpoint_dir_param_.get_parameter_value().string_value

        # Set variables
        self._prefix_path_ = os.path.join(
            os.path.expanduser("~"),
            self._ws_param_.get_parameter_value().string_value,
            'full_data',
            self._date_param_.get_parameter_value().string_value,
            self._place_param_.get_parameter_value().string_value,
        )
        self._suffix_paths_dict_ = {
            map_dir: ["", "re"], 
            bag_dir: ["", "backup_", "production_"], 
            waypoint_dir: ["", "re", "final"], 
            branchpoint_dir: [""]
        }
        self._suffix_path_key_list_ = [
            map_dir,
            bag_dir,
            waypoint_dir,
            branchpoint_dir
        ]

        self.make_dir()

        # Debug
        # print(self._prefix_path_)

    def make_dir(self):
        
        for suffix_path_key in self._suffix_path_key_list_:
            for suffix_path in self._suffix_paths_dict_[suffix_path_key]:
                full_path = os.path.join(
                    self._prefix_path_,
                    suffix_path + suffix_path_key
                )
                if not os.path.exists(full_path):
                    os.makedirs(full_path)
                    # Debug
                    # print(full_path)
                else:
                    self.get_logger().warn("Path exists!" + full_path)
                    return
        
        self.get_logger().info("Directories are generated successfully.")

def main(args=None):
    rclpy.init(args=args)
    # rclpy.spin(MakeDir)
    # rclpy.shutdown()
    node = MakeDir()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
