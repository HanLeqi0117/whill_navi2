import rclpy
from rclpy.node import Node
from ament_index_python.packages import get_package_share_directory
import os


class MakeDir(Node):
    def __init__(self):
        super().__init__("make_dir_node")

        # Intialize variables
        self._prefix_path_ = str()
        self._suffix_paths_dict_ = {str : str}
        self._suffix_path_key_list_ = [str]
        
        # Declare Parameters
        self._ws_param_ = self.declare_parameter("ws_path", "whill2_ws")
        self._place_param_ = self.declare_parameter("place_path", "nakanoshima")
        self._data_param_ = self.declare_parameter("data_path", "2023_07_21")
        self._map_path_param_ = self.declare_parameter("map_path", "map")
        self._bagfile_path_param_ = self.declare_parameter("bagfile_path", "bagfile")
        self._waypoint_path_param_ = self.declare_parameter("waypoint_path", "waypoint")
        self._dbranchpoint_path_param_ = self.declare_parameter("branchpoint_path", "branchpoint")

        # Set variables
        self._prefix_path_ = os.path.join(
            os.path.expanduser("~"),
            self._ws_param_.get_parameter_value().string_value,
            self._place_param_.get_parameter_value().string_value,
            self._data_param_.get_parameter_value().string_value
        )
        self._suffix_paths_dict_ = {
            "map": ["", "re"], 
            "bagfile": ["data_gather_", "production_"], 
            "waypoint": ["", "re", "final"], 
            "branchpoint": [""]
        }
        self._suffix_path_key_list_ = [
            self._map_path_param_.get_parameter_value().string_value,
            self._bagfile_path_param_.get_parameter_value().string_value,
            self._waypoint_path_param_.get_parameter_value().string_value,
            self._dbranchpoint_path_param_.get_parameter_value().string_value
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


if __name__ == '__main__':
    main()
