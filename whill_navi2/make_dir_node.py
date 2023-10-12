import rclpy
from rclpy.node import Node
from ament_index_python.packages import get_package_share_directory
import os
import yaml


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
        self._date_param_ = self.declare_parameter("date_path", "date_today")
        self._map_path_param_ = self.declare_parameter("map_path", "map")
        self._bag_path_param_ = self.declare_parameter("bag_path", "bag")
        self._waypoint_path_param_ = self.declare_parameter("waypoint_path", "waypoint")
        self._branchpoint_path_param_ = self.declare_parameter("branchpoint_path", "branchpoint")

        map_path = self._map_path_param_.get_parameter_value().string_value
        bag_path = self._bag_path_param_.get_parameter_value().string_value
        waypoint_path = self._waypoint_path_param_.get_parameter_value().string_value
        branchpoint_path = self._branchpoint_path_param_.get_parameter_value().string_value

        # Set variables
        self._prefix_path_ = os.path.join(
            os.path.expanduser("~"),
            self._ws_param_.get_parameter_value().string_value,
            'full_data',
            self._date_param_.get_parameter_value().string_value,
            self._place_param_.get_parameter_value().string_value,
        )
        self._suffix_paths_dict_ = {
            map_path: ["", "re"], 
            bag_path: ["production_"], 
            waypoint_path: ["", "re", "final"], 
            branchpoint_path: [""]
        }
        self._suffix_path_key_list_ = [
            map_path,
            bag_path,
            waypoint_path,
            branchpoint_path
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

    paramspath_make_dir = os.path.join(
        get_package_share_directory('whill_navi2'),
        'config', 'params', 'make_dir_node_params.yaml'
    )
        
    with open(paramspath_make_dir) as f:
        nodeparams_make_dir = yaml.safe_load(f)['make_dir_node']['ros__parameters']
        base_path = os.path.join(
            os.environ['HOME'],
            nodeparams_make_dir['ws_path'],
            "full_data",
            str(nodeparams_make_dir['date_path']),
            nodeparams_make_dir['place_path']
        )
        launcharg_full_data_path = {'full_data_path_launch' : {}}
        for key, value in nodeparams_make_dir.items():
            if 'path' in key:
                launcharg_full_data_path['full_data_path_launch'][key + "_abs"] = (os.path.join(base_path, str(value)))
            if 'name' in key:
                launcharg_full_data_path['full_data_path_launch'][key] = value
            with open(os.path.join(
                    get_package_share_directory('whill_navi2'),
                    'config', 'launch_arg', 'full_data_path_launch_arg.yaml'
                ), 'w') as f:
                yaml.safe_dump(launcharg_full_data_path, f)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
