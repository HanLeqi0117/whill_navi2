import rclpy
from rclpy.node import Node
from whill_navi2.modules.ros2_launch_utils import shutil, os, ruamel, DataPath

class Map2RemapNode(Node):

    def __init__(self):
        super().__init__('map2remap_node')
        data_path = DataPath()
        map_pgm_file_path = os.path.join(data_path.map_dir, data_path.map_name + '.pgm')
        map_yaml_file_path = os.path.join(data_path.map_dir, data_path.map_name + '.yaml')
        remap_pgm_file_path = os.path.join(data_path.remap_dir, data_path.remap_name + '.pgm')
        remap_yaml_file_path = os.path.join(data_path.remap_dir, data_path.remap_name + '.yaml')
        print(map_pgm_file_path, map_yaml_file_path, remap_pgm_file_path, remap_yaml_file_path)
        if os.path.exists(map_pgm_file_path) and os.path.exists(map_yaml_file_path) and os.path.exists(os.path.join(data_path.remap_dir)):
            self.get_logger().info("Copy files in map to remap, and rewrite the parameter in remap.yaml")
            shutil.copyfile(map_pgm_file_path, remap_pgm_file_path)
            shutil.copyfile(map_yaml_file_path, remap_yaml_file_path)
            with open(remap_yaml_file_path, "r") as f:
                remap_yaml_dict = ruamel.yaml.safe_load(f)
            remap_yaml_dict['image'] = data_path.remap_name + '.pgm'
            with open(remap_yaml_file_path, "w") as f:
                ruamel.yaml.safe_dump(remap_yaml_dict, f)
        else:
            self.get_logger().warn("Could not find directories!")
        
def main(args=None):
    rclpy.init(args=args)
    map2remap_node = Map2RemapNode()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
