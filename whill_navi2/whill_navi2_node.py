from gps_wp_pkg.waypoint_utils.utils import (
    rclpy, quaternion_from_euler, NavSatFix,
    Buffer, TransformListener, TransformStamped,
    PoseStamped, os, ruamel, ParameterDescriptor,
    get_dist_between_geos
)
from geographic_msgs.msg import GeoPose
from rclpy.time import Time
from rclpy.duration import Duration
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from whill_navi2.modules.ros2_launch_utils import get_package_prefix, DataPath

class NaviMode:
    search = False
    find = False
    catch_obj = False
    catch_signal = False
    go_person = False
    arrive_person = False
    skip = False
    send_mode = 0
    reference_waypoint_now = 0
    referenced_waypoint_past = 0
    gomi_num = 0
    is_last_point = False
    catch_name = ''
    
    def reset(self):
        self.search = False
        self.find = False
        self.catch_obj = False
        self.catch_signal = False
        self.go_person = False
        self.arrive_person = False
        self.is_last_point = False

        send_mode = -1
    
    def is_last_waypoint():
        pass
    
    def next_reference_pose():
        pass

class WhillNavi2Node(BasicNavigator):
    
    def __init__(self):
        super().__init__(node_name="whill_navi2_node")
        self._tf_buffer_ = Buffer()
        self._tf_listener_ = TransformListener(self._tf_buffer_, self)
        self._now_mode_ = NaviMode()
        self._navsat_fix_data_ = NavSatFix()
        self._waypoints_ = {"waypoints" : [{}]}
        
        sound_list_dir = os.path.join(
            get_package_prefix("whill_navi2"), '..', '..', 
            'src', "whill_navi2", 'doc', 'sound_list'
        )
        data_path = DataPath()
        read_file = data_path.get_rewapypoint_path()[0]
        self._read_file_ = self.declare_parameter(
            "read_file", 
            read_file
        ).get_parameter_value().string_value
        self._mode_ = self.declare_parameter(
            "mode",
            "SLAM",
            ParameterDescriptor(name="navi_mode", description="SLAM or GPS")
        ).get_parameter_value().string_value
        self._person_name_ = self.declare_parameter(
            "person_name", 
            "Michael"
        ).get_parameter_value().string_value
        self._gomi_list_ = self.declare_parameter(
            "gomi_list", [
                'Bento', 
                'Can', 
                'Bottle'
            ]
        ).get_parameter_value().string_array_value
        self._signal_list_ = self.declare_parameter(
            "signal_list", [
                'grenn_signal', 
                'yellow_signal', 
                'red_signal'
            ]
        ).get_parameter_value().string_array_value
        self._sound_list_ = self.declare_parameter(
            "sound_list", [
                os.path.join(sound_list_dir, "Find_bento.wav"),
                os.path.join(sound_list_dir, "find_can.wav"),
                os.path.join(sound_list_dir, "find_petbottle.wav")
            ]
        ).get_parameter_value().string_array_value
        
        
        self._start_point_ = self.declare_parameter(
            'start_point', 0
        ).get_parameter_value().integer_value
        self._waypoints_size_ = self.declare_parameter(
            'waypoints_size', 3
        ).get_parameter_value().integer_value
        
        self._change_distance_ = self.declare_parameter(
            'change_distance', 2.0
        ).get_parameter_value().double_value
        self._arrive_distance_ = self.declare_parameter(
            'arrive_distance', 0.2
        ).get_parameter_value().double_value
        self._gomi_wait_duration_ = self.declare_parameter(
            'gomi_wait_duration', 10.0
        ).get_parameter_value().double_value
        self._before_gomi_wait_duration_ = self.declare_parameter(
            'before_gomi_wait_duration', 5.0
        ).get_parameter_value().double_value
        self._signal_wait_duration_ = self.declare_parameter(
            'signal_wait_duration', 10.0
        ).get_parameter_value().double_value
        self._before_signal_wait_duration_ = self.declare_parameter(
            'before_signal_wait_duration', 8.0
        ).get_parameter_value().double_value
        
        self._navsat_fix_sub_ = self.create_subscription(NavSatFix, "gps/filtered", self.navsat_fix_callback, 20)
                
        with open(self._read_file_, 'r') as f:
            self._waypoints_ = ruamel.yaml.safe_load(f)
                
        self.get_logger().info("Get waypoint file")
        self._now_mode_.reference_waypoint_now = self._now_mode_.referenced_waypoint_past = self._start_point_
        self._now_mode_.reset()
        
        # self._sub_object_ = self.create_subscription()
        # self._pub_sound_ = self.create_publisher()
        # self._map_change_ = self.create_publisher()        
            
    def get_tf(self, 
               source_frame : str, 
               target_frame : str, 
               tf_stamp : Time
        ) -> TransformStamped:
        
        if self._tf_buffer_.can_transform(target_frame, source_frame, tf_stamp, Duration()):
            return self._tf_buffer_.lookup_transform(target_frame, source_frame, tf_stamp, Duration())
        else:
            self.get_logger().warn("waiting for TF...")
            self.get_clock().sleep_for(Duration(nanoseconds=(int(5e7))))
    
    def navsat_fix_callback(self, msg : NavSatFix):
        self._navsat_fix_data_ = msg
    
    def get_waypoints_mode(self) -> int:
        pass
    
    def need_resend_waypoints(result : TaskResult) -> bool:
        pass
    
    def calc_dist(waypoint : dict, transform : TransformStamped) -> float:
        pass
    
    def waypoint_num_nearby(transform : TransformStamped) -> int:
        pass
    
    def get_poses(self, waypoints : [{}]) -> [PoseStamped]:
        goal_poses = []
        for waypoint in waypoints:
            goal_pose = PoseStamped()
            goal_pose.pose.position.x = waypoint["pos_x"]
            goal_pose.pose.position.y = waypoint["pos_y"]
            goal_pose.pose.position.z = waypoint["pos_z"]
            quaternion = quaternion_from_euler(0.0, 0.0, waypoint['yaw'])
            goal_pose.pose.orientation.x = quaternion[0]
            goal_pose.pose.orientation.y = quaternion[1]
            goal_pose.pose.orientation.z = quaternion[2]
            goal_pose.pose.orientation.w = quaternion[3]
            goal_pose.header.frame_id = 'map'
            goal_pose.header.stamp = self.get_clock().now().to_msg()
            goal_poses.append(goal_pose)
        
        return goal_poses

    def get_geo_poses(self, waypoints : [{}]) -> [GeoPose] :
        geo_poses = []
        
        for waypoint in waypoints:
            geo_pose = GeoPose()
            geo_pose.position.latitude = waypoint['latitude']
            geo_pose.position.longitude = waypoint['longitude']
            quaternion = quaternion_from_euler(0.0, 0.0, waypoint['yaw'])
            geo_pose.orientation.x = quaternion[0]
            geo_pose.orientation.y = quaternion[1]
            geo_pose.orientation.z = quaternion[2]
            geo_pose.orientation.w = quaternion[3]
            geo_poses.append(geo_pose)
        
        return geo_poses
            
            
    def whill_go(self, goal_poses = []):
        
        field_types = goal_poses[0].get_fields_and_field_types().values()
        
        if "geometry_msgs/Pose" in field_types:
            self.followWaypoints(goal_poses)
            mode = "SLAM"
        elif "geographic_msgs/GeoPoint" in field_types:
            self.followGpsWaypoints(goal_poses)
            mode = "GPS"
        else :
            self.get_logger().warn("Goals Type Error: {}".format(field_types))
        
        while not self.isTaskComplete():
            self.get_logger().info("Navigating to the Goals...")
            self.get_clock().sleep_for(Duration(nanoseconds=int(5e8)))
            transform_now = self.get_tf("base_link", "map", Time())
            if mode == "SLAM" : 
                dist = self.calc_dist(goal_poses[self._waypoints_size_ - 1], transform_now)
            elif mode == "GPS" :
                dist = get_dist_between_geos(
                    goal_poses[self._waypoints_size_ - 1]["latitude"],
                    goal_poses[self._waypoints_size_ - 1]["longitude"],
                    self._navsat_fix_data_.latitude,
                    self._navsat_fix_data_.longitude
                )
            
            if dist < self._arrive_distance_:
                self.cancelTask()
                self.get_logger().info("Nearby the last waypoint.")
                return
                
    
    def run(self):
        if self._mode_ == "SLAM":
            self.waitUntilNav2Active(localizer="amcl")
        elif self._mode_ == "GPS":
            self.waitUntilNav2Active(localizer="robot_localization")
        else :
            self.get_logger().error("Unavailable mode:[{}] set!!!".format(self._mode_))
            return
        
        self.get_logger().info("ready to go")
        
        while rclpy.ok():
            waypoints = []
                   
            waypoint_remain = len(self._waypoints_['waypoints'])
            if waypoint_remain > self._waypoints_size_:
                for index in range(self._waypoints_size_):
                    waypoints.append(self._waypoints_['waypoints'].pop(0))
            else:
                for index in range(waypoint_remain):
                    waypoints.append(self._waypoints_['waypoints'].pop(0))
                    self._now_mode_.is_last_point = True

            if self._mode_ == "SLAM":
                goal_poses = self.get_poses(waypoints)
            elif self._mode_ == "GPS":
                goal_poses = self.get_geo_poses(waypoints)
            
            if not self._now_mode_.is_last_point:
                self.whill_go(goal_poses)
            else:
                self.get_logger().info("Last Waypoints, and shutdown after navigation.")
                self.whill_go(goal_poses)
                return

def main(args=None):
    rclpy.init(args=args)
    node = WhillNavi2Node()
    node.run()
    # rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
