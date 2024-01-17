from gps_wp_pkg.waypoint_utils.utils import *
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
        self._waypoints_list_ = []
        self._now_mode_ = NaviMode()
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
        self._waypoints_number_ = self.declare_parameter(
            'waypoints_number', 3
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
                
        with open(self._read_file_, 'r') as f:
            wp_tmp = Waypoint()
            waypoints_data = ruamel.yaml.safe_load(f)["waypoints"]
            
            for waypoint in waypoints_data:
                wp_tmp.pose.position.x = waypoint['pose_x']
                wp_tmp.pose.position.y = waypoint['pose_y']
                wp_tmp.pose.position.z = waypoint['pose_z']
                wp_tmp.pose.orientation.x = waypoint['quat_x']
                wp_tmp.pose.orientation.y = waypoint['quat_y']
                wp_tmp.pose.orientation.z = waypoint['quat_z']
                wp_tmp.pose.orientation.w = waypoint['quat_w']
                wp_tmp.fix.longitude = waypoint['longitude']
                wp_tmp.fix.latitude = waypoint['latitude']
                wp_tmp.mode = waypoint['mode']
                
                self._waypoints_list_.append(wp_tmp)
                
        self.get_logger().info("Get waypoint file")
        self._now_mode_.reference_waypoint_now = self._now_mode_.referenced_waypoint_past = self._start_point_
        self._now_mode_.reset()
        
        # self._sub_object_ = self.create_subscription()
        # self._pub_sound_ = self.create_publisher()
        # self._map_change_ = self.create_publisher()        
            
    def get_tf(self, 
               source_frame = str(), 
               target_frame = str(), 
               tf_stamp = Time()
        ) -> TransformStamped:
        
        if self._tf_buffer_.can_transform(target_frame, source_frame, tf_stamp, Duration(nanoseconds=int(1e8))):
            return self._tf_buffer_.lookup_transform(target_frame, source_frame, tf_stamp, Duration(nanoseconds=int(1e8)))
        else:
            self.get_logger().warn("waiting for TF...")
            self.get_clock().sleep_for(Duration(nanoseconds=(int(5e7))))
    
    def get_waypoints_mode(self) -> int:
        pass
    
    def need_resend_waypoints(result = TaskResult) -> bool:
        pass
    
    def calc_dist(waypoint = Waypoint, transform = TransformStamped) -> float:
        pass
    
    def waypoint_num_nearby(transform = TransformStamped) -> int:
        pass
    
    def get_goal_poses(self, waypoints = [Waypoint()]) -> [PoseStamped]:
        goal_poses = []
        goal_pose = PoseStamped()    
        for waypoint in waypoints:
            goal_pose.pose = waypoint.pose
            goal_pose.header.frame_id = 'map'
            goal_pose.header.stamp = self.get_clock().now().to_msg()
            goal_poses.append(goal_pose)
        
        return goal_poses
    
    def whill_go(self, goal_poses = [PoseStamped]):
        
        while self.followWaypoints(goal_poses):
            transform_now = self.get_tf("base_link", "map", Time(seconds=0, nanoseconds=0))
            dist = self.calc_dist(goal_poses[self._waypoints_number_ - 1], transform_now)
            if dist < self._arrive_distance_:
                self.cancelTask()
                self.get_logger().info("Nearby the last waypoint.")
                return
        
        self.get_clock().sleep_for(Duration(seconds=0, nanoseconds=int(1 / 30.0 * 1e9)))
    
    
    def run(self):
        self.waitUntilNav2Active(localizer="amcl")
        self.get_logger().info("ready to go")
        
        while rclpy.ok():
            waypoints = list[Waypoint]()
            transform_now = TransformStamped()
                   
            waypoint_reamin = len(self._waypoints_list_)
            if waypoint_reamin > self._waypoints_number_:
                for index in range(self._waypoints_number_):
                        waypoints.append(self._waypoints_list_.pop(0))
            else:
                for index in range(waypoint_reamin):
                    waypoints.append(self._waypoints_list_.pop(0))
                    self._now_mode_.is_last_point = True
                    
            goal_poses = self.get_goal_poses(waypoints)

            if not self._now_mode_.is_last_point:
                self.whill_go(goal_poses)
            else:
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
