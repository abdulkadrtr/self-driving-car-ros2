import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu , LaserScan, Image
from ackermann_msgs.msg import AckermannDrive
from geometry_msgs.msg import PointStamped, PoseStamped
import math , json , torch , cv2
import numpy as np
import torch.nn as nn
from cv_bridge import CvBridge
from ultralytics import YOLO
from nav_msgs.msg import Path
import os

dizin = os.getcwd() + "/src/autonomous_controller/config/"

waypoints = "path1.csv"

LOOK_HEAD_DISTANCE = 6.0

# Sınıf numaraları
# yesil-isik 3
# kirmizi-isik 2
# hiz-limiti 1
# dur 0

# Bu class sınıfını colcon build ile derleme yaptıktan sonra 
# install/autonomous_controller/lib/autonomous_controller/autonomous_controller.py dosyasına kopyalayın
class NeuralNetwork(nn.Module):
    def __init__(self):
        super(NeuralNetwork, self).__init__()
        self.layer1 = nn.Linear(6, 32)
        self.layer2 = nn.Linear(32, 16)
        self.output = nn.Linear(16, 2)

    def forward(self, x):
        x = torch.relu(self.layer1(x))
        x = torch.relu(self.layer2(x))
        x = self.output(x)
        return x

def detect(image_path, model,c):
    flag = 0
    results = model.predict(image_path, imgsz=640, conf=c, iou=0.45, verbose=False)
    json_object = results[0].tojson()
    json_array = json.loads(json_object)
    for item in json_array:
        if item["class"] == 2:
            flag = 1
            break
    annotated_frame = results[0].plot()
    w , h = 800, 600
    annotated_frame = cv2.resize(annotated_frame, (w, h))
    return flag, annotated_frame

def euler_from_quaternion(x,y,z,w):
    t0 = +2.0 * (w * x + y * z)
    t1 = +1.0 - 2.0 * (x * x + y * y)
    roll_x = math.atan2(t0, t1)
    t2 = +2.0 * (w * y - z * x)
    t2 = +1.0 if t2 > +1.0 else t2
    t2 = -1.0 if t2 < -1.0 else t2
    pitch_y = math.asin(t2)
    t3 = +2.0 * (w * z + x * y)
    t4 = +1.0 - 2.0 * (y * y + z * z)
    yaw_z = math.atan2(t3, t4)
    return yaw_z

def dx_dy_calc(x_location, y_location, path):
    global LOOK_HEAD_DISTANCE
    look_head_distance = LOOK_HEAD_DISTANCE
    closest_point_index = 0
    min_distance = float('inf')
    for i, (x,y) in enumerate(path):
        distance = math.sqrt((x - x_location)**2 + (y - y_location)**2)
        if distance < min_distance:
            min_distance = distance
            closest_point_index = i
    look_ahead_index = closest_point_index
    while look_ahead_index < len(path):
        x, y = path[look_ahead_index]
        distance = math.sqrt((x - x_location)**2 + (y - y_location)**2)
        if distance > look_head_distance:
            break
        look_ahead_index += 1
        if look_ahead_index == len(path):
            break
    if look_ahead_index == len(path):
        dx = 0.0
        dy = 0.0
    else:
        target_x, target_y = path[look_ahead_index]
        dx = target_x - x_location
        dy = target_y - y_location
    return dx, dy

def normalize_angle(angle):
    while angle > math.pi:
        angle -= 2 * math.pi
    while angle < -math.pi:
        angle += 2 * math.pi
    return angle

def obstacle_detect(min, limit):
    if min < limit:
        return 1
    else:
        return 0

def prepare_waypoints(waypoints):
    path = []
    with open(waypoints, 'r') as file:
        path = np.loadtxt(file, delimiter=',')
    new_x = np.interp(np.arange(0, len(path), 0.3), np.arange(0, len(path)), path[:, 0])
    new_y = np.interp(np.arange(0, len(path), 0.3), np.arange(0, len(path)), path[:, 1])
    path = list(zip(new_x, new_y))
    return path

class OtonomSurus(Node):
    def __init__(self):
        super().__init__('otonom_surus')
        self.subscription = self.create_subscription(Imu,'/imu',self.imu_callback,10)
        self.publisher_cmd = self.create_publisher(AckermannDrive, '/cmd_ackermann', 10)
        self.subscription = self.create_subscription(PointStamped,'/vehicle/gps',self.gps_callback,10)
        self.subscription = self.create_subscription(LaserScan,'/vehicle/lidar_on',self.lidar_front_callback,10)
        self.subscription = self.create_subscription(LaserScan,'/vehicle/lidar_sag',self.lidar_right_callback,10)
        self.subscription = self.create_subscription(LaserScan,'/vehicle/lidar_sol',self.lidar_left_callback,10)
        self.subscription = self.create_subscription(Image,'/vehicle/camera/image_color',self.camera_callback,10)
        self.publisher_cam = self.create_publisher(Image, '/vehicle/camera/detect_cam', 10)
        self.publisher_route = self.create_publisher(Path, '/vehicle/route/waypoints', 10)
        self.publisher_location = self.create_publisher(Path, '/vehicle/location', 10)

        self.path = prepare_waypoints(dizin+waypoints)

        self.timer = self.create_timer(0.1, self.timer_callback)
        self.speed = None
        self.steering_angle = 0.0
        
        self.x = None
        self.y = None

        self.obstacle_front = False
        self.obstacle_right = False
        self.obstacle_left = False
        self.traffic_light = 0
        
        self.dx = None
        self.dy = None

        self.path_location = Path()
        self.path_location.header.frame_id = 'map'

        self.model = torch.load(dizin+'otonom_model.pth')
        self.model.eval()

        self.bridge = CvBridge()
        self.yolo_model = YOLO(dizin+'yolov8_s.pt')
        self.conf = 0.25
        self.img = None

        self.timer2 = self.create_timer(1.0, self.timer_callback2)
        self.get_logger().info('BASLATILIYOR..')

    
    def timer_callback(self):
        if self.x != None and self.yaw != None:
            self.dx, self.dy = dx_dy_calc(self.x, self.y, self.path)
            input = [self.yaw, self.dx, self.dy, math.atan2(self.dy, self.dx), self.obstacle_front , self.traffic_light]
            input_tensor = torch.tensor(input, dtype=torch.float32).unsqueeze(0)
            with torch.no_grad():
                output = self.model(input_tensor)

            self.speed = output[0][0].item()
            print(self.speed)
            self.steering_angle = output[0][1].item()
        
            msg = AckermannDrive()
            msg.steering_angle = self.steering_angle
            msg.speed = self.speed
            self.publisher_cmd.publish(msg)
            if self.traffic_light == 1:
                self.get_logger().info('KIRMIZI ISIK')
            elif self.obstacle_front == 1:
                self.get_logger().info('ON ENGEL')
            else:
                self.get_logger().info('HIZ: %.2f, DIREKSIYON: %.2f' % (self.speed, math.degrees(self.steering_angle)))

            msg = PoseStamped()
            msg.pose.position.x = self.x*0.04
            msg.pose.position.y = self.y*0.04
            msg.header.frame_id = 'map'
            self.path_location.poses.append(msg)
            self.publisher_location.publish(self.path_location)

    def timer_callback2(self):
        self.traffic_light, annotated_frame = detect(self.img, self.yolo_model, self.conf)
        self.publisher_cam.publish(self.bridge.cv2_to_imgmsg(annotated_frame, "bgr8"))
        self.publish_waypoints()
        

    def publish_waypoints(self):
        msg = Path()
        msg.header.frame_id = 'map'
        for x, y in self.path:
            point = PoseStamped()
            point.header.frame_id = 'map'
            point.pose.position.x = x*0.04
            point.pose.position.y = y*0.04
            msg.poses.append(point)
        self.publisher_route.publish(msg)

    def camera_callback(self, msg):
        self.img = self.bridge.imgmsg_to_cv2(msg, "bgr8")

    def lidar_front_callback(self, msg):
        min_v = min(msg.ranges)
        limit = 6.0
        self.obstacle_front = obstacle_detect(min_v, limit)
    
    def lidar_right_callback(self, msg):
        min_v = min(msg.ranges)
        limit = 3.5
        self.obstacle_right = obstacle_detect(min_v, limit)
        
    def lidar_left_callback(self, msg):
        min_v = min(msg.ranges)
        limit = 4.3
        self.obstacle_left = obstacle_detect(min_v, limit)
        
    def imu_callback(self, msg):
        self.yaw = euler_from_quaternion(msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w)

    def gps_callback(self, msg):
        self.x = msg.point.x
        self.y = msg.point.y



def main(args=None):
    rclpy.init(args=args)
    otonom_surus = OtonomSurus()
    rclpy.spin(otonom_surus)
    otonom_surus.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

        