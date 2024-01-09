import rospy
from udi_msgs.msg import PerceptionObjects
from udi_msgs.msg import PerceptionObject
from private_msgs.msg import GnssCoords
from tracker.tracker import Tracker3D
from udi_msgs.msg import PredictionObstacles
from udi_msgs.msg import PredictionObstacle
from udi_msgs.msg import LocalizationEstimate
import numpy as np
import copy

from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Pose, Point
from std_msgs.msg import ColorRGBA

import time
import math

class Config:
    def __init__(self):
        # tracking type
        self.tracking_type = "Car"

        # KF parameters
        self.state_func_covariance = 100
        self.measure_func_covariance = 0.001
        self.prediction_score_decay = 0.03
        self.LiDAR_scanning_frequency = 10

        # max prediction number of state function
        self.max_prediction_num = 12
        self.max_prediction_num_for_new_object = 2

        # detection score threshold
        self.input_score = 0
        self.init_score = 2
        self.update_score = 0
        self.post_score = 0
        

        # tracking latency (s)
        # -1= global tracking
        # 0.->500= online or near online tracking
        self.latency = 0

class UdiTracker():

    def __init__(self):
        self.config = Config()
        self.tracker = Tracker3D(box_type="Kitti", tracking_features=False, config = self.config)
        self.frame_num = 0
        #self.objects = None
        self.objects = np.array([])
        self.objects_data = None
        self.det_scores = None
        self.have_new_object = False
        self.have_new_pose = False
        #self.pose = np.array([0,0,0,0,0,0,1])
        self.pose = None
        self.pose2 = None
        self.pose_localization_estimate = None
        self.bbs = []
        self.ids = []
        self.velocities = []

    def convert_perception_objects_to_prediction_obstacles(self, data):
        vehicle_position = self.pose2.pose.position
        vehicle_orientation = self.pose2.pose.orientation
        # Convert each PerceptionObject in the PerceptionObjects message to a PredictionObstacle
        for perception_object in data.perception_object:
            # 获取障碍物相对于车辆的位置和朝向
            obstacle_position = perception_object.position
            obstacle_orientation = perception_object.theta

            # 进行坐标转换
            # 首先，将障碍物的位置从车辆坐标系转换为世界坐标系
            world_x = vehicle_position.x + obstacle_position.x * math.cos(vehicle_orientation.z) - obstacle_position.y * math.sin(vehicle_orientation.z)
            world_y = vehicle_position.y + obstacle_position.x * math.sin(vehicle_orientation.z) + obstacle_position.y * math.cos(vehicle_orientation.z)
            world_z = vehicle_position.z + obstacle_position.z

            # 其次，将障碍物的朝向从车辆坐标系转换为世界坐标系
            world_theta = vehicle_orientation.z + obstacle_orientation

            perception_object.theta = world_theta
            perception_object.position.x = world_x
            perception_object.position.y = world_y
            perception_object.position.z = world_z
    
    def convert_result_to_global_prediction_obstacles_and_pub(self):
        data = self.objects_data
        ids = self.ids
        velocities = self.velocities
        print('ids:',ids,'object_size:',len(self.objects_data.perception_object))
        vehicle_position = self.pose2.pose.position
        vehicle_orientation = self.pose2.pose.orientation
        vehicle_heading = self.pose2.pose.heading
        qx,qy,qz,qw = vehicle_orientation.qx,vehicle_orientation.qy,vehicle_orientation.qz,vehicle_orientation.qw
        vehicle_heading = math.atan2(2*qx*qy+2*qw*qz,qw*qw+qx*qx-qy*qy-qz*qz)

        # Convert each PerceptionObject in the PerceptionObjects message to a PredictionObstacle
        prediction_obstacles = PredictionObstacles()
        print('99:',data.header)
        prediction_obstacles.header = data.header
        print('101:',prediction_obstacles.header)
        now_object = -1
        for perception_object in data.perception_object:
            now_object += 1
            # 获取障碍物相对于车辆的位置和朝向
            obstacle_position = perception_object.position
            obstacle_orientation = perception_object.theta

            # 进行坐标转换
            # 首先，将障碍物的位置从车辆坐标系转换为世界坐标系
            world_x = vehicle_position.x + obstacle_position.x * \
                math.cos(vehicle_heading) - \
                obstacle_position.y * math.sin(vehicle_heading)
            world_y = vehicle_position.y + obstacle_position.x * \
                math.sin(vehicle_heading) + \
                obstacle_position.y * math.cos(vehicle_heading)
            world_z = vehicle_position.z + obstacle_position.z

            # 其次，将障碍物的朝向从车辆坐标系转换为世界坐标系
            world_theta = vehicle_heading + obstacle_orientation

            tmp_prediction_obstacle = PredictionObstacle()
            new_perception_object = copy.deepcopy(perception_object)
            new_perception_object.theta = world_theta
            new_perception_object.position.x = world_x
            new_perception_object.position.y = world_y
            new_perception_object.position.z = world_z

            print('131:',velocities[now_object])
            new_perception_object.velocity.x = velocities[now_object][0]
            new_perception_object.velocity.y = velocities[now_object][1]
            new_perception_object.velocity.z = velocities[now_object][2]

            new_perception_object.id = ids[now_object]
            for polygon_point in new_perception_object.polygon_point:
                point_world_x = vehicle_position.x + polygon_point.x * \
                    math.cos(vehicle_heading) - \
                    polygon_point.y * math.sin(vehicle_heading)
                point_world_y = vehicle_position.y + polygon_point.x * \
                    math.sin(vehicle_heading) + \
                    polygon_point.y * math.cos(vehicle_heading)
                point_world_z = vehicle_position.z + polygon_point.z

                polygon_point.x = point_world_x
                polygon_point.y = point_world_y
                polygon_point.z = point_world_z

            tmp_prediction_obstacle.perception_object = new_perception_object
            tmp_prediction_obstacle.timestamp = perception_object.timestamp
            prediction_obstacles.header.timestamp_sec = perception_object.timestamp # todo fix, remove this after xiaoming add timestamp to PerceptionObjects
            prediction_obstacles.prediction_obstacle.append(tmp_prediction_obstacle)
        print('117 pub prediction_obstacles!!')
        self.prediction_obstacles_pub.publish(prediction_obstacles)

    def convert_objects_to_kitti(self, data):
        tmp_objects = []
        det_scores = []

        vehicle_position = self.pose2.pose.position
        vehicle_orientation = self.pose2.pose.orientation
        vehicle_heading = self.pose2.pose.heading
        qx,qy,qz,qw = vehicle_orientation.qx,vehicle_orientation.qy,vehicle_orientation.qz,vehicle_orientation.qw
        vehicle_heading = math.atan2(2*qx*qy+2*qw*qz,qw*qw+qx*qx-qy*qy-qz*qz)


        for perception_object in data.perception_object:


            x = perception_object.position.x
            y = perception_object.position.y
            z = perception_object.position.z
            length = perception_object.length
            width = perception_object.width
            height = perception_object.height
            
            # to world to meet the demand of velocity estimator
            yaw = perception_object.theta

            world_x = vehicle_position.x + x * \
                math.cos(vehicle_heading) - \
                y * math.sin(vehicle_heading)
            world_y = vehicle_position.y + x * \
                math.sin(vehicle_heading) + \
                y * math.cos(vehicle_heading)
            world_z = vehicle_position.z + z

            # 其次，将障碍物的朝向从车辆坐标系转换为世界坐标系
            world_theta = vehicle_heading + yaw

            tmp_objects.append(np.array([world_x,world_y,world_z,length,width,height,world_theta]))
            det_scores.append(100)
        return np.array(tmp_objects),np.array(det_scores)

    def convert_pose_to_kitti(self, data):
        tmp_pose = 0
        return tmp_pose

    
    def objects_callback(self, data):
        #if not self.have_new_pose: ???
        #    print("!")
        #    return
        print("@")
        self.objects_data = data

        if self.have_new_pose:
            self.objects, self.det_scores = self.convert_objects_to_kitti(data)
            self.have_new_object = True
        if ((len(self.objects) == 0)):
            self.have_new_object = False
        else:
            self.have_new_object = True
        self.run_track()
        #self.convert_perception_objects_to_prediction_obstacles(data)
        #self.prediction_pub.publish(data)
        #print(data)

    def localization_callback(self,data):
        self.pose2 = data
        self.have_new_pose = True
        return
    
    def localization_estimate_callback(self,data):
        self.pose2 = data
        self.pose_localization_estimate = data
        self.have_new_pose = True
        return

    def run_track(self):
        print('165:self.have_new_pose:',self.have_new_pose)
        print('165:self.have_new_object:',self.have_new_object)
        if self.have_new_object and self.have_new_pose:
            self.frame_num += 1
            self.bbs,self.ids,self.velocities = self.tracker.tracking(self.objects[:, :7],
                            features=None,
                            scores=self.det_scores,
                            pose=self.pose,
                            timestamp=self.frame_num)

            self.convert_result_to_global_prediction_obstacles_and_pub()
            self.have_new_object = False
            self.have_new_pose = True

            #print('bbs:',self.bbs,'ids:',self.ids)
    
    def vis(self):
        

        bounding_boxes = self.bbs
        ids = self.ids
        marker_array = MarkerArray()

        for i in range(len(bounding_boxes)):
            # Set the pose of the marker to the center of the bounding box
            marker = Marker()
            marker.header.frame_id = "robosense_first"
            marker.type = Marker.CUBE
            marker.action = Marker.ADD
            marker.scale.x = 1
            marker.scale.y = 1
            marker.scale.z = 1
            marker.color.a = 0.5
            marker.color.r = 1.0
            marker.color.g = 0.0
            marker.color.b = 0.0
            marker.lifetime = rospy.Duration(0.1)
            pose = Pose()
            pose.position.x = bounding_boxes[i][0]
            pose.position.y = bounding_boxes[i][1]
            pose.position.z = bounding_boxes[i][2]
            pose.orientation.w = math.cos(bounding_boxes[i][6]/2)
            pose.orientation.x = 0
            pose.orientation.y = 0
            pose.orientation.z = math.sin(bounding_boxes[i][6]/2)
            marker.pose = pose

            # Set the scale of the marker to the size of the bounding box
            marker.scale.x = bounding_boxes[i][3]
            marker.scale.y = bounding_boxes[i][4]
            marker.scale.z = bounding_boxes[i][5]


            # Set the id of the marker
            marker.id = i+1

            # Add the marker to the marker array
            marker_array.markers.append(marker)

            # Create a text marker to display the id
            text_marker = Marker()
            text_marker.header.frame_id = "robosense_first"
            text_marker.type = Marker.TEXT_VIEW_FACING
            text_marker.action = Marker.ADD
            text_marker.scale.z = 5
            text_marker.color.a = 1.0
            text_marker.color.r = 1.0
            text_marker.color.g = 1.0
            text_marker.color.b = 1.0
            text_marker.pose.position.x = bounding_boxes[i][0]
            text_marker.pose.position.y = bounding_boxes[i][1]
            text_marker.pose.position.z = bounding_boxes[i][2] + bounding_boxes[i][5]/2 + 0.5
            text_marker.text = str(ids[i])
            text_marker.lifetime = rospy.Duration(0.1)

            # Set the id of the text marker
            text_marker.id = i+len(bounding_boxes)+1

            # Add the text marker to the marker array
            marker_array.markers.append(text_marker)

        self.marker_pub.publish(marker_array)


    def listener(self):
        rospy.init_node('udi_castrack', anonymous=True)
        rospy.Subscriber('/perception/detOBB', PerceptionObjects, self.objects_callback)
        rospy.Subscriber('/gnss_coords', GnssCoords, self.localization_callback)
        rospy.Subscriber('/localization_estimate', LocalizationEstimate, self.localization_estimate_callback)
        self.marker_pub = rospy.Publisher('/bounding_boxes', MarkerArray, queue_size=1)
        #self.prediction_pub = rospy.Publisher('/perception_objects_world', PerceptionObjects, queue_size=1)
        self.prediction_obstacles_pub = rospy.Publisher('/prediction_obstacles', PredictionObstacles, queue_size=1)

        rate = rospy.Rate(10) # 设置循环速率为10Hz
        while not rospy.is_shutdown():
            #self.run_track()
            self.vis()
            rate.sleep() # 按照所设置的频率暂停循环

if __name__ == '__main__':

    udi_tracker = UdiTracker()
    udi_tracker.listener()


# todo 2023.09.18
'''
一、主要流程
1.把定节的话题接进来，然后把输出的障碍物装换到世界坐标系
2.把障碍物转换成udi_msgs的PredictionObstacles类型输出

二、和感知联调试
1.和感知对 det_scores 这个置信度，目前全都给的100 (吴晓民)
2.和感知对部署到统一台电脑上测试
3.看可视化是否结果正确，感知是否结果跳变很多

三、调参
1.调一些参数使得效果更好
2.如果有数据集 在数据集上eval这个算法

四、杂项
1.改cmakelist 使得能catkin_make install 和 roslaunch
目前只能 python3 ./src/3D-Multi-Object-Tracker/udi_castrack.py 启动
2.把python3程序打成deb包部署上车的方法 
'''
