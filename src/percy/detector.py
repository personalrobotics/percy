PACKAGE = 'percy'
import roslib; roslib.load_manifest(PACKAGE)
import rospy
import atexit
import visualization_msgs.msg as vm
import yaml
import pose
import time
import numpy
from tf.transformations import quaternion_matrix

DETECTIONS_MSG_TYPE = vm.MarkerArray

open_nodes = {}

def detect_callback(data,instance):
    instance.detections = data

class Detector(object):
    def __init__(self, nodename='apriltags_prosilica',
                 publish_topic='marker_array'):
        self.nodename = nodename
        self.publish_topic = publish_topic
        self.start_id = 0
        self.detections = None
        #rospy.init_node('detector',anonymous=True)#GENERALIZE LATER
     
    def detect(self,n_iters=10, timeout=15):
        
        ros_detections = []

        def callback(detection_data):
            ros_detections.append(detection_data)
            if len(ros_detections) >= n_iters:
                #Kill Subscriber
                subscriber.unregister()

        time_start = time.time()
        subscriber = rospy.Subscriber(self.nodename + '/' + self.publish_topic,
                                      vm.MarkerArray,callback)  
        
        while(len(ros_detections))<n_iters:
            time.sleep(0.03)
            if time.time() - time_start > timeout:
                raise Exception, 'TIMEOUT'

        tag_detections = {}
        i = 0
        for detection in ros_detections:
            for tag in detection.markers:
                tag_ns = tag.ns
                tag_id = tag.id
                tag_detections.setdefault((tag_ns,tag_id),[])
                position_data = tag.pose.position
                orientation_data = tag.pose.orientation
                #print 'p', position_data
                #print 'o', orientation_data
                #print orientation_data - G
                tag_pose = numpy.matrix(quaternion_matrix([orientation_data.x,
                                                           orientation_data.y,
                                                           orientation_data.z,
                                                           orientation_data.w]))
                #print tag_pose
                tag_pose[0,3] = position_data.x
                tag_pose[1,3] = position_data.y
                tag_pose[2,3] = position_data.z
                #print 'tp', tag_pose
                
                tag_detections[(tag_ns,tag_id)].append(tag_pose)
        
        return tag_detections
    
    def filter_detections(self,
                          raw_detections,
                          min_detections = 3,
                          angle_inlier = 0.1,
                          distance_inlier = 0.05):
        detections = {}
        inlier_ratios = {}
        for tag in raw_detections:
            if len(raw_detections[tag])>=min_detections:
                ransac_result = pose.ransac_pose_estimate(
                        raw_detections[tag],
                        angle_inlier = angle_inlier,
                        distance_inlier = distance_inlier)
                '''
                filtered_pose, inlier_ratio, rv, tv = (
                        pose.ransac_pose_estimate(
                        raw_detections[tag],
                        angle_inlier = angle_inlier,
                        distance_inlier = distance_inlier))
                '''
                detections[tag] = ransac_result
                
        return detections
