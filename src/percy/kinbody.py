PACKAGE = 'percy'
import roslib; roslib.load_manifest(PACKAGE)
import detector
import prpy
import os
import tf
import rospy
import detector
import numpy
import pose

class KinBodyDetector(object):
    def __init__(self,
                 env,
                 prefix,
                 topic,
                 model_directory,
                 from_frame,
                 to_frame):
        
        self.env = env
        self.model_directory = model_directory
        self.from_frame = from_frame
        self.to_frame = to_frame
        
        rospy.init_node('kinbody_detector', anonymous=True)
        
        self.detector = detector.Detector(prefix, topic)       
        self.listener = tf.TransformListener()
    '''
    def get_unique_objects(self, detections, r_thresh, t_thresh):
        unique_objects = {}
        for tag_name, tag_id in detections:
            unique_objects.setdefault(tag_name, [])
            
            # Iterate through all detections with the same tag name and id
            for detection in detections[(tag_name, tag_id)]:
                
                # Now see if belongs to any already known unique objects
                unique_index = None
                for i, unique_object in enumerate(unique_objects[tag_name]):
                    for element in unique_object:
                        angle, dist = pose.pose_similarity(detection, element)
                        if angle < r_thresh and dist < t_thresh:
                            unique_index = i
                            break
                    else:
                        continue
                    
                    break
                if unique_index is None:
                    unique_objects[tag_name].append([detection])
                else:
                    unique_objects[tag_name][unique_index].append(detection)
        
        averaged_unique_objects = {}
        for tag_name in unique_objects:
            averaged_unique_objects.setdefault([])
            for unique_object in unique_objects[tag_name]:
                averaged_unique_objects[tag_name] = pose.mean(unique_object)
    '''
    
    def update(self):
        
        detections = self.detector.detect(n_iters=5, timeout=30)
        
        filtered_detections = self.detector.filter_detections(detections,
                                                              min_detections=1,
                                                              angle_inlier=0.3)
        
        try:
            # Get the transform between world and head
            self.listener.waitForTransform(self.from_frame,
                                           self.to_frame,
                                           rospy.Time(),
                                           rospy.Duration(10.0))
            trans, rot = self.listener.lookupTransform(self.from_frame,
                                                       self.to_frame,
                                                       rospy.Time(0))
            
            offset = numpy.matrix(tf.transformations.quaternion_matrix(rot))
            offset[0,3] = trans[0]
            offset[1,3] = trans[1]
            offset[2,3] = trans[2]
            offset = self.env.GetRobot('herb').GetTransform()*offset
            
            #update_kinbodies(self.env, filtered_detections, self.directory, 
            #                 offset)
            
            with self.env:
                for tag_ns, tag_id in detections:
                    tag_name = 'tag_%s_%i'%(tag_ns, tag_id)
                    existing_body = self.env.GetKinBody(tag_name)
                    if existing_body is None:
                        try:
                            print 'NS', tag_ns
                            kinbody_path = os.path.join(
                                    self.model_directory,
                                    tag_ns + '.kinbody.xml')
                            existing_body = prpy.rave.add_object(
                                    self.env,
                                    tag_name,
                                    kinbody_path)
                        except Exception:
                            print ('WARNING: Unable to load kinbody "%s"'
                                   %tag_name)
                            continue
                    
                    tag_transform = (offset *
                            filtered_detections[(tag_ns,tag_id)]['mean'])
                    
                    existing_body.SetTransform(tag_transform.tolist())
                
                for kinbody in self.env.GetBodies():
                    name = kinbody.GetName()
                    if name[0:3] == 'tag':
                        _, body_type, tag_id = name.split('_')
                        if (body_type, int(tag_id)) not in filtered_detections:
                            self.env.Remove(kinbody)
        
        except (tf.LookupException,
                tf.ConnectivityException,
                tf.ExtrapolationException):
            print 'WARNING, unable to get TF offset'
            raise
