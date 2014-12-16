PACKAGE = 'percy'
import roslib; roslib.load_manifest(PACKAGE)
import detector
import prpy
import os
import tf
import rospy
import detector
import numpy

class KinBodyDetector(object):
    def __init__(self, env, prefix, topic, directory, from_frame, to_frame):
        self.env = env
        self.directory = directory
        self.from_frame = from_frame
        self.to_frame = to_frame
        
        rospy.init_node('kinbody_detector', anonymous=True)
        
        self.detector = detector.Detector(prefix, topic)
        self.listener = tf.TransformListener()
    
    def update(self):
        detections = self.detector.detect(n_iters=5, timeout=20)
        filtered_detections = self.detector.filter_detections(detections,
                                                              min_detections=1,
                                                              angle_inlier=0.3)
        try:
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
            update_kinbodies(self.env, filtered_detections, self.directory, 
                             offset)

        except (tf.LookupException,
                tf.ConnectivityException,
                tf.ExtrapolationException):
            print 'WARNING, unable to get TF offset'
            raise

def update_kinbodies(env, detections, directory, to_frame):
    
    with env:
        for tag_ns, tag_id in detections:
            print tag_ns, tag_id
            tag_name = tag_ns
            existing_body = env.GetKinBody(tag_name)
            print existing_body
            if existing_body is None:
                try:
                    kinbody_path = os.path.join(
                            directory, tag_name + '.kinbody.xml')
                    print kinbody_path
                    existing_body = prpy.rave.add_object(
                            env,
                            tag_name,
                            kinbody_path)
                except Exception:
                    print 'WARNING: Unable to load kinbody "%s"'%tag_name
                    continue
            
            tag_transform = to_frame * detections[(tag_ns,tag_id)]['mean']
            existing_body.SetTransform(tag_transform.tolist())
        
        for kinbody in env.GetBodies():
            name = kinbody.GetName()
            if name[0:3] == 'tag':
                number = int(name.split('_')[0][3:])
                if ('tag%i'%number, number) not in detections:
                    env.Remove(kinbody)
