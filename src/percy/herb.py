PACKAGE = 'percy'
import roslib; roslib.load_manifest(PACKAGE)
import apriltags.apriltags as apriltags
import roslaunch

cache_topics = ['/apriltags/marker_array','/singletracker/tracked_marker']

class HerbPerception(object):
    def __init__(self):
        self.apriltags = apriltags.AprilTags()
        self.apriltags_singletracker = singletracker.SingleTracker()
    
    def StartTrack(mode, instance_id):
        if mode == 'apriltags':
            self.singletracker.Start(instance_id)
        
        if mode == 'moped':
            self.singletracker.SetTopic('/moped/marker_array')
            self.singletracker.Start(instance_id)
        
        # etc
    
    
    
