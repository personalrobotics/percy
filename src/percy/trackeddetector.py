PACKAGE = 'percy'
import roslib; roslib.load_manifest(PACKAGE)

class TrackedDetector(object):
    def __init__(self, detectors, tracker):
        self.detectors = detectors
        self.tracker = tracker
    
    def Start():
        for detector in self.detectors:
            detector.Start()
        
        self.tracker.Start()
        self.tracker.SetSubcribeTopics(
                [d.publish_topic for d in self.detectors])
    
    def Stop():
        for detector in self.detectors:
            detector.Stop()
        
        self.tracker.Stop()
    
    
