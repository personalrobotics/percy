#!/usr/bin/python

import detector

dd = detector.Detector()
visible_tags = dd.detect()

print visible_tags
