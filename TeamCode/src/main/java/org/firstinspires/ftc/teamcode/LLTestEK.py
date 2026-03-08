#hi i am figuring out my life. pls don't touch. thank u.
from networktables import NetworkTables
import time
NetworkTables.initialize(server='limelight.local')
limelight = NetworkTables.getTable('limelight')
while True:
 # Get the ID of the currently detected AprilTag (-1 means none)
    tag_id = limelight.getNumber('tid', -1)
    # Check if the camera sees any target at all
    has_target = limelight.getNumber('tv', 0)  # 1 = sees target, 0 = no target
if has_target == 0:
        print("No target detected")
elif tag_id == 24:
        print("I can see tag 24")
else:
        print("I don't see shite")
  
