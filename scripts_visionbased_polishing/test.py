import sys

import os


# o_path = os.getcwd() 
o_path="/data/ros/yue_ws_201903/src/visionbased_polishing"
print(o_path)
sys.path.append(o_path) 

# from scripts_arm.frompitoangle import *

Q9=[1.4794633333333334, -2.17445, -1.3624111111111112, 1.7372922222222222, -1.6854822222222223, 1.5698255555555556]
# display(getangle(Q9))

a=(0.1,0.2)
b=[]
b.append(int(a[0]))
b.append(int(a[1]))
b=tuple(b)
print(b)