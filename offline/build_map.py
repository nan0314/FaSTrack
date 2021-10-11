#!/usr/bin/env python3

import numpy as np
import yaml
import matplotlib.pyplot as plt
import matplotlib.image as mpimg


xx, yy = np.mgrid[:100, :100]
circle = (xx-50)**2 + (yy-50)**2
donut  = (circle < (2500 + 50))


def circle_obstacle(x,y,r,map,num=10**8):
    xx,yy = np.mgrid[-x:map.shape[0]-x, -y:map.shape[1]-y]
    mask = xx*xx + yy*yy <= r*r
    map[mask] = num

def rect_obstacle(x1,x2,y1,y2,map,num=10**8):
    xlow = min(x1,x2)
    xhigh = max(x1,x2)
    ylow = min(y1,y2)
    yhigh = max(y1,y2)

    yy, xx = np.mgrid[ range(map.shape[0]), range(map.shape[1])]
    mask = np.logical_and.reduce([xx >= xlow, xx <= xhigh, yy >= ylow, yy <= yhigh])
    map[mask] = num


# initialize map
map = np.zeros((500,500))

# map attributes
attributes = {"width" : 25, "height" : 25, "dl" : 25/map.shape[0]}

# Add obstacles
circle_obstacle(250,250,100,map)

rect_obstacle(70,170,100,120,map)

rect_obstacle(80,190,420,440,map)
rect_obstacle(125, 145, 440,340,map)

rect_obstacle(370,470,324,336,map)
rect_obstacle(420-6,426,280,380,map)

imgplot = plt.imshow(map,cmap=None)
plt.show()

np.savetxt("../online/src/planner/config/map.csv", map, delimiter=",")
with open("../online/src/planner/config/map_attributes.yaml", "w") as fh:  
  yaml.dump(attributes, fh)
