'''
Created on May 20, 2018

@author: glendal
'''


self.r = 0.077  # wheel radius
self.R = 0.165  # half axle length (1/2 the distance between the wheels)

#according to encode docs (https://www.pololu.com/product/3542) 12 counts per revolution of the motor shaft.
self.inc = 2000  # counter inc per turn (the number of ticks a wheel makes in one complete rotation)
self.ratio = 1.0  # left/right wheel ratio
self.m = 2 *self.r * pi / self.inc  #the distance the wheel moves per click

self.wheelRadiusMeters = 0.01525
self.halfAxleLengthMeters = 0.04
def computeVelocitiesOrig(self, odometry_prev, odometry_curr):
    # the distance the robot travels in a second, is assigned to be the average of the distance covered by each of the wheels
    dxy = (odometry_curr.q1 - odometry_prev.q1 + odometry_curr.q2 - odometry_prev.q2) / 2 
    dtheta = ((odometry_prev.q2 - odometry_curr.q2) - (odometry_prev.q1 - odometry_curr.q1)) / 2 * self.halfAxleLengthMeters
    
    #q1 left wheel
    #q2 right wheel