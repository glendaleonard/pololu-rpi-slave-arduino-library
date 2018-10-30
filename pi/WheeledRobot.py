'''
Created on May 20, 2018

@author: glendal
'''
import math


class WheeledRobot(object):
    '''
    An abstract class supporting ododmetry for wheeled
    robots. 
    Your 
    implementing class should provide the
    method:
    extractOdometry(self, timestamp, leftWheel, rightWheel)
    -->
    (timestampSeconds, leftWheelDegrees,
    rightWheelDegrees)      
    '''
    
    
    def __init__(self, wheelRadiusMeters, halfAxleLengthMeters):
        '''
        wheelRadiusMeters - radius of each odometry wheel, in meters        
        halfAxleLengthMeters - half the length of the axle between the odometry wheels, in meters
        '''
        self.wheelRadiusMeters = wheelRadiusMeters
        self.halfAxleLengthMeters = halfAxleLengthMeters
        self.timestampSecondsPrev = None
        self.leftWheelDegreesPrev = None
        self.rightWheelDegreesPrev = None
        
        
    def computeVelocities(self, timestamp, leftWheelOdometry, rightWheelOdometry):
        '''
        Computes forward and angular velocities based on
        odometry.
        Parameters: timestamp - time stamp, in whatever units your robot uses  
        leftWheelOdometry - odometry for left wheel, in whatever units your robot uses       
        rightWheelOdometry - odometry for right wheel, in whatever units your robot uses
        Returns a tuple (dxyMeters, dthetaDegrees, dtSeconds):
        dxyMeters - forward distance traveled, in meters
        dthetaDegrees - change in angular position, in degrees
        dtSeconds - elapsed time since previous odometry, in seconds
        '''
        dxyMeters = 0
        dthetaDegrees = 0
        dtSeconds = 0
        timestampSecondsCurr, leftWheelDegreesCurr, rightWheelDegreesCurr = self.extractOdometry(timestamp, leftWheelOdometry, rightWheelOdometry)
        if self.timestampSecondsPrev != None:
            leftDiffDegrees = leftWheelDegreesCurr - self.leftWheelDegreesPrev
            rightDiffDegrees = rightWheelDegreesCurr - self.rightWheelDegreesPrev
            dxyMeters = (math.radians(leftDiffDegrees) + math.radians(rightDiffDegrees)) / 2
            dthetaDegrees = (rightDiffDegrees -leftDiffDegrees) / (2 * self.halfAxleLengthMeters)
            dtSeconds = timestampSecondsCurr - self.timestampSecondsPrev
        # Store current odometry for next time
        self.timestampSecondsPrev = timestampSecondsCurr        
        self.leftWheelDegreesPrev = leftWheelDegreesCurr
        self.rightWheelDegreesPrev = rightWheelDegreesCurr
        # Return linear velocity, angular velocity, time difference
        return dxyMeters, dthetaDegrees, dtSeconds