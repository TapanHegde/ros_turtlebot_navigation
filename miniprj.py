#!/usr/bin/env python

import rospy   #python to ROS
import math    #importing math fuctions like sq root and pi
from geometry_msgs.msg import Twist   #to get vel fn 'cmd'
from goal_publisher.msg import PointArray #to receive goals which are published by goal publisher
from sensor_msgs.msg import LaserScan #to receive the laser data
from gazebo_msgs.msg import ModelStates #for receiving position
from location import Location, necessary_heading #importing the location class and its function
from dist import Dist #import the distance class from distance file
import tf.transformations as transform #import transformation to calculate euler angles
import sys #provides functions and variables used to manipulate different parts of the Python runtime environment.
global tx1, ty1, tx2, ty2, tx3, ty3  #declaring global variables for the goals
tx1 = 0 #intialization of global variables
ty1 = 0
tx2 = 0
ty2 = 0
tx3 = 0
ty3 = 0

current_location = Location() #initializing an object of location class
current_dists = Dist() #initializing an object of distance class

delta = .1 #max acceptance level for location error
WALL_PADDING = .5 #min to be maintained from the distance

STRAIGHT = 0 #states for the direction
LEFT = 1
RIGHT = 2
MSG_STOP = 3

def init_listener():     #function to intialise the node and subscribe to different topics
    rospy.init_node('mini_project', anonymous=True)
    rospy.Subscriber('gazebo/model_states', ModelStates, location_callback)
    rospy.Subscriber('scan', LaserScan, sensor_callback)
    rospy.Subscriber('goals', PointArray,goals_callback)

def goals_callback(data):   #callback fn for goals to assign the received goals to global variables
    global tx1, ty1, tx2, ty2, tx3, ty3
    tx1 = data.goals[0].x
    ty1 = data.goals[0].y
    tx2 = data.goals[1].x
    ty2 = data.goals[1].y
    tx3 = data.goals[2].x
    ty3 = data.goals[2].y


def location_callback(data): #callback fn to assign the received data to variables used in the program
    p = data.pose[1].position #to obtain the position data
    q = (
            data.pose[1].orientation.x,       #to copy the 4D quaterion data
            data.pose[1].orientation.y,
            data.pose[1].orientation.z,
            data.pose[1].orientation.w)
    t = transform.euler_from_quaternion(q)[2] #transforming the quaterion in eulers[-pi, pi]
    current_location.update_location(p.x, p.y, t)  #updating the values to current location object by using updtae location fn

def sensor_callback(data): #callback function to send laser data to current_dists object
    current_dists.update(data)

class Goal_one:
    global tx1, ty1
    def __init__(self): #intialising the goal one object

        self.pub = rospy.Publisher('cmd_vel', Twist, queue_size=1) #defining publisher to assign velocity data to turtlebot
        self.tx1 = tx1  #copying the global goal data into the class
        self.ty1 = ty1
        self.lh = None  #variable to store the orientation @the point of wall encounter
        self.encountered_wall_at = (None, None) #variable to store the location of wall encounter

    def go(self, direction): #function to move the robot
        cmd = Twist()
        if direction == STRAIGHT:
            cmd.linear.x = 0.5
        elif direction == LEFT:
            cmd.angular.z = 0.25
        elif direction == RIGHT:
            cmd.angular.z = -0.25
        elif direction == MSG_STOP:
            cmd.linear.x = 0
            cmd.angular.z = 0
        self.pub.publish(cmd) #publishing to the cmd velocity topic

    def go_until_obstacle(self): #fn to control the movement of the robot towards the goal/wall

        print "Going until destination or obstacle"
        while current_location.distance(tx1, ty1) > delta: #running the loop till we reach the destination
            (frontdist, _, _) = current_dists.get()  #copying the obstacle distance  infront of robot
            if frontdist <= WALL_PADDING:  #checking if there any obstacle infront of robot
                return True

            if current_location.facing_point(tx1, ty1):       #checking if robo is facing towards goal
                self.go(STRAIGHT)
            elif current_location.faster_left(tx1, ty1):  #checking if goal is towards left of robot
                self.go(LEFT)
            else:
                self.go(RIGHT)
            rospy.sleep(.01)  #delaying the loop for .01sec to facilitate movement of robot
        return False

    def follow_wall(self):  #fn to follow the wall
        print "Following wall"
        while current_dists.get()[0] <= WALL_PADDING:   #moving towards right till robot stops facing the wall
            self.go(RIGHT)
            rospy.sleep(.01)
        while not self.should_leave_wall():   #moving along the walll
            (front, left, right) = current_dists.get()  #copying the obstacle data
            if front <= WALL_PADDING:
                self.go(RIGHT)
            elif WALL_PADDING - .1 <= left <= WALL_PADDING + .1:   #move forward if there is an obstacle on the left of the robot within .5
                self.go(STRAIGHT)
            elif left > WALL_PADDING + .1: #realigning the robot towards the wall
                self.go(LEFT)
            elif left < 1 and right < 1: #facilitating robot to move straight inacse of obstacles @both sides
                self.go(STRAIGHT)
            else:                    #else go right
                self.go(RIGHT)
            rospy.sleep(.01)

    def should_leave_wall(self): #fn to leave the wall
        (x, y, _) = current_location.current_location()  #copying the current location
        if None in self.encountered_wall_at:  #checking if the robot has encountered the wall previously
            self.encountered_wall_at = (x, y) #saving the current encountred wall location
            self.lh = necessary_heading(x, y, self.tx1, self.ty1) #calculating the angle towards destination
            return False
        t_angle = necessary_heading(x, y, self.tx1, self.ty1) #calculating the angle towards the destination
        (ox, oy) = self.encountered_wall_at #copying the ecountered wall location data
        od = math.sqrt((ox-self.tx1)**2 + (oy-self.ty1)**2) #calculating distance b/w the goal and wall encounter position
        cd = math.sqrt( (x-self.tx1)**2 +  (y-self.ty1)**2) #calculating distance b/w the goal and current position
        dt = 0.01 #angle tolerance in the direction towards goal

        if self.lh - dt <= t_angle <= self.lh + dt and not near(x, y, ox, oy): #condition to leave the wall
            if cd < od:
                print "Leaving wall"
                return True
        return False

    def face_goal(self): #fn to make the robot face towards the goal
        while not current_location.facing_point(tx1, ty1):
                self.go(RIGHT)
                rospy.sleep(.01)

class Goal_two: #same as Goal_one class
    global tx2, ty2
    def __init__(self):

        self.pub = rospy.Publisher('cmd_vel', Twist, queue_size=1)
        self.tx2 = tx2
        self.ty2 = ty2
        self.lh = None
        self.encountered_wall_at = (None, None)

    def go(self, direction):
        cmd = Twist()
        if direction == STRAIGHT:
            cmd.linear.x = 0.3
        elif direction == LEFT:
            cmd.angular.z = 0.25
        elif direction == RIGHT:
            cmd.angular.z = -0.25
        elif direction == MSG_STOP:
            cmd.linear.x = 0
            cmd.angular.z = 0
        self.pub.publish(cmd)

    def go_until_obstacle(self):

        print "Going until destination or obstacle"
        while current_location.distance(tx2, ty2) > delta:
            (frontdist, _, _) = current_dists.get()
            if frontdist <= WALL_PADDING:
                return True

            if current_location.facing_point(tx2, ty2):
                self.go(STRAIGHT)
            elif current_location.faster_left(tx2, ty2):
                self.go(LEFT)
            else:
                self.go(RIGHT)
            rospy.sleep(.01)
        return False

    def follow_wall(self):
        print "Following wall"
        while current_dists.get()[0] <= WALL_PADDING:
            self.go(RIGHT)
            rospy.sleep(.01)
        while not self.should_leave_wall():
            (front, left, right) = current_dists.get()
            if front <= WALL_PADDING:
                self.go(RIGHT)
            elif WALL_PADDING - .1 <= left <= WALL_PADDING + .1:
                self.go(STRAIGHT)
            elif left > WALL_PADDING + .1:
                self.go(LEFT)
            elif left < 1 and right < 1:
                self.go(STRAIGHT)
            else:
                self.go(RIGHT)
            rospy.sleep(.01)

    def should_leave_wall(self):
        (x, y, _) = current_location.current_location()
        if None in self.encountered_wall_at:
            self.encountered_wall_at = (x, y)
            self.lh = necessary_heading(x, y, self.tx2, self.ty2)
            return False
        t_angle = necessary_heading(x, y, self.tx2, self.ty2)
        (ox, oy) = self.encountered_wall_at
        od = math.sqrt((ox-self.tx2)**2 + (oy-self.ty2)**2)
        cd = math.sqrt( (x-self.tx2)**2 +  (y-self.ty2)**2)
        dt = 0.01

        if self.lh - dt <= t_angle <= self.lh + dt and not near(x, y, ox, oy):
            if cd < od:
                print "Leaving wall"
                return True
        return False

    def face_goal(self):
        while not current_location.facing_point(tx2, ty2):
                self.go(RIGHT)
                rospy.sleep(.01)


class Goal_three:   #same as class goal_one
    global tx3, ty3

    def __init__(self):
        global tx3,ty3
        self.pub = rospy.Publisher('cmd_vel', Twist, queue_size=1)
        self.tx3 = tx3
        self.ty3 = ty3
        self.lh = None
        self.encountered_wall_at = (None, None)

    def go(self, direction):
        cmd = Twist()
        if direction == STRAIGHT:
            cmd.linear.x = 0.5
        elif direction == LEFT:
            cmd.angular.z = 0.1
        elif direction == RIGHT:
            cmd.angular.z = -0.1
        elif direction == MSG_STOP:
            cmd.linear.x = 0
            cmd.angular.z = 0
        self.pub.publish(cmd)

    def go_until_obstacle(self):

        print "Going until destination or obstacle"
        while current_location.distance(tx3, ty3) > delta:
            (frontdist, _, _) = current_dists.get()
            if frontdist <= WALL_PADDING:
                return True

            if current_location.facing_point(tx3, ty3):
                self.go(STRAIGHT)
            elif current_location.faster_left(tx3, ty3):
                self.go(LEFT)
            else:
                self.go(RIGHT)
            rospy.sleep(.01)
        return False

    def follow_wall(self):
        print "Following wall"
        while current_dists.get()[0] <= WALL_PADDING:
            self.go(RIGHT)
            rospy.sleep(.01)
        while not self.should_leave_wall():
            (front, left, right) = current_dists.get()
            if front <= WALL_PADDING:
                self.go(RIGHT)
            elif WALL_PADDING - .1 <= left <= WALL_PADDING + .1:
                self.go(STRAIGHT)
            elif left > WALL_PADDING + .1:
                self.go(LEFT)
            elif left < 1 and right < 1:
                self.go(STRAIGHT)
            else:
                self.go(RIGHT)
            rospy.sleep(.01)

    def should_leave_wall(self):
        (x, y, _) = current_location.current_location()
        if None in self.encountered_wall_at:
            self.encountered_wall_at = (x, y)
            self.lh = necessary_heading(x, y, self.tx3, self.ty3)
            return False
        t_angle = necessary_heading(x, y, self.tx3, self.ty3)
        (ox, oy) = self.encountered_wall_at
        od = math.sqrt((ox-self.tx3)**2 + (oy-self.ty3)**2)
        cd = math.sqrt( (x-self.tx3)**2 +  (y-self.ty3)**2)
        dt = 0.01

        if self.lh - dt <= t_angle <= self.lh + dt and not near(x, y, ox, oy):
            if cd < od:
                print "Leaving wall"
                return True
        return False

    def face_goal(self):
        while not current_location.facing_point(tx3, ty3):
                self.go(RIGHT)
                rospy.sleep(.01)




def near(cx, cy, x, y):  #fn to check weather the two points are near or not
    nearx = x - .3 <= cx <= x + .3
    neary = y - .3 <= cy <= y + .3
    return nearx and neary

#Main Function
if __name__ == '__main__':
    print "Initialising.."
    init_listener()
    print "Calibrating sensors..."
    rospy.sleep(5)
    print "Calibrated"
    print "Target", (tx1,ty1)
    goal_one = Goal_one()
    while current_location.distance(tx1, ty1) > delta:  #loop running till we reach the goal
        hit_wall = goal_one.go_until_obstacle()  #going till obstacle or wall in the direction of goal
        if hit_wall:
            goal_one.follow_wall()    #following the obstacle that was encountered
        goal_one.face_goal() #facing the wall
    goal_one.go(MSG_STOP)  #stopping the robot as we reach the goal
    print "Arrived at", (tx1, ty1)
    rospy.sleep(5)
    print "Target", (tx2,ty2)
    goal_two = Goal_two()
    while current_location.distance(tx2, ty2) > delta:
        hit_wall = goal_two.go_until_obstacle()
        if hit_wall:
            goal_two.follow_wall()
        goal_two.face_goal()
    goal_two.go(MSG_STOP)
    print "Arrived at", (tx2, ty2)
    rospy.sleep(5)
    print "Target", (tx3,ty3)
    goal_three = Goal_three()
    while current_location.distance(tx3, ty3) > delta:
        hit_wall = goal_three.go_until_obstacle()
        if hit_wall:
            goal_three.follow_wall()
        goal_three.face_goal()
    goal_three.go(MSG_STOP)
    print "Arrived at", (tx3, ty3)
    rospy.sleep(5)
    rospy.is_shutdown()
