#!/usr/bin/env python
''' test_square.py
'''
import argparse

import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Pose, Twist
from std_srvs.srv import Empty
import numpy as np


HORIZONTAL_LINE_LENGTH = 50


class square_test:
    ''' Quantifies performance of turtlebot driving in a square at constant speed
    '''

    def __init__(self, waypoint_tolerance=0.1, run_count=2):
        rospy.init_node('test_square')  
        rospy.sleep(1.0)  # Wait for node to initialize     

        # Basic testing detection variables
        self.waypoint_tolerance = waypoint_tolerance  # Waypoint tolerance distance for passing through points
        self.max_time = 20.                           # maximum time for test in seconds
        self.delta_speed_threshold = 0.1              # maximum permissible change in speed after reaching min_dist from start
        self.nedge = 6                                # Testing points per edge, including one corner point for each edge. Total is nedge * 4 testing points including four corners.
        self.edge_length = 1.                         # Length of the test cube edge.
        self.min_dist = self.edge_length / 4.         # Starts various tests after reaching this distance from start
        self.square = self.square_points()            # Waypoints
        self.run_count = run_count                    # Number of runs to track and average across

        # Settings to enable auto-restarting of test
        self.run_scores = []                          # Tracks {run_count} subsequent run scores in order to compute an average
        self.is_init_run = False
        self.is_done = True
        self.is_started = False
        self.completed_time = rospy.Time.now()
        self.reset_time = rospy.Time.now()

        # To command robot to stop before resetting
        self.pub = rospy.Publisher('cmd_vel',Twist, queue_size=1)

        # Testing is done using /odom
        rospy.loginfo('Subscribing to /odom')
        rospy.Subscriber("/odom", Odometry, self.callback_odom)


    @property
    def current_run(self):
        return len(self.run_scores) + 1


    def get_run_display(self):
        return f'{self.current_run} of {self.run_count}'


    def square_points(self):
        ''' Create waypoints in a square '''
        square = np.concatenate( (
                np.stack( (np.arange(1,self.nedge+1)/self.nedge, np.zeros((self.nedge,))) ), 
                np.stack( (np.ones((self.nedge,)), np.arange(1,self.nedge+1)/self.nedge) ),
                np.stack( (np.arange(self.nedge,0,-1)/self.nedge, np.ones((self.nedge,))) ),
                np.stack( (np.zeros((self.nedge,)), np.arange(self.nedge,0,-1)/self.nedge) ),
                 ), axis=1)
        return square
        

    def reset(self):
        ''' Reset world '''
        self.pub.publish(Twist())  # Command robot to stop
        rospy.sleep(1.)            # Wait 1 second
        rospy.wait_for_service('/gazebo/reset_world')
        reset_world = rospy.ServiceProxy('/gazebo/reset_simulation', Empty)
        reset_world()
        rospy.loginfo('Reset world')
        self.reset_time = rospy.Time.now()
        rospy.loginfo(f'Reset time: {self.reset_time.to_sec():.3f}')


    def init_run(self):
        ''' Initialize variables for a new run '''
        self.completed = np.zeros( (self.nedge*4,), dtype=np.bool)
        self.is_started = False
        self.min_dist_done = False 
        self.sumspeed = 0.
        self.minspeed = 100.  # Any large value
        self.maxspeed = 0.
        self.nsum = 0
        self.is_done = False
        self.start_time = 0
        self.elapsed_time = 0
        self.is_init_run = True

        rospy.loginfo( HORIZONTAL_LINE_LENGTH*'=' )
        rospy.loginfo(f'Initialized and ready for test {self.get_run_display()} with max duration: {self.max_time:.1f}')


    def check_done(self):
        ''' Test if time if out or if we have reached the last waypoint '''
        if self.is_started:
            etime = (rospy.Time.now()-self.start_time).to_sec()
            if etime >= self.max_time or self.completed[-1]:
                self.is_done = True
                self.elapsed_time = etime

    def callback_odom(self, msg):
        ''' Determine if robot has started and log which waypoints it reaches before time expires '''

        if self.is_done:
            # If we're done, wait for 2 seconds and then reset
            if (rospy.Time.now()-self.completed_time).to_sec() > 2.:                
                self.reset()
                self.is_done = False
            else:
                return

        if not self.is_init_run:
            # Only initialize after waiting 1 second from reset, since it takes a while for speed -> 0
            if (rospy.Time.now()-self.reset_time).to_sec() > 1.:                
                self.init_run()
            return

        # Linear speed of Turtlebot
        speed = np.sqrt( msg.twist.twist.linear.x**2 + msg.twist.twist.linear.y**2 )

        if not self.is_started:
            # Start run when speed passes a small threshold:
            if speed > 0.01:
                rospy.loginfo(f'Starting run {self.get_run_display()}, speed: {speed:.3f}')
                self.is_started = True
                self.start_time = rospy.Time.now()
            else:
                return

        # Position of Turtlebot
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y

        if x**2 + y**2 > self.min_dist**2: 
            self.min_dist_done = True

        if self.min_dist_done:
            # Speed stats:
            self.sumspeed += speed
            if speed > self.maxspeed:
                self.maxspeed = speed
            if speed < self.minspeed:
                self.minspeed = speed
            self.nsum += 1

        # Mark 
        N = self.nedge*4 if self.min_dist_done else self.nedge*2
        dist2 = (x-self.square[0,:N])**2 + (y-self.square[1,:N])**2
        close = dist2 < self.waypoint_tolerance**2
        N_completed_waypoints = self.completed.sum()
        self.completed[:N] += close
        N_new_completed_waypoints = self.completed.sum() - N_completed_waypoints
        if N_new_completed_waypoints:
            print(N_new_completed_waypoints*'.',end='',flush=True)

        self.check_done()
        if self.is_done:
            speed_mean = self.sumspeed /max(self.nsum,1)
            print(f' {self.completed.sum()} waypoints reached: {self.completed.astype(int)}')
            rospy.loginfo( f'Number waypoints completed:     {self.completed.sum()} / {self.nedge*4}')
            rospy.loginfo( f'Reached end-point:              {self.completed[-1]}')
            rospy.loginfo( f'Time on course (sec):           {self.elapsed_time:0.2f} sec' )
            rospy.loginfo( f'Speed: min, mean and max (m/s): {self.minspeed:.2f}, {speed_mean:.2f}, {self.maxspeed:.2f}')
            delta_speed = self.maxspeed - self.minspeed

            if delta_speed > 0.1:
                rospy.loginfo( f'Speed max - min:                {delta_speed:.2f} is over threshold of 0.1')
            else:
                rospy.loginfo( f'Speed max - min:                {delta_speed:.2f} is good')

            # Now calculate score:
            score = 4 if self.min_dist_done else 0
            completion_score = min( self.completed.sum() / 4, 5 )
            score += completion_score
            rospy.loginfo( f'Score for completing waypoints: {score:.2f}')
            if self.elapsed_time <= 15:
                score += 0.5
                rospy.loginfo( f'Under 15 seconds: +0.5')
            if self.elapsed_time <= 10:
                score += 0.5
                rospy.loginfo( f'Under 10 seconds: +0.5')
            if self.maxspeed - self.minspeed > 0.1:
                score -= 2
                rospy.loginfo( f'Not constant speed: -2.0')
            rospy.loginfo(HORIZONTAL_LINE_LENGTH*'-')
            rospy.loginfo(f'Estimated score: {score} / 10')

            # No need to keep appending/cleaning/printing average if run count is 1
            if self.run_count > 1:
                self.run_scores.append(score)
                if len(self.run_scores) == self.run_count:
                    rospy.loginfo(HORIZONTAL_LINE_LENGTH * '-')
                    rospy.loginfo(f'All runs complete.')
                    rospy.loginfo(f'Final score: {round(sum(self.run_scores) / len(self.run_scores), 2)} / 10')
                    rospy.loginfo(HORIZONTAL_LINE_LENGTH * '-')
                    self.run_scores.clear()

            self.completed_time = rospy.Time.now()
            self.is_init_run = False

    
if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='Test Square Arguments')
    parser.add_argument('--count', default=2, type=int, help='Number of test runs to average')
    args, _ = parser.parse_known_args()
    square_test(0.1, args.count)
    rospy.spin() # Keeps python from exiting and handles callbacks
