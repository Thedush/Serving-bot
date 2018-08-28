#!/usr/bin/env python
"""
   twist_to_motors - converts a twist message to motor commands.  Needed for navigation stack
   
   
    Copyright (C) 2012 Jon Stephan. 
     
    This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <http://www.gnu.org/licenses/>.
"""

import rospy
import roslib
from std_msgs.msg import Float32
from geometry_msgs.msg import Twist 

#############################################################
#############################################################
class TwistToMotors():
#############################################################
#############################################################

    #############################################################
    def __init__(self):
    #############################################################
        rospy.init_node("twist_to_motors")
        nodename = rospy.get_name()
        #rospy.loginfo("%s started" % nodename)
    
        self.wheel_radius = rospy.get_param("~wheel_radius", 0.05)
        self.lx = rospy.get_param("~length_x", 0.24)
        self.ly= rospy.get_param("~length_y", 0.22)
        self.pub_l_f_motor = rospy.Publisher('l_f_wheel_vtarget', Float32,queue_size=10)
        self.pub_r_f_motor = rospy.Publisher('r_f_wheel_vtarget', Float32,queue_size=10)
        self.pub_l_b_motor = rospy.Publisher('l_b_wheel_vtarget', Float32,queue_size=10)
        self.pub_r_b_motor = rospy.Publisher('r_b_wheel_vtarget', Float32,queue_size=10)
        rospy.Subscriber('cmd_vel', Twist, self.twistCallback)
    
    
        self.rate = rospy.get_param("~rate", 50)
        self.timeout_ticks = rospy.get_param("~timeout_ticks", 2)
        self.left = 0
        self.right = 0
        
    #############################################################
    def spin(self):
    #############################################################
    
        r = rospy.Rate(self.rate)
        idle = rospy.Rate(10)
        then = rospy.Time.now()
        self.ticks_since_target = self.timeout_ticks
    
        ###### main loop  ######
        while not rospy.is_shutdown():
        
            while not rospy.is_shutdown() and self.ticks_since_target < self.timeout_ticks:
                self.spinOnce()
                r.sleep()
            idle.sleep()
                
    #############################################################
    def spinOnce(self):
    #############################################################
    
        # dx = (l + r) / 2
        # dr = (r - l) / w
        self.left_for = (self.dx + self.dy -((self.lx+self.ly) * self.dr) )
        self.right_for = (self.dx - self.dy +((self.lx+self.ly) *self.dr) )
        self.left_back = (self.dx - self.dy -((self.lx+self.ly) * self.dr) )
        self.right_back = (self.dx + self.dy +((self.lx+self.ly) *self.dr) )
        # rospy.loginfo("publishing: (%d, %d)", left, right) 
        #rospy.loginfo("%0.3f ,%0.3f,%0.3f,%0.3f " ,self.left_for*self.wheel_radius,self.right_for*self.wheel_radius,self.left_back*self.wheel_radius,self.right_back*self.wheel_radius)        
        self.pub_l_f_motor.publish(self.left_for)
        self.pub_r_f_motor.publish(self.right_for)
        self.pub_l_b_motor.publish(self.left_back)
        self.pub_r_b_motor.publish(self.right_back)
        self.ticks_since_target += 1

    #############################################################
    def twistCallback(self,msg):
    #############################################################
        # rospy.loginfo("-D- twistCallback: %s" % str(msg))
        self.ticks_since_target = 0
        self.dx = msg.linear.x
        self.dr = msg.angular.z
        self.dy = msg.linear.y
    
#############################################################
#############################################################
if __name__ == '__main__':
    """ main """
    twistToMotors = TwistToMotors()
    twistToMotors.spin()
