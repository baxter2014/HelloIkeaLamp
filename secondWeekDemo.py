#!/usr/bin/python2
""" this is the first week demo
    it lets you use baxter's buttons to controll the angle of an ikea lamp that is held in the left gripper
    it also lets you grab components (currently it just plays back a pre recorded limb action,
    letting the user define and modify the action is for next week)
"""
import argparse
import sys
import math
import rospy
import  baxter_interface


rospy.init_node('hellowo', anonymous=True)

#this should import the grippers left and right
right_gripper = baxter_interface.Gripper('right')
left_gripper = baxter_interface.Gripper('left')
#force calibration of left gripper 
left_gripper.calibrate()
#initialize the left gripper as closed
left_gripper.close()
Lclosed = True

#grab the left arm
Llimb = baxter_interface.Limb('left')
#set left limb to 'comfortable' position to get started with
Langles = {'left_w0': 2.0793109556030274, 'left_w1': -1.4522963092712404, 'left_w2': -2.9, 'left_e0': -0.17832526638793947, 'left_e1': 2.1414371774414063, 'left_s0': 0.22472818516845705, 'left_s1': -0.683388440222168}
Llimb.move_to_joint_positions(Langles)

#grab the right arm
Rlimb = baxter_interface.Limb('right')
Rangles = {'right_s0': -0.12923788123168947, 'right_s1': -0.34361169609375003, 'right_w0': -1.1830826813049318, 'right_w1': 1.1347622865417482, 'right_w2': 2.5966459757263185, 'right_e0': 1.1581554935302736, 'right_e1': 1.3974564961669922}
Rlimb.move_to_joint_positions(Rangles)
Rnav = (baxter_interface.Navigator('right'))


class Waypoints(object):
    def __init__(self, limb, speed, accuracy):
        # Create baxter_interface limb instance

        self._arm = limb
        self._limb = baxter_interface.Limb(self._arm)

        # Parameters which will describe joint position moves
        self._speed = speed
        self._accuracy = accuracy

        # Recorded waypoints
        self._waypoints = list()

        # Recording state
        self._is_recording = False

        # Verify robot is enabled
        print("Getting robot state... ")
        self._rs = baxter_interface.RobotEnable()
        self._init_state = self._rs.state().enabled
        print("Enabling robot... ")
        self._rs.enable()
        # Create Navigator I/O

    def _record_waypoint(self, value):
        """
        Stores joint position waypoints

        Navigator 'OK/Wheel' button callback
        """
        if value:
            print("Waypoint Recorded")
            self._waypoints.append(self._limb.joint_angles())

    def _stop_recording(self, value):
        """
        Sets is_recording to false

        Navigator 'Rethink' button callback
        """
        # On navigator Rethink button press, stop recording
        print 'recording should stop'
        if value:
            self._is_recording = False

    def record(self):
        """
        Records joint position waypoints upon each Navigator 'OK/Wheel' button
        press.
        """
        rospy.loginfo("Waypoint Recording Started")
        print("Press Navigator 'OK/Wheel' button to record a new joint "
        "joint position waypoint.")
        print("Press Navigator 'Rethink' button when finished recording "
              "waypoints to begin playback")
        # Connect Navigator I/O signals
        # Navigator scroll wheel button press
        # Rnav.button0_changed.connect(self._record_waypoint)
        # Navigator Rethink button press
        Rnav.button2_changed.connect(self._stop_recording)

        # Set recording flag
        self._is_recording = True

        # Loop until waypoints are done being recorded ('Rethink' Button Press)
        while self._is_recording and not rospy.is_shutdown():
            rospy.sleep(0.1)

        # We are now done with the navigator I/O signals, disconnecting them
        Rnav.button0_changed.disconnect(self._record_waypoint)
        Rnav.button2_changed.disconnect(self._stop_recording)

    def playback(self):
        """
        Loops playback of recorded joint position waypoints until program is
        exited
        """
        rospy.sleep(1.0)

        rospy.loginfo("Waypoint Playback Started")
        print("  Press Ctrl-C to stop...")

        # Set joint position speed ratio for execution
        self._limb.set_joint_position_speed(self._speed)

        # Loop until program is exited
        loop = 0
        while not rospy.is_shutdown():
            loop += 1
            print("Waypoint playback loop #%d " % (loop,))
            for waypoint in self._waypoints:
                if rospy.is_shutdown():
                    break
                self._limb.move_to_joint_positions(waypoint, timeout=20.0,
                                                   threshold=self._accuracy)
            # Sleep for a few seconds between playback loops
            rospy.sleep(3.0)

        # Set joint position speed back to default
        self._limb.set_joint_position_speed(0.3)

    def clean_shutdown(self):
        print("\nExiting example...")
        if not self._init_state:
            print("Disabling robot...")
            self._rs.disable()
        return True


recorder = Waypoints('right', 0.5, 0.1)
recording = False




def rotateLeftGripper(rotationAng = -0.2):

    print("rotating the cuff")
    Langles = Llimb.joint_angles()

    if Langles['left_w2'] >= 3:
        Langles['left_w2'] = -2.9
        Llimb.move_to_joint_positions(Langles)
    elif Langles['left_w2'] <= -3:
        Langles['left_w2'] = 2.9
        Llimb.move_to_joint_positions(Langles)
    else:
        Langles['left_w2'] = Langles['left_w2'] + rotationAng
        Llimb.move_to_joint_positions(Langles)

def wheel_moved(v):
    if v == -1:
        rotateLeftGripper(0.2)
    else:
        rotateLeftGripper()
    print (v)
    
def scrollWheel(value):
    if value:
        print "hey, someone clicked on the scroll Wheel" 
        rotateLeftGripper()

def pickFlower(value):
    if value:
        print "picking a flower"
        recorder.playback()

def toggleLeftGripper(value):
    global Lclosed
    if value and Lclosed:
        print 'I got a gripper change'
        left_gripper.open()
        print 'should be open'
        Lclosed = False
    elif value:
        print 'I got a gripper change'
        left_gripper.close()
        print 'should be closed'
        Lclosed = True

def recordNewFlowerPick(value):

    Rnav.button0_changed.disconnect(pickFlower)
    Rnav.button2_changed.disconnect(recordNewFlowerPick)

    Rnav.button2_changed.connect(fuckity)
    recorder.record()

    # Rnav.button0_changed.connect(pickFlower)
    # Rnav.button2_changed.connect(recordNewFlowerPick)

def fuckity(value):
    if value:
        print 'fuck'
        recorder._stop_recording
def setButtonHandlers(arm):
# inputs
    if arm == 'left':

        _close_io = baxter_interface.DigitalIO('%s_upper_button' % (arm,)) # 'dash' btn
        _close_io.state_changed.connect(toggleLeftGripper)

        _open_io = baxter_interface.DigitalIO('%s_lower_button' % (arm,)) # 'circle' btn
        _open_io.state_changed.connect(rotateLeftGripper)
        
        Lnav = (baxter_interface.Navigator('left'))
        Lnav.button0_changed.connect(scrollWheel)
        Lnav.wheel_changed.connect(wheel_moved)

    elif arm == 'right':

        Rnav.button0_changed.connect(pickFlower)
        Rnav.button2_changed.connect(recordNewFlowerPick)

    else:
        print 'uhm something went wrong here'

def main():
    print 'setting the left arm to move at 0.5'
    Llimb.set_joint_position_speed(0.5)

    print 'setting the right arm to move at 0.8'
    Rlimb.set_joint_position_speed(0.8)

    setButtonHandlers('left')
    setButtonHandlers('right')
   
    rospy.spin()
    return 0
    




#start with the waypoint following code as pulled from the Rethink Robotics Github
# Copyright (c) 2013-2014, Rethink Robotics
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
# 1. Redistributions of source code must retain the above copyright notice,
#    this list of conditions and the following disclaimer.
# 2. Redistributions in binary form must reproduce the above copyright
#    notice, this list of conditions and the following disclaimer in the
#    documentation and/or other materials provided with the distribution.
# 3. Neither the name of the Rethink Robotics nor the names of its
#    contributors may be used to endorse or promote products derived from
#    this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

def try_float(x):
    try:
        return float(x)
    except ValueError:
        return None


def clean_line(line, names):
    """
    Cleans a single line of recorded joint positions

    @param line: the line described in a list to process
    @param names: joint name keys
    """
    #convert the line of strings to a float or None
    line = [try_float(x) for x in line.rstrip().split(',')]
    #zip the values with the joint names
    combined = zip(names[1:], line[1:])
    #take out any tuples that have a none value
    cleaned = [x for x in combined if x[1] is not None]
    #convert it to a dictionary with only valid commands
    command = dict(cleaned)
    right_command = dict((key, command[key]) for key in command.keys()
                         if key[:-2] == 'right_')
    left_command = dict((key, command[key]) for key in command.keys()
                         if key[:-2] == 'left_')
    return (command, left_command, right_command, line)


def map_file(filename, loops=1):
    """
    Loops through csv file

    @param filename: the file to play
    @param loops: number of times to loop
                  values < 0 mean 'infinite'

    Does not loop indefinitely, but only until the file is read
    and processed. Reads each line, split up in columns and
    formats each line into a controller command in the form of
    name/value pairs. Names come from the column headers
    first column is the time stamp
    """

    grip_right = baxter_interface.Gripper('right')


    rate = rospy.Rate(1000)

    print("Playing back: %s" % (filename,))
    with open(filename, 'r') as f:
        lines = f.readlines()
    keys = lines[0].rstrip().split(',')

    l = 0
    # If specified, repeat the file playback 'loops' number of times
    while loops < 1 or l < loops:
        i = 0
        l += 1
        print("Moving to start position...")
        print clean_line(lines[1], keys)
        _cmd, lcmd_start, rcmd_start, _raw = clean_line(lines[1], keys)
        #Llimb.move_to_joint_positions(lcmd_start)
        Rlimb.move_to_joint_positions(rcmd_start)
        start_time = rospy.get_time()
        for values in lines[1:]:
            i += 1
            loopstr = str(loops) if loops > 0 else "forever"
            sys.stdout.write("\r Record %d of %d, loop %d of %s" %
                             (i, len(lines) - 1, l, loopstr))
            sys.stdout.flush()

            cmd, lcmd, rcmd, values = clean_line(values, keys)
            #command this set of commands until the next frame
            while (rospy.get_time() - start_time) < values[0]:
                if rospy.is_shutdown():
                    print("\n Aborting - ROS shutdown")
                    return False
               # if len(lcmd):
               #    Llimb.set_joint_positions(lcmd)
                if len(rcmd):
                    Rlimb.set_joint_positions(rcmd)
                if ('right_gripper' in cmd and
                    grip_right.type() != 'custom'):
                    grip_right.command_position(cmd['right_gripper'])
                rate.sleep()
        print
    return True


if __name__ == '__main__':
    sys.exit(main())



