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

#grab the left arm
Llimb = baxter_interface.Limb('left')
Langles = Llimb.joint_angles()
Lvel = Llimb.joint_velocities()

#grab the right arm
Rlimb = baxter_interface.Limb('right')
Rangles = Rlimb.joint_angles()

def rotateLeftGripper(self, rotationAng = 0.2):

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
        rotateLeftGripper(v)
    else:
        rotateLeftGripper(v, -0.2)
    print (v)
    
def scrollWheel(self):
    print "hey, someone clicked on the scroll Wheel" 
    rotateLeftGripper(self)

def setButtonHandlers(arm):
# inputs
    _close_io = baxter_interface.DigitalIO('%s_upper_button' % (arm,)) # 'dash' btn
    _close_io.state_changed.connect(rotateLeftGripper)

    _open_io = baxter_interface.DigitalIO('%s_lower_button' % (arm,)) # 'circle' btn
    _open_io.state_changed.connect(rotateLeftGripper)
    
    Lnav = (baxter_interface.Navigator('left'))
    Lnav.button0_changed.connect(scrollWheel)
    Lnav.wheel_changed.connect(wheel_moved)

def main():
    
    #print("listening on the left arm")
    setButtonHandlers('left')
    print ""
    print Lvel
   

    lights = baxter_interface.DigitalIO('left_itb_light_outer')
    print "Initial state: ", lights.state

    #print("Press cuff buttons to control grippers. Spinning")
    rospy.spin()
    return 0
    




#start with the waypoint following code


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
    left_command = dict((key, command[key]) for key in command.keys()
                        if key[:-2] == 'left_')
    right_command = dict((key, command[key]) for key in command.keys()
                         if key[:-2] == 'right_')
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
    left = baxter_interface.Limb('left')
    right = baxter_interface.Limb('right')
    grip_left = baxter_interface.Gripper('left', CHECK_VERSION)
    grip_right = baxter_interface.Gripper('right', CHECK_VERSION)
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

        _cmd, lcmd_start, rcmd_start, _raw = clean_line(lines[1], keys)
        Limb.move_to_joint_positions(lcmd_start)
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
                if ('left_gripper' in cmd and
                    grip_left.type() != 'custom'):
                    grip_left.command_position(cmd['left_gripper'])
                if ('right_gripper' in cmd and
                    grip_right.type() != 'custom'):
                    grip_right.command_position(cmd['right_gripper'])
                rate.sleep()
        print
    return True

















if __name__ == '__main__':
    sys.exit(main())




