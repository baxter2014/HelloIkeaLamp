import argparse
import sys
import math
import rospy

from baxter_interface import (
    DigitalIO,
    Gripper,
    Navigator,
    CHECK_VERSION,
)

#this should import the grippers left and right
right_gripper = Gripper('right')
left_gripper = Gripper('left')



Llimb = baxter_interface.Limb('left')
Langles = Llimb.joint_angles()

Rlimb = baxter_interface.Limb('right')
Rangles = Rlimb.joint_angles()


def rotateLeft(rotationAng = 0.2):
	Langles['left_w2'] = Langles['left_w2'] + rotationAng
	limb.move_to_joint_positions(Langles)

def rotateRight(rotationAng = 0.2):
	Langles['left_w2'] = Langles['left_w2'] - rotationAng
	limb.move_to_joint_positions(Langles)



def  initializeArms():
	#set left limb in default position
	Langles = {'right_s0': 0.0, 'right_s1': 0.0, 'right_e0': 0.0, 'right_e1': 0.0, 'right_w0': 0.0, 'right_w1': 0.0, 'right_w2': 0.0}
	Llimb.move_to_joint_positions(Langles)
	#set right limb in default position
	Rangles = {'right_s0': 0.0, 'right_s1': 0.0, 'right_e0': 0.0, 'right_e1': 0.0, 'right_w0': 0.0, 'right_w1': 0.0, 'right_w2': 0.0}
	Rlimb.move_to_joint_positions(Rangles)

def onLeftArmXButtonPressed(self, value):
	print("button pressed" + value)

def setButtonHandlers(arm):

        # inputs
        _close_io = DigitalIO('%s_upper_button' % (arm,))  # 'dash' btn

        _close_io.state_changed.connect(onLeftArmXButtonPressed)

