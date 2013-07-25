import rospy
import smach
from actuator_driver.srv import PulseValve, SetValve
from actuator_driver.msg import Switches

VALVE_DROPPER = 0
VALVE_GRABBER_CLOSE = 1
VALVE_GRABBER_OPEN = 2
VALVE_SHOOTER_RIGHT = 3
VALVE_GAS_POWERED_STICK = 4
VALVE_SHOOTER_LEFT = 5

class CloseGrabberState(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded', 'empty'])
        self._setvalve = rospy.ServiceProxy('actuator_driver/set_valve', SetValve)
        
    def execute(self, userdata):
        self._setvalve(VALVE_GRABBER_OPEN, False)
        rospy.sleep(.1)
        self._setvalve(VALVE_GRABBER_CLOSE, True)
        rospy.sleep(1)
        pressed = rospy.wait_for_message('actuator_driver/switches', Switches).pressed
        if not all(pressed):
            return 'succeeded'
        else:
            return 'empty'

class OpenGrabberState(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded'])
        self._setvalve = rospy.ServiceProxy('actuator_driver/set_valve', SetValve)
        
    def execute(self, userdata):
        self._setvalve(VALVE_GRABBER_CLOSE, False)
        rospy.sleep(.1)
        self._setvalve(VALVE_GRABBER_OPEN, True)
        rospy.sleep(1)
        return 'succeeded'

class DropBallState(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded'])
        self._pulsevalve = rospy.ServiceProxy('actuator_driver/pulse_valve', PulseValve)
        
    def execute(self, userdata):
        self._pulsevalve(VALVE_DROPPER, rospy.Duration(1))
        return 'succeeded'

class ShootTorpedoState(smach.State):
    def __init__(self, shooter_side):
        smach.State.__init__(self, outcomes=['succeeded'])
        assert(shooter_side in ('left', 'right'))
        self._shooter_side = shooter_side
        self._pulsevalve = rospy.ServiceProxy('actuator_driver/pulse_valve', PulseValve)
        
    def execute(self, userdata):
        valve = VALVE_SHOOTER_LEFT if self._shooter_side == 'left' else VALVE_SHOOTER_RIGHT
        self._pulsevalve(valve, rospy.Duration(.3))
        return 'succeeded'

class GasPoweredStickState(smach.State):
    def __init__(self, extended):
        smach.State.__init__(self, outcomes=['succeeded'])
        self._extended = extended
        self._setvalve = rospy.ServiceProxy('actuator_driver/set_valve', SetValve)
        
    def execute(self, userdata):
        self._setvalve(VALVE_GAS_POWERED_STICK, self._extended)
        rospy.sleep(1)
        return 'succeeded'
