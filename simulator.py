import time

import pybullet as p
import pybullet_data
import upg_planning as pl

class Megabot:
    """
        Links : self.legs[leg_id][i] = id du i-ème membre de la patte leg_id
        Joints : self.joint_name_to_id['ij'] = id du j-ème joint de la patte i
    """

    def __init__(
            self,
    ):
        self.id = p.loadURDF("megabot/robot.urdf", [0, 0, 0.6], p.getQuaternionFromEuler([0, 0, 0]))
        self.connectors = [[16, 8, 6, 9, 12, 17, 15, 72],
                           [34, 26, 24, 27, 30, 35, 33, 73],
                           [52, 44, 42, 45, 48, 53, 51, 74],
                           [70, 62, 60, 63, 66, 71, 69, 75]]
        self._BuildJointNameToIdDict()
        self._CreateClosedKinematicLoops()

    def _BuildJointNameToIdDict(self):
        num_joints = p.getNumJoints(self.id)
        self.joint_name_to_id = {}
        for i in range(num_joints):
            joint_info = p.getJointInfo(self.id, i)
            self.joint_name_to_id[joint_info[1].decode("UTF-8")] = joint_info[0]

    def _CreateClosedKinematicLoops(self):
        for i in range(4):
            self._Constraint(i, 0, [0, 1, 0])
            self._Constraint(i, 1, [1, 0, 0])
            self._Constraint(i, 2, [1, 0, 0])
            self._Constraint(i, 3, [1, 0, 0])

    def _Constraint(self, leg_id, n, axis):
        p.createConstraint(self.id, self.connectors[leg_id][2 * n], self.id, self.connectors[leg_id][2 * n + 1],
                           p.JOINT_POINT2POINT, axis, [0, 0, 0], [0, 0, 0])

    def MoveCylinders(self, V, force):
        p.setJointMotorControl2(self.id, self.joint_name_to_id['0v1'], targetPosition=0.4455 - V[0] * 0.001,
                                controlMode=p.POSITION_CONTROL, force=force)
        p.setJointMotorControl2(self.id, self.joint_name_to_id['0v2'], targetPosition=0.4455 - V[1] * 0.001,
                                controlMode=p.POSITION_CONTROL, force=force)
        p.setJointMotorControl2(self.id, self.joint_name_to_id['0v3'], targetPosition=0.4455 - V[2] * 0.001,
                                controlMode=p.POSITION_CONTROL, force=force)
        p.setJointMotorControl2(self.id, self.joint_name_to_id['1v1'], targetPosition=0.4455 - V[3] * 0.001,
                                controlMode=p.POSITION_CONTROL, force=force)
        p.setJointMotorControl2(self.id, self.joint_name_to_id['1v2'], targetPosition=0.4455 - V[4] * 0.001,
                                controlMode=p.POSITION_CONTROL, force=force)
        p.setJointMotorControl2(self.id, self.joint_name_to_id['1v3'], targetPosition=0.4455 - V[5] * 0.001,
                                controlMode=p.POSITION_CONTROL, force=force)
        p.setJointMotorControl2(self.id, self.joint_name_to_id['2v1'], targetPosition=0.4455 - V[6] * 0.001,
                                controlMode=p.POSITION_CONTROL, force=force)
        p.setJointMotorControl2(self.id, self.joint_name_to_id['2v2'], targetPosition=0.4455 - V[7] * 0.001,
                                controlMode=p.POSITION_CONTROL, force=force)
        p.setJointMotorControl2(self.id, self.joint_name_to_id['2v3'], targetPosition=0.4455 - V[8] * 0.001,
                                controlMode=p.POSITION_CONTROL, force=force)
        p.setJointMotorControl2(self.id, self.joint_name_to_id['3v1'], targetPosition=0.4455 - V[9] * 0.001,
                                controlMode=p.POSITION_CONTROL, force=force)
        p.setJointMotorControl2(self.id, self.joint_name_to_id['3v2'], targetPosition=0.4455 - V[10] * 0.001,
                                controlMode=p.POSITION_CONTROL, force=force)
        p.setJointMotorControl2(self.id, self.joint_name_to_id['3v3'], targetPosition=0.4455 - V[11] * 0.001,
                                controlMode=p.POSITION_CONTROL, force=force)

################################ SIMULATION ################################

physicsClient = p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())
planeId = p.loadURDF("plane.urdf")
p.setGravity(0, 0, -10)
megabot = Megabot()

FRAME_RATE = 1 / 10 # Fréquence à laquelle un ordre est transmis au vérins
TICK_RATE = 1 / 240 # Fréquence à laquelle le simulateur s'actualise
CYL_FORCE = 20000

pl.init()
LV = pl.legs_up(com_radius=150, passenger_weight=0)

# p.changeVisualShape(megabot.id, megabot.connectors[0][0], rgbaColor=[1, 0, 0, 1])
# p.changeVisualShape(megabot.id, megabot.connectors[0][1], rgbaColor=[1, 0, 0, 1])
# p.changeVisualShape(megabot.id, megabot.connectors[0][2], rgbaColor=[1, 1, 0, 1])
# p.changeVisualShape(megabot.id, megabot.connectors[0][3], rgbaColor=[1, 1, 0, 1])
# p.changeVisualShape(megabot.id, megabot.connectors[0][4], rgbaColor=[1, 0, 1, 1])
# p.changeVisualShape(megabot.id, megabot.connectors[0][5], rgbaColor=[1, 0, 1, 1])
# p.changeVisualShape(megabot.id, megabot.connectors[0][6], rgbaColor=[0, 1, 0, 1])
# p.changeVisualShape(megabot.id, megabot.connectors[0][7], rgbaColor=[0, 1, 0, 1])

# friction_v = 10
# restitution_v = 0
# p.changeDynamics(megabot.id, 3, lateralFriction=friction_v)
# p.changeDynamics(megabot.id, 21, lateralFriction=friction_v)
# p.changeDynamics(megabot.id, 39, lateralFriction=friction_v)
# p.changeDynamics(megabot.id, 57, lateralFriction=friction_v)
# p.changeDynamics(megabot.id, 3, restitution=restitution_v)
# p.changeDynamics(megabot.id, 21, restitution=restitution_v)
# p.changeDynamics(megabot.id, 39, restitution=restitution_v)
# p.changeDynamics(megabot.id, 57, restitution=restitution_v)

for i in range(20):
    p.stepSimulation()
    if i < 10:
        basePos = p.getBasePositionAndOrientation(megabot.id)[0]
        p.resetBasePositionAndOrientation(megabot.id, basePos, p.getQuaternionFromEuler([0, 0, 0]))
    else:
        megabot.MoveCylinders(pl.ROBOT['idle_pos'], CYL_FORCE)
    time.sleep(TICK_RATE)

elapsed_time = 0

for i in range(10000):
    p.stepSimulation()

    megabot.MoveCylinders(LV[int(elapsed_time / FRAME_RATE)], 20000)

    # megabot.MoveCylinders(pl.ROBOT['idle_pos'], CYL_FORCE)

    time.sleep(TICK_RATE)
    elapsed_time += TICK_RATE

p.disconnect()
