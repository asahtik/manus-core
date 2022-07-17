
import manus.messages as messages

import numpy as np

NAME = 'Manus'
VERSION = 'N/A'

try:
    with open('/usr/share/manus/version', 'r') as f:
        VERSION = f.read()
except IOError:
    pass

class MoveTo(object):
    def __init__(self, location, rotation = None, grip=0, speed=1.0):
        self.location = location
        self.rotation = rotation
        self.grip = grip
        self.speed = speed

    def generate(self, manipulator):
        seg = messages.TrajectorySegment(gripper=self.grip, required=True, speed= self.speed)
        seg.frame.origin = messages.Point3D(self.location[0], self.location[1], self.location[2])
        if self.rotation is None:
            seg.rotation = False
        else:
            seg.frame.rotation = messages.Rotation3D(self.rotation[0], self.rotation[1], self.rotation[2])
        return seg

class MoveJoints(object):
    def __init__(self, goal, speed=1.0):
        self.goal = goal
        if not speed is list:
            self.speed = [speed] * len(goal)
        else:
            self.speed = speed

    def generate(self, manipulator):
        seg = messages.PlanSegment()
        seg.joints = [messages.JointCommand(j, s) for j, s in zip(self.goal, self.speed)]
        return seg

class Manipulator(object):

    def __init__(self, client, name):
        self.name = name
        self.state = None
        self._client = client
        self._listeners = []
        self._description = messages.ManipulatorDescriptionSubscriber(client, "%s.description" % name, self._description_callback)
        self._state = messages.ManipulatorStateSubscriber(client, "%s.state" % name, self._state_callback)
        self._move = messages.PlanPublisher(client, "%s.plan" % name)
        self._planner = messages.TrajectoryPublisher(client, "%s.trajectory" % name)
        self._planstate = messages.PlanStateSubscriber(client, "%s.planstate" % name, self._planstate_callback)

    def listen(self, listener):
        self._listeners.append(listener)

    def unlisten(self, listener):
        try:
            self._listeners.remove(listener)
        except ValueError:
            pass

    @property
    def description(self):
        return self._description_data

    def move_safe(self, identifier='safe'):
        plan = messages.Plan()
        plan.identifier = identifier
        segment = messages.PlanSegment()
        plan.segments.append(segment)
        self._move.send(plan)
        
    def move(self, identifier, states):
        plan = messages.Plan()
        plan.identifier = identifier
        plan.segments = [s.generate(self) for s in states]
        self._move.send(plan)

    def move_joint(self, joint, goal, speed = 1.0, identifier='move joint'):
        segment = self._state_to_segment()
        segment.joints[joint].speed = speed
        segment.joints[joint].goal = goal
        plan = messages.Plan(identifier = identifier)
        plan.segments.append(segment)
        self._move.send(plan)
        self.state.joints[joint].goal = goal

    def trajectory(self, identifier, goals):
        msg = messages.Trajectory()
        msg.identifier = identifier
        msg.segments = [s.generate(self) for s in goals]
        self._planner.send(msg)

    def _state_callback(self, state):
        self.state = state
        for s in self._listeners:
            s.on_manipulator_state(self, state)

    def _description_callback(self, description):
        self._description_data = description

    def _state_to_segment(self):
        segment = messages.PlanSegment()
        for j in self.state.joints:
            segment.joints.append(messages.JointCommand(j.goal, j.speed))
        return segment

    def _planstate_callback(self, state):
        for s in self._listeners:
            s.on_planner_state(self, state)

    def _mat_mult_vec(m, v):
        return np.array([
            m[0] * v[0] + m[1] * v[1] + m[2] * v[2],
            m[3] * v[0] + m[4] * v[1] + m[5] * v[2],
            m[6] * v[0] + m[7] * v[1] + m[8] * v[2]
        ])

    def _mat_mult_mat(m1, m2):
        return np.array([
            m1[0] * m2[0] + m1[1] * m2[3] + m1[2] * m2[6],
            m1[0] * m2[1] + m1[1] * m2[4] + m1[2] * m2[7],
            m1[0] * m2[2] + m1[1] * m2[5] + m1[2] * m2[8],
            m1[3] * m2[0] + m1[4] * m2[3] + m1[5] * m2[6],
            m1[3] * m2[1] + m1[4] * m2[4] + m1[5] * m2[7],
            m1[3] * m2[2] + m1[4] * m2[5] + m1[5] * m2[8],
            m1[6] * m2[0] + m1[7] * m2[3] + m1[8] * m2[6],
            m1[6] * m2[1] + m1[7] * m2[4] + m1[8] * m2[7],
            m1[6] * m2[2] + m1[7] * m2[5] + m1[8] * m2[8]
        ])

    def _vec_add_vec(v1, v2):
        return np.array([
            v1[0] + v2[0],
            v1[1] + v2[1],
            v1[2] + v2[2]
        ])

    def transform(self, joint):
        origin = np.array([
            1, 0, 0, 0,
            0, 1, 0, 0,
            0, 0, 1, 0,
            0, 0, 0, 1
        ])
        rotation = np.array([
            1, 0, 0,
            0, 1, 0,
            0, 0, 1
        ])
        translation = np.array([0, 0, 0])
        if self.description is None or joint < 0 or joint >= len(self.state.joints):
            return origin
        for j in xrange(0, joint+1):
            tx, ty, tz, rr, rp, ry = \
                self._description_data.joints[j].tx, \
                self._description_data.joints[j].ty, \
                self._description_data.joints[j].tz, \
                self._description_data.joints[j].rr, \
                self._description_data.joints[j].rp, \
                self._description_data.joints[j].ry

            qrotation = np.array([
                1, 0, 0,
                0, 1, 0,
                0, 0, 1
            ])
            qtranslation = np.array([0, 0, 0])
            q_pos = self.state.joints[j].position
            if self._description_data.joints[j].type == messages.JointType.ROTATION:
                if self._description_data.joints[j].axis == messages.JointAxis.X:
                    qrotation = np.array([
                        1, 0, 0,
                        0, np.cos(q_pos), -np.sin(q_pos),
                        0, np.sin(q_pos), np.cos(q_pos)
                    ])
                elif self._description_data.joints[j].axis == messages.JointAxis.Y:
                    qrotation = np.array([
                        np.cos(q_pos), 0, np.sin(q_pos),
                        0, 1, 0,
                        -np.sin(q_pos), 0, np.cos(q_pos)
                    ])
                elif self._description_data.joints[j].axis == messages.JointAxis.Z:
                    qrotation = np.array([
                        np.cos(q_pos), -np.sin(q_pos), 0,
                        np.sin(q_pos), np.cos(q_pos), 0,
                        0, 0, 1
                    ])
            elif self._description_data.joints[j].type == messages.JointType.TRANSLATION:
                if self._description_data.joints[j].axis == messages.JointAxis.X:
                    qtranslation = np.array([q_pos, 0, 0])
                elif self._description_data.joints[j].axis == messages.JointAxis.Y:
                    qtranslation = np.array([0, q_pos, 0])
                elif self._description_data.joints[j].axis == messages.JointAxis.Z:
                    qtranslation = np.array([0, 0, q_pos])

            ca1 = np.cos(ry)
            sa1 = np.sin(ry)
            cb1 = np.cos(rp)
            sb1 = np.sin(rp)
            cc1 = np.cos(rr)
            sc1 = np.sin(rr)

            lrotation = np.array([
                ca1*cb1, ca1*sb1*sc1 - sa1*cc1, ca1*sb1*cc1 + sa1*sc1,
                sa1*cb1, sa1*sb1*sc1 + ca1*cc1, sa1*sb1*cc1 - ca1*sc1,
                -sb1, cb1*sc1, cb1*cc1
            ])
            ltranslation = np.array([tx, ty, tz])

            ltranslation = self.vec_add_vec(qtranslation, self.mat_mult_vec(qrotation, ltranslation))
            lrotation = self.mat_mult_mat(qrotation, lrotation)
            translation = self.vec_add_vec(translation, self.mat_mult_vec(rotation, ltranslation))
            rotation = self.mat_mult_mat(rotation, lrotation)

        return np.array([
            [rotation[0], rotation[1], rotation[2], translation[0]],
            [rotation[3], rotation[4], rotation[5], translation[1]],
            [rotation[6], rotation[7], rotation[8], translation[2]],
            [0, 0, 0, 1]
        ])
