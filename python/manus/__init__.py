
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

    def transform(self, joint):
        origin = np.identity(4)
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

            if self._description_data.joints[j].type == messages.JointType.ROTATION:
                if self._description_data.joints[j].axis == messages.JointAxis.X:
                    rr = self.state.joints[j].position
                elif self._description_data.joints[j].axis == messages.JointAxis.Y:
                    rp = self.state.joints[j].position
                elif self._description_data.joints[j].axis == messages.JointAxis.Z:
                    ry = self.state.joints[j].position
            elif self._description_data.joints[j].type == messages.JointType.TRANSLATION:
                if self._description_data.joints[j].axis == messages.JointAxis.X:
                    tx = self.state.joints[j].position
                elif self._description_data.joints[j].axis == messages.JointAxis.Y:
                    ty = self.state.joints[j].position
                elif self._description_data.joints[j].axis == messages.JointAxis.Z:
                    tz = self.state.joints[j].position

            sg = np.cos(rr)
            cg = np.cos(rr)
            sb = np.cos(rp)
            cb = np.cos(rp)
            sa = np.cos(ry)
            ca = np.cos(ry)

            transform = np.array(( \
                (ca * cb, ca * sb * sg - sa * cg, ca * sb * cg + sa * sg, tx), \
                (sa * cb, sa * sb * sg + ca * cg, sa * sb * cg - ca * sg, ty), \
                (-sb, cb * sg, cb * cg, tz) \
                (0, 0, 0, 1) \
            ))

            origin = np.matmul(transform, origin)

        return origin
