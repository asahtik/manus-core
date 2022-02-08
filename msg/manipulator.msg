import "geometry.msg";

enumerate ManipulatorStateType { UNKNOWN, CONNECTED, PASSIVE, ACTIVE, CALIBRATION }
enumerate JointType { ROTATION, TRANSLATION, GRIPPER, FIXED }
enumerate JointAxis { X, Y, Z, NONE }
enumerate JointStateType { IDLE, MOVING, ERROR }

enumerate PlanStateType { PLANNING, FAILED, PLANNED, RUNNING, STOPPED, COMPLETED }
 
structure JointDescription {
	JointType type;
	JointAxis axis;
	float tx;
	float ty;
	float tz;
	float rr;
	float rp;
	float ry;
	float min;
	float max;
	float safe;
}

structure JointState {
	JointStateType type;
	float position;
	float goal;
	float speed;
}

structure JointCommand {
	float goal;
	float speed;
}

structure PlanSegment {
	JointCommand[] joints;
}

structure TrajectorySegment {
	Frame3D frame;
	bool required = true;
	float gripper = 0;
	float speed = 1;
	bool rotation = true;
}

message ManipulatorDescription {
	string name;
	float version;
	Frame3D frame;
	JointDescription[] joints;
}

message ManipulatorState {
	header header;
	ManipulatorStateType state;
	JointState[] joints;
}

message PlanState {
	Timestamp timestamp;
	string identifier;
	PlanStateType type;
	long timestamp;
}

message Plan {
	string identifier;
	PlanSegment[] segments;
}

message Trajectory {
	string identifier;
	TrajectorySegment[] segments;
	float speed = 1;
}

