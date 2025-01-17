
#ifndef __MANIPULATOR_H
#define __MANIPULATOR_H

#include <string>
#include <vector>

#define RADIAN_TO_DEGREE(X) ((X * 180) / M_PI )
#define DEGREE_TO_RADIAN(X) ((X / 180) * M_PI )

using namespace std;

#include <manus/messages.h>

#include <echolib/client.h>
#include <echolib/helpers.h>
#include <echolib/datatypes.h>

using namespace echolib;
using namespace manus::messages;

class Manipulator {
public:
	Manipulator() {};
	~Manipulator() {};

	virtual int size() = 0;
	virtual bool move(int joint, float goal, float speed) = 0;

	virtual ManipulatorDescription describe() = 0;
	virtual ManipulatorState state() = 0;
};

class ManipulatorException : public std::runtime_error
{
public:
    ManipulatorException(char const* const message) throw();
    virtual char const* what() const throw();
};

bool parse_description(const string& filename, ManipulatorDescription& manipulator);

string manipulator_state_string(ManipulatorStateType status);
string joint_type_string(JointType type);

JointDescription joint_description(JointType type, JointAxis axis, float rr, float rp, float ry, float tx, float ty, float tz, float min, float max);

JointState joint_state(const JointDescription& joint, float position, JointStateType type = JOINTSTATETYPE_IDLE);


class ManipulatorManager {
public:
    ManipulatorManager(SharedClient client, shared_ptr<Manipulator> manipulator);

    virtual ~ManipulatorManager();

    void flush();

    void push(shared_ptr<Plan> t);

    void update();

protected:

	virtual void on_subscribers(int s);

private:

    void step(bool force = false);

    SharedClient client;

    int subscribers = 0;

	SubscriptionWatcher watcher;

    shared_ptr<Manipulator> manipulator;

    shared_ptr<Plan> plan;

    SharedTypedPublisher<ManipulatorState> state_publisher;
    SharedTypedPublisher<PlanState> planstate_publisher;
    shared_ptr<StaticPublisher<ManipulatorDescription> > description_publisher;
    SharedTypedSubscriber<Plan> plan_listener;
};

#endif

