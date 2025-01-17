
#include <iostream>
#include <fstream>
#include <memory>
#include <cmath>
#include <chrono>

#include <echolib/client.h>
//#include <echolib/opencv.h>

using namespace std;
using namespace echolib;

#include <manus/manipulator.h>
#include <manus/files.h>

class SimulatedManipulator : public Manipulator {
public:
    SimulatedManipulator(const string& model, float speed);
    ~SimulatedManipulator();

	bool step(float time);
	
    virtual int size();
	virtual bool move(int joint, float speed, float position);	

	virtual ManipulatorDescription describe();
	virtual ManipulatorState state();
    virtual void prepareNewGoal(bool begin_trajectory);

private:

	float speed;

    ManipulatorDescription _description;
    ManipulatorState _state;

};

SimulatedManipulator::SimulatedManipulator(const string& model, float speed) : speed(speed) {

    string model_path = find_file(model);

    cout << model_path << endl;

    if (!parse_description(model_path, _description)) {
        throw ManipulatorException("Unable to parse manipulator model description");
    }

    for (int i = 0; i < _description.joints.size(); i++) {
        _state.joints.push_back(joint_state(_description.joints[i], _description.joints[i].safe));
    }

}

SimulatedManipulator::~SimulatedManipulator() {


}

#define MOVE_RESOLUTION_GRIP 0.5
#define MOVE_RESOLUTION_TRANSLATION 30
#define MOVE_RESOLUTION_ROTATION (M_PI / 2)

bool SimulatedManipulator::step(float time) {

    bool idle = true;

    for (int i = 0; i < _description.joints.size(); i++) {
        float resolution = _description.joints[i].type == JOINTTYPE_ROTATION ?
            MOVE_RESOLUTION_ROTATION : (_description.joints[i].type == JOINTTYPE_TRANSLATION ? MOVE_RESOLUTION_TRANSLATION : MOVE_RESOLUTION_GRIP);

        resolution *= (time / 1000.0) * _state.joints[i].speed * speed;

        if ((_state.joints[i].position - _state.joints[i].goal) > (resolution / 2)) {
            _state.joints[i].position -= resolution;
            _state.joints[i].type = JOINTSTATETYPE_MOVING;
            idle = false;
        }
        else if ((_state.joints[i].position - _state.joints[i].goal) < (-resolution / 2)) {
            _state.joints[i].position += resolution;
            _state.joints[i].type = JOINTSTATETYPE_MOVING;
            idle = false;
        }
        else {
            _state.joints[i].position = _state.joints[i].goal;
            _state.joints[i].type = JOINTSTATETYPE_IDLE;
        }

    }

    _state.state = idle ? MANIPULATORSTATETYPE_PASSIVE : MANIPULATORSTATETYPE_ACTIVE;

    _state.header.timestamp = std::chrono::system_clock::now();

    return idle;
}

int SimulatedManipulator::size() {
    return _state.joints.size();
}

bool SimulatedManipulator::move(int joint, float position, float speed) {

    if (joint < 0 || joint >= size())
        return false;

    if (_description.joints[joint].min > position)
        position = _description.joints[joint].min;

    if (_description.joints[joint].max < position)
        position = _description.joints[joint].max;

    _state.joints[joint].goal = position;
    _state.joints[joint].speed = speed;

	return true;
}

ManipulatorDescription SimulatedManipulator::describe() {

    return _description;

}

ManipulatorState SimulatedManipulator::state() {

    return _state;

}

void SimulatedManipulator::prepareNewGoal(bool begin_trajectory) {

}

#define SIMULATION_DELTA 50

int main(int argc, char** argv) {

    string description(get_env("MANUS_MANIPULATOR_MODEL", "manipulator.yaml"));

    if (argc > 1) {
        description = string(argv[1]);
    }  

    shared_ptr<SimulatedManipulator> manipulator = shared_ptr<SimulatedManipulator>(new SimulatedManipulator(description, 1));

    SharedClient client = echolib::connect(string(), "simulator");

    ManipulatorManager manager(client, manipulator);

    while (echolib::wait(SIMULATION_DELTA)) {        
        manager.update();
        manipulator->step(SIMULATION_DELTA);
    }

    exit(0);
}


