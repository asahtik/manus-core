
#include <iostream>
#include <memory>
#include <functional>
#include <cmath>

#include <yaml-cpp/yaml.h>

#include <manus/manipulator.h>

using namespace std;

ManipulatorException::ManipulatorException(char const* const message) throw()
    : std::runtime_error(message) {

}

char const * ManipulatorException::what() const throw() {
    return std::runtime_error::what();
}

string manipulator_state_string(ManipulatorStateType status) {

    switch (status) {
    case MANIPULATORSTATETYPE_UNKNOWN: return "unknown";
    case MANIPULATORSTATETYPE_CONNECTED: return "connected";
    case MANIPULATORSTATETYPE_PASSIVE: return "passive";
    case MANIPULATORSTATETYPE_ACTIVE: return "active";
    case MANIPULATORSTATETYPE_CALIBRATION: return "calibration";
    }

    return "unknown";
}

string joint_type_string(JointType type) {

    switch (type) {
    case JOINTTYPE_ROTATION: return "rotation";
    case JOINTTYPE_TRANSLATION: return "translation";
    case JOINTTYPE_FIXED: return "fixed";
    case JOINTTYPE_GRIPPER: return "gripper";
    }

    return "unknown";
}

JointAxis joint_axis_enum(string axis) {
    if (axis == "x") return JOINTAXIS_X;
    else if (axis == "y") return JOINTAXIS_Y;
    else if (axis == "none") return JOINTAXIS_NONE;
    else return JOINTAXIS_Z;
}

bool parse_joint(const YAML::Node& node, JointDescription& joint) {
    string type = node["type"].as<string>();
    if (type == "rotation") {
        joint.type = JOINTTYPE_ROTATION;
        joint.axis = joint_axis_enum(node["axis"].as<string>());
        joint.tx = node["transform"]["tx"].as<float>();
        joint.ty = node["transform"]["ty"].as<float>();
        joint.tz = node["transform"]["tz"].as<float>();
        joint.rr = DEGREE_TO_RADIAN(node["transform"]["rr"].as<float>());
        joint.rp = DEGREE_TO_RADIAN(node["transform"]["rp"].as<float>());
        joint.ry = DEGREE_TO_RADIAN(node["transform"]["ry"].as<float>());
        joint.min = DEGREE_TO_RADIAN(node["min"].as<float>());
        joint.max = DEGREE_TO_RADIAN(node["max"].as<float>());
        joint.safe = DEGREE_TO_RADIAN(node["safe"].as<float>());

        switch (joint.axis) {
            case JOINTAXIS_X:
                joint.rr = joint.safe;
            break;
            case JOINTAXIS_Y:
                joint.rp = joint.safe;
            break;
            case JOINTAXIS_Z:
                joint.ry = joint.safe;
            break;
        }
        return true;
    }
    if (type == "translation") {
        joint.type = JOINTTYPE_TRANSLATION;
        joint.axis = joint_axis_enum(node["axis"].as<string>());
        joint.tx = node["transform"]["tx"].as<float>();
        joint.ty = node["transform"]["ty"].as<float>();
        joint.tz = node["transform"]["tz"].as<float>();
        joint.rr = DEGREE_TO_RADIAN(node["transform"]["rr"].as<float>());
        joint.rp = DEGREE_TO_RADIAN(node["transform"]["rp"].as<float>());
        joint.ry = DEGREE_TO_RADIAN(node["transform"]["ry"].as<float>());
        joint.min = DEGREE_TO_RADIAN(node["min"].as<float>());
        joint.max = DEGREE_TO_RADIAN(node["max"].as<float>());
        joint.safe = DEGREE_TO_RADIAN(node["safe"].as<float>());

        switch (joint.axis) {
            case JOINTAXIS_X:
                joint.tx = joint.safe;
            break;
            case JOINTAXIS_Y:
                joint.ty = joint.safe;
            break;
            case JOINTAXIS_Z:
                joint.tz = joint.safe;
            break;
        }
        return true;
    }
    if (type == "fixed") {
        joint.type = JOINTTYPE_FIXED;
        joint.axis = JOINTAXIS_NONE;
        joint.tx = node["transform"]["tx"].as<float>();
        joint.ty = node["transform"]["ty"].as<float>();
        joint.tz = node["transform"]["tz"].as<float>();
        joint.rr = DEGREE_TO_RADIAN(node["transform"]["rr"].as<float>());
        joint.rp = DEGREE_TO_RADIAN(node["transform"]["rp"].as<float>());
        joint.ry = DEGREE_TO_RADIAN(node["transform"]["ry"].as<float>());
        return true;
    }
    if (type == "gripper") {
        joint.type = JOINTTYPE_GRIPPER;
        joint.axis = JOINTAXIS_NONE;
        joint.min = node["min"].as<float>();
        joint.max = node["max"].as<float>();
        return true;
    }
    return false;
}

bool parse_description(const string& filename, ManipulatorDescription& manipulator) {

    YAML::Node doc = YAML::LoadFile(filename);

    manipulator.name = doc["name"].as<string>();
    manipulator.version = doc["version"].as<float>();

    const YAML::Node& joints = doc["joints"];
    manipulator.joints.clear();

    for (int i = 0; i < joints.size(); i++) {
        JointDescription d;
        parse_joint(joints[i], d);
        manipulator.joints.push_back(d);
    }

    return true;
}

JointDescription joint_description(JointType type, JointAxis axis, float tx, float ty, float tz, float rr, float rp, float ry, float min, float max) {
    JointDescription joint;
    joint.type = type;
    joint.axis = axis;
    joint.tx = tx;
    joint.ty = ty;
    joint.tz = tz;
    joint.rr = DEGREE_TO_RADIAN(rr);
    joint.rp = DEGREE_TO_RADIAN(rp);
    joint.ry = DEGREE_TO_RADIAN(ry);
    joint.min = type == JOINTTYPE_ROTATION ? DEGREE_TO_RADIAN(min) : min;
    joint.max = type == JOINTTYPE_ROTATION ? DEGREE_TO_RADIAN(max) : max;
    return joint;
}

JointState joint_state(const JointDescription& joint, float position, JointStateType type) {
    JointState state;
    state.type = type;
    state.position = joint.type == JOINTTYPE_ROTATION ? DEGREE_TO_RADIAN(position) : position;
    state.goal = joint.type == JOINTTYPE_ROTATION ? DEGREE_TO_RADIAN(position) : position;
    return state;
}

bool close_enough(float a, float b) {
    return std::fabs(a - b) < 0.05;
}

inline float normalizeAngle(float val, const float min, const float max) {

    if (val > max) {
        //Find actual angle offset
        float diffangle = fmod(val - max, 2 * M_PI);
        // Add that to upper bound and go back a full rotation
        val = max + diffangle - 2 * M_PI;
    }

    if (val < min) {
        //Find actual angle offset
        float diffangle = fmod(min - val, 2 * M_PI);
        // Add that to upper bound and go back a full rotation
        val = min - diffangle + 2 * M_PI;
    }
    return val;
}

ManipulatorManager::ManipulatorManager(SharedClient client, shared_ptr<Manipulator> manipulator): manipulator(manipulator), client(client),
    watcher(client, "state", std::bind(&ManipulatorManager::on_subscribers, this, std::placeholders::_1)), subscribers(0) {

    state_publisher = make_shared<TypedPublisher<ManipulatorState> >(client, "state");

    planstate_publisher = make_shared<TypedPublisher<PlanState> >(client, "planstate");

    plan_listener = make_shared<TypedSubscriber<Plan> >(client, "plan",
    [this](shared_ptr<Plan> param) {
        this->push(param);
    });

}

ManipulatorManager::~ManipulatorManager() {

    flush();

}

void ManipulatorManager::flush() {

    plan.reset();
    /*
        ManipulatorState state = manipulator->state();

        for (size_t i = 0; i < manipulator->size(); i++) {
            manipulator->move(i, state.joints[i].position, 1);
        }
    */
}

void ManipulatorManager::push(shared_ptr<Plan> t) {

    ManipulatorDescription description = manipulator->describe();

    cout << "Received new plan" << endl;

    for (size_t s = 0; s < t->segments.size(); s++) {
        // Move to safe position if no joint given
        if (t->segments[s].joints.size() == 0) {
            cout << "Moving to safe position" << endl;
            for (size_t j = 0; j < manipulator->size(); j++) {
                JointCommand tmp;
                tmp.goal = description.joints[j].safe;
                tmp.speed = 1;
                t->segments[s].joints.push_back(tmp);
            }
        }

        if (t->segments[s].joints.size() != manipulator->size()) {
            cout << "Wrong manipulator size " << t->segments[s].joints.size() << " vs. " << manipulator->size() << endl;
            return;
        }
        for (size_t j = 0; j < manipulator->size(); j++) {
            float goal = t->segments[s].joints[j].goal;
            if (description.joints[j].type == JOINTTYPE_ROTATION) {
                goal = normalizeAngle(goal, description.joints[j].min, description.joints[j].max);
            }

            if (goal < description.joints[j].min ||
                    t->segments[s].joints[j].goal > description.joints[j].max) {
                if (description.joints[j].type != JOINTTYPE_FIXED) {
                    cout << "Warning: wrong joint " << j << " goal " << goal << " out of range " << description.joints[j].min << " to " << description.joints[j].max << ". Truncating." << endl;
                    goal = max(description.joints[j].min, min(description.joints[j].max, goal));
                } else {
                    goal = description.joints[j].min;
                }
            }
            t->segments[s].joints[j].goal = goal;
        }
    }

    flush();

    PlanState state;
    state.identifier = t->identifier;
    state.type = PLANSTATETYPE_RUNNING;
    planstate_publisher->send(state);

    plan = t;

    step(true);

}

void ManipulatorManager::update() {

    if (manipulator->state().state != MANIPULATORSTATETYPE_UNKNOWN && !description_publisher) {
        cout << "Manipulator ready" << endl;
        ManipulatorDescription description = manipulator->describe();
        description_publisher = make_shared<StaticPublisher<ManipulatorDescription> >(client, "description", description);
    }

    if (manipulator->state().state != MANIPULATORSTATETYPE_PASSIVE && manipulator->state().state != MANIPULATORSTATETYPE_UNKNOWN) {

        state_publisher->send(manipulator->state());
    }

    step();
}

void ManipulatorManager::on_subscribers(int s) {
    if (s > subscribers) {
        state_publisher->send(manipulator->state());
    }
    subscribers = s;
}

void ManipulatorManager::step(bool force) {

    if (!plan) return;

    bool idle = true;
    bool goal = true;

    ManipulatorState state = manipulator->state();
    for (size_t i = 0; i < manipulator->size(); i++) {
        idle &= state.joints[i].type == JOINTSTATETYPE_IDLE;
        goal &= close_enough(state.joints[i].position, state.joints[i].goal);
    }

    if (goal || force) {
        int planSize = plan->segments.size();
        if (planSize == 0) {
            PlanState state;
            state.identifier = plan->identifier;
            state.type = PLANSTATETYPE_COMPLETED;
            planstate_publisher->send(state);
            plan.reset();

            lastPlanSize = 0;
            return;
        }

        manipulator->prepareNewGoal(lastPlanSize == 0);
        for (size_t i = 0; i < manipulator->size(); i++) {
            manipulator->move(i, plan->segments[0].joints[i].goal, plan->segments[0].joints[i].speed);
        }

        lastPlanSize = planSize;
        plan->segments.erase(plan->segments.begin());
    }

}

