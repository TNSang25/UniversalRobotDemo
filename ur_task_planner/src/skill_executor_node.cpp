#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/msg/attached_collision_object.hpp>
#include <moveit_msgs/msg/collision_object.hpp>
#include <moveit_msgs/msg/object_color.hpp>
#include <shape_msgs/msg/solid_primitive.hpp>
#include <std_msgs/msg/color_rgba.hpp>
#include <std_msgs/msg/string.hpp>
#include <ur_task_planner/srv/execute_skill.hpp>
#include <ur_task_planner/msg/observed_scene.hpp>
#include <ur_task_planner/observation_recovery.hpp>
#include <condition_variable>
#include <mutex>
#include <set>
#include <iomanip>
#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <functional>
#include <map>
#include <memory>
#include <sstream>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

using namespace std::chrono_literals;
using moveit::planning_interface::MoveGroupInterface;
using moveit::planning_interface::PlanningSceneInterface;

namespace {
const char* const ARM_GROUP = "ur_manipulator";
const char* const GRIPPER_GROUP = "gripper";
const char* const BASE_FRAME = "base_link";
const char* const TOOL_LINK = "tool0";
const char* const LOCATION_TABLE = "table";
const char* const LOCATION_GRIPPER = "gripper";
const char* const LOCATION_UNKNOWN = "unknown";
const std::vector<std::string> GRIPPER_LINKS = {"gripper_base_link", "left_finger", "right_finger"};
const double EEF_STEP = 0.005;
}  // namespace

class SkillExecutorNode : public rclcpp::Node {
public:
    SkillExecutorNode(const rclcpp::NodeOptions& options)
        : Node("skill_executor_node", options) {

        load_scene();

        // Mutually exclusive: skills are executed one at a time
        callback_group_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

        service_ = this->create_service<ur_task_planner::srv::ExecuteSkill>(
            "execute_skill",
            std::bind(&SkillExecutorNode::handle_skill_request, this, std::placeholders::_1, std::placeholders::_2),
            rmw_qos_profile_services_default,
            callback_group_);

        observation_group_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
        rclcpp::SubscriptionOptions observation_options;
        observation_options.callback_group = observation_group_;
        observation_subscription_ = this->create_subscription<ur_task_planner::msg::ObservedScene>(
            "observed_scene", rclcpp::QoS(1),
            [this](ur_task_planner::msg::ObservedScene::SharedPtr msg) {
                std::lock_guard<std::mutex> lock(observation_mutex_);
                latest_observation_ = msg;
                observation_changed_.notify_all();
            }, observation_options);

        state_publisher_ = this->create_publisher<std_msgs::msg::String>(
            "scene_state", rclcpp::QoS(1).transient_local());
    }

    void init_move_group() {
        const double startup_timeout = param("startup_timeout_sec", 120.0);
        if (!std::isfinite(startup_timeout) || startup_timeout <= 0.0) {
            throw std::runtime_error("startup_timeout_sec must be a positive finite number");
        }
        const auto server_timeout = rclcpp::Duration::from_seconds(startup_timeout);
        move_group_ = std::make_shared<MoveGroupInterface>(
            shared_from_this(), ARM_GROUP, nullptr, server_timeout);
        move_group_->setPoseReferenceFrame(BASE_FRAME);
        move_group_->setEndEffectorLink(TOOL_LINK);
        move_group_->setPlanningTime(10.0);
        move_group_->setNumPlanningAttempts(5);
        move_group_->setMaxVelocityScalingFactor(arm_velocity_scaling_);
        move_group_->setMaxAccelerationScalingFactor(arm_acceleration_scaling_);

        gripper_group_ = std::make_shared<MoveGroupInterface>(
            shared_from_this(), GRIPPER_GROUP, nullptr, server_timeout);
        gripper_group_->setPlanningTime(5.0);

        // Wait for controller joint states and /clock instead of assuming a fast startup.
        if (!move_group_->getCurrentState(startup_timeout) ||
            !gripper_group_->getCurrentState(startup_timeout)) {
            throw std::runtime_error(
                "No current robot state; check /clock, /joint_states and active Gazebo controllers");
        }

        planning_scene_ = std::make_shared<PlanningSceneInterface>();
        if (!add_scene_to_moveit()) {
            throw std::runtime_error("could not add the collision objects to the planning scene");
        }

        publish_scene_state();
        ready_ = true;
        RCLCPP_INFO(this->get_logger(), "Skill Executor Node ready.");
    }

private:
    struct Box {
        double x, y, z;  // centre, in the base_link frame
        double yaw;
        double size_x, size_y, size_z;
    };

    struct Object {
        Box box;
        std_msgs::msg::ColorRGBA color;
        std::string location;  // "table", "gripper", "unknown" or the name of a zone
    };

    struct Zone {
        double x, y, z;  // centre of the surface, in the base_link frame
        double size_x, size_y;
    };

    struct Result {
        std::string status;
        std::string message;
        bool ok() const { return status == "SUCCESS"; }
    };

    rclcpp::CallbackGroup::SharedPtr callback_group_;
    rclcpp::Service<ur_task_planner::srv::ExecuteSkill>::SharedPtr service_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr state_publisher_;
    std::shared_ptr<MoveGroupInterface> move_group_;
    std::shared_ptr<MoveGroupInterface> gripper_group_;
    std::shared_ptr<PlanningSceneInterface> planning_scene_;
    std::atomic<bool> ready_{false};

    std::map<std::string, Object> objects_;
    std::map<std::string, Zone> zones_;
    std::map<std::string, Box> obstacles_;
    std::string held_object_;
    uint64_t state_revision_{0};
    std::vector<double> base_, table_bounds_;
    double table_surface_z_, placement_gap_, robot_keepout_, placement_reach_;
    bool use_perception_, observed_{false};
    double observation_timeout_;
    rclcpp::CallbackGroup::SharedPtr observation_group_;
    rclcpp::Subscription<ur_task_planner::msg::ObservedScene>::SharedPtr observation_subscription_;
    ur_task_planner::msg::ObservedScene::SharedPtr latest_observation_;
    std::mutex observation_mutex_;
    std::condition_variable observation_changed_;

    double tcp_offset_;
    double approach_distance_;
    double place_approach_distance_;
    double place_clearance_;
    double collision_margin_;
    double gripper_open_position_;
    double gripper_grasp_position_;
    double gripper_closed_position_;
    bool verify_grasp_;
    double arm_velocity_scaling_;
    double arm_acceleration_scaling_;
    double cartesian_speed_scaling_;
    double cartesian_jump_threshold_;

    // ---------------------------------------------------------------- parameters

    template <typename T>
    T param(const std::string& name, const T& default_value) {
        if (!this->has_parameter(name)) {
            this->declare_parameter<T>(name, default_value);
        }
        return this->get_parameter(name).get_value<T>();
    }

    std::vector<double> vector_param(const std::string& name, size_t size) {
        if (!this->has_parameter(name)) {
            throw std::runtime_error("parameter '" + name + "' is not set, load config/scene.yaml");
        }
        const std::vector<double> value = this->get_parameter(name).as_double_array();
        if (value.size() != size) {
            throw std::runtime_error("parameter '" + name + "' must have " + std::to_string(size) + " values");
        }
        return value;
    }

    std::vector<std::string> names_param(const std::string& name) {
        if (!this->has_parameter(name)) {
            throw std::runtime_error("parameter '" + name + "' is not set, load config/scene.yaml");
        }
        return this->get_parameter(name).as_string_array();
    }

    // Positions of the configuration are given in the Gazebo world frame
    void load_scene() {
        const std::vector<double> base = vector_param("robot_base_position", 3);
        base_ = base;
        use_perception_ = param("use_perception", false);
        observation_timeout_ = param("observation_timeout_sec", 5.0);
        table_bounds_ = param("table_placement_bounds", std::vector<double>{-0.25, 0.015, -0.34, 0.34});
        table_surface_z_ = param("table_surface_z", 0.8) - base[2];
        placement_gap_ = param("placement_gap", 0.015);
        robot_keepout_ = param("robot_keepout_radius", 0.12);
        placement_reach_ = param("placement_max_reach", 0.56);
        if (table_bounds_.size() != 4 || table_bounds_[0] >= table_bounds_[1] ||
            table_bounds_[2] >= table_bounds_[3] || observation_timeout_ <= 0.0) {
            throw std::runtime_error("invalid table placement bounds or observation timeout");
        }

        for (const std::string& name : names_param("object_names")) {
            const std::string prefix = "objects." + name + ".";
            const std::vector<double> position = vector_param(prefix + "position", 3);
            const std::vector<double> size = vector_param(prefix + "size", 3);
            const std::vector<double> color = param(prefix + "color", std::vector<double>{0.5, 0.5, 0.5});

            Object object;
            object.box = {position[0] - base[0], position[1] - base[1], position[2] - base[2],
                          param(prefix + "yaw", 0.0), size[0], size[1], size[2]};
            object.color.r = color.size() > 0 ? color[0] : 0.5;
            object.color.g = color.size() > 1 ? color[1] : 0.5;
            object.color.b = color.size() > 2 ? color[2] : 0.5;
            object.color.a = 1.0;
            object.location = use_perception_ ? LOCATION_UNKNOWN : LOCATION_TABLE;
            objects_[name] = object;
        }

        for (const std::string& name : names_param("zone_names")) {
            const std::vector<double> position = vector_param("zones." + name + ".position", 3);
            const auto size = param("zones." + name + ".size", std::vector<double>{0.15, 0.15});
            if (size.size() != 2) throw std::runtime_error("zone size must be XY");
            zones_[name] = {position[0] - base[0], position[1] - base[1], position[2] - base[2],
                            size[0], size[1]};
        }

        for (const std::string& name : param("obstacle_names", std::vector<std::string>{})) {
            const std::string prefix = "obstacles." + name + ".";
            const std::vector<double> position = vector_param(prefix + "position", 3);
            const std::vector<double> size = vector_param(prefix + "size", 3);
            obstacles_[name] = {position[0] - base[0], position[1] - base[1], position[2] - base[2],
                                param(prefix + "yaw", 0.0), size[0], size[1], size[2]};
        }

        tcp_offset_ = param("tcp_offset", 0.20);
        approach_distance_ = param("approach_distance", 0.10);
        // The trays are near UR3e's reach limit. A 10 cm place hover above
        // zone A/C has no IK even though the release pose is reachable.
        place_approach_distance_ = param("place_approach_distance", 0.05);
        if (!std::isfinite(place_approach_distance_) || place_approach_distance_ <= 0.0) {
            throw std::runtime_error("place_approach_distance must be a positive finite number");
        }
        place_clearance_ = param("place_clearance", 0.003);
        collision_margin_ = param("collision_margin", 0.002);
        gripper_open_position_ = param("gripper_open_position", 0.0);
        gripper_grasp_position_ = param("gripper_grasp_position", 0.023);
        gripper_closed_position_ = param("gripper_closed_position", 0.04);
        verify_grasp_ = param("verify_grasp", true);
        arm_velocity_scaling_ = param("arm_velocity_scaling", 0.3);
        arm_acceleration_scaling_ = param("arm_acceleration_scaling", 0.3);
        cartesian_speed_scaling_ = param("cartesian_speed_scaling", 0.2);
        cartesian_jump_threshold_ = param("cartesian_jump_threshold", 5.0);
    }

    // ------------------------------------------------------------ planning scene

    static moveit_msgs::msg::CollisionObject make_collision_box(const std::string& id, const Box& box) {
        moveit_msgs::msg::CollisionObject collision_object;
        collision_object.header.frame_id = BASE_FRAME;
        collision_object.id = id;
        collision_object.pose.position.x = box.x;
        collision_object.pose.position.y = box.y;
        collision_object.pose.position.z = box.z;
        collision_object.pose.orientation.z = std::sin(box.yaw / 2.0);
        collision_object.pose.orientation.w = std::cos(box.yaw / 2.0);

        shape_msgs::msg::SolidPrimitive primitive;
        primitive.type = primitive.BOX;
        primitive.dimensions.resize(3);
        primitive.dimensions[primitive.BOX_X] = box.size_x;
        primitive.dimensions[primitive.BOX_Y] = box.size_y;
        primitive.dimensions[primitive.BOX_Z] = box.size_z;

        geometry_msgs::msg::Pose primitive_pose;
        primitive_pose.orientation.w = 1.0;

        collision_object.primitives.push_back(primitive);
        collision_object.primitive_poses.push_back(primitive_pose);
        collision_object.operation = collision_object.ADD;
        return collision_object;
    }

    // Objects are made slightly smaller than they are: an object resting on the
    // table would otherwise be reported as colliding with it once it is attached.
    moveit_msgs::msg::CollisionObject make_object(const std::string& name) const {
        Box box = objects_.at(name).box;
        box.size_x -= 2.0 * collision_margin_;
        box.size_y -= 2.0 * collision_margin_;
        box.size_z -= 2.0 * collision_margin_;
        return make_collision_box(name, box);
    }

    moveit_msgs::msg::ObjectColor make_color(const std::string& name) const {
        moveit_msgs::msg::ObjectColor color;
        color.id = name;
        color.color = objects_.at(name).color;
        return color;
    }

    bool add_scene_to_moveit() {
        // Objects left in the gripper by a previous run go back to the world
        for (const auto& attached : planning_scene_->getAttachedObjects()) {
            if (objects_.count(attached.first) > 0 && !release_attached_object(attached.first)) {
                return false;
            }
        }

        std::vector<moveit_msgs::msg::CollisionObject> collision_objects;
        std::vector<moveit_msgs::msg::ObjectColor> colors;
        for (const auto& obstacle : obstacles_) {
            collision_objects.push_back(make_collision_box(obstacle.first, obstacle.second));
        }
        for (const auto& object : objects_) {
            if (object.second.location == LOCATION_UNKNOWN) continue;
            collision_objects.push_back(make_object(object.first));
            colors.push_back(make_color(object.first));
        }
        return planning_scene_->applyCollisionObjects(collision_objects, colors);
    }

    bool attach_object(const std::string& name) {
        moveit_msgs::msg::AttachedCollisionObject attached;
        attached.link_name = TOOL_LINK;
        attached.object.id = name;
        attached.object.operation = moveit_msgs::msg::CollisionObject::ADD;
        attached.touch_links = GRIPPER_LINKS;
        return planning_scene_->applyAttachedCollisionObject(attached);
    }

    bool release_attached_object(const std::string& name) {
        moveit_msgs::msg::AttachedCollisionObject attached;
        attached.link_name = TOOL_LINK;
        attached.object.id = name;
        attached.object.operation = moveit_msgs::msg::CollisionObject::REMOVE;
        return planning_scene_->applyAttachedCollisionObject(attached);
    }

    // Detach the object and put it in the world at the pose stored in objects_
    bool detach_object(const std::string& name) {
        if (!release_attached_object(name)) {
            return false;
        }
        return planning_scene_->applyCollisionObjects({make_object(name)}, {make_color(name)});
    }

    std::string zone_occupant(const std::string& zone_name) const {
        for (const auto& object : objects_) {
            if (object.second.location != LOCATION_UNKNOWN && object.second.location != LOCATION_GRIPPER &&
                overlaps_zone(object.second.box.x, object.second.box.y,
                              footprint_radius(object.second.box), zones_.at(zone_name))) {
                return object.first;
            }
        }
        return "";
    }

    std::string scene_state_json() const {
        std::ostringstream json;
        json << std::setprecision(10);
        json << "{\"revision\": " << state_revision_ << ", \"holding\": ";
        if (held_object_.empty()) {
            json << "null";
        } else {
            json << "\"" << held_object_ << "\"";
        }
        json << ", \"objects\": {";
        bool first = true;
        for (const auto& object : objects_) {
            json << (first ? "" : ", ") << "\"" << object.first << "\": \"" << object.second.location << "\"";
            first = false;
        }
        json << "}, \"observed\": " << (observed_ ? "true" : "false") << ", \"positions\": {";
        first = true;
        for (const auto& entry : objects_) {
            if (entry.second.location == LOCATION_UNKNOWN || entry.second.location == LOCATION_GRIPPER) continue;
            const auto& box = entry.second.box;
            json << (first ? "" : ", ") << "\"" << entry.first << "\": ["
                 << box.x + base_[0] << ", " << box.y + base_[1] << ", " << box.z + base_[2] << "]";
            first = false;
        }
        json << "}, \"zones\": {";
        first = true;
        for (const auto& zone : zones_) {
            json << (first ? "" : ", ") << "\"" << zone.first << "\": [";
            first = false;
            bool first_occupant = true;
            for (const auto& entry : objects_) {
                const auto& object = entry.second;
                if (object.location == LOCATION_UNKNOWN || object.location == LOCATION_GRIPPER) continue;
                if (overlaps_zone(object.box.x, object.box.y, footprint_radius(object.box), zone.second)) {
                    json << (first_occupant ? "" : ", ") << "\"" << entry.first << "\"";
                    first_occupant = false;
                }
            }
            json << "]";
        }
        json << "}}";
        return json.str();
    }

    void publish_scene_state() {
        ++state_revision_;
        std_msgs::msg::String msg;
        msg.data = scene_state_json();
        state_publisher_->publish(msg);
    }

    // ------------------------------------------------------------------- service

    void handle_skill_request(const std::shared_ptr<ur_task_planner::srv::ExecuteSkill::Request> request,
                              std::shared_ptr<ur_task_planner::srv::ExecuteSkill::Response> response) {

        RCLCPP_INFO(this->get_logger(), "Received skill: %s (object: '%s', zone: '%s')",
                    request->skill.c_str(), request->object_name.c_str(), request->zone.c_str());

        Result result;
        if (!ready_) {
            RCLCPP_ERROR(this->get_logger(), "MoveGroupInterface is not initialized yet. Please wait.");
            result = {"FAILED", "MoveIt is not ready yet"};
        } else {
            try {
                result = execute_skill(*request);
            } catch (const std::exception& e) {
                result = {"FAILED", e.what()};
            }
            publish_scene_state();
        }

        response->status = result.status;
        response->message = result.message;
        if (ready_) response->scene_state = scene_state_json();
        if (result.ok()) {
            RCLCPP_INFO(this->get_logger(), "Skill %s: SUCCESS", request->skill.c_str());
        } else {
            RCLCPP_ERROR(this->get_logger(), "Skill %s: %s (%s)", request->skill.c_str(),
                         result.status.c_str(), result.message.c_str());
        }
    }

    Result execute_skill(const ur_task_planner::srv::ExecuteSkill::Request& request) {
        if (request.skill == "observe") {
            return execute_observe();
        }
        if (use_perception_ && (request.skill == "pick" || request.skill == "place" ||
                                request.skill == "place_on_table" || request.skill == "move_above")) {
            Result observation = execute_observe();
            if (!observation.ok()) return observation;
        }
        if (request.skill == "place_on_table") {
            return execute_place_on_table(request.object_name, request.position);
        }
        if (request.skill == "home") {
            return execute_home();
        } else if (request.skill == "open_gripper") {
            return execute_gripper(gripper_open_position_, "open gripper");
        } else if (request.skill == "close_gripper") {
            return execute_gripper(gripper_closed_position_, "close gripper");
        } else if (request.skill == "move_above") {
            return execute_move_above(request.object_name);
        } else if (request.skill == "move_to_zone") {
            return execute_move_to_zone(request.zone);
        } else if (request.skill == "pick") {
            return execute_pick(request.object_name);
        } else if (request.skill == "place") {
            return execute_place(request.object_name, request.zone);
        }
        return {"INVALID_SKILL", "unknown skill '" + request.skill + "'"};
    }

    // -------------------------------------------------------------------- motion

    // Tool pointing down, fingers closing along the direction given by yaw
    static geometry_msgs::msg::Pose tool_pose(double x, double y, double z, double yaw) {
        geometry_msgs::msg::Pose pose;
        pose.position.x = x;
        pose.position.y = y;
        pose.position.z = z;
        pose.orientation.x = -std::sin(yaw / 2.0);
        pose.orientation.y = std::cos(yaw / 2.0);
        pose.orientation.z = 0.0;
        pose.orientation.w = 0.0;
        return pose;
    }

    // A box looks the same every quarter turn: use the smallest wrist rotation
    static double grasp_yaw(const Box& box) {
        return std::remainder(box.yaw, M_PI / 2.0);
    }

    Result plan_and_execute(MoveGroupInterface& group, const std::string& what) {
        MoveGroupInterface::Plan my_plan;
        if (!(group.plan(my_plan) == moveit::core::MoveItErrorCode::SUCCESS)) {
            return {"PLANNING_FAILED", what + ": no collision free plan within the joint limits"};
        }
        if (!(group.execute(my_plan) == moveit::core::MoveItErrorCode::SUCCESS)) {
            return {"FAILED", what + ": execution failed"};
        }
        return {"SUCCESS", ""};
    }

    Result move_arm(const geometry_msgs::msg::Pose& target_pose, const std::string& what) {
        move_group_->setStartStateToCurrentState();

        // The goal is the IK solution next to the current joint positions: the
        // planner would otherwise pick any solution and often swing the arm around.
        if (move_group_->setJointValueTarget(target_pose, TOOL_LINK)) {
            const Result result = plan_and_execute(*move_group_, what);
            if (result.status != "PLANNING_FAILED") {
                return result;
            }
            RCLCPP_WARN(this->get_logger(), "%s: closest IK solution is not usable, trying the others",
                        what.c_str());
        }

        move_group_->setStartStateToCurrentState();
        move_group_->setPoseTarget(target_pose);
        const Result result = plan_and_execute(*move_group_, what);
        move_group_->clearPoseTargets();
        return result;
    }

    Result move_arm_cartesian(const geometry_msgs::msg::Pose& target_pose, const std::string& what) {
        std::vector<geometry_msgs::msg::Pose> waypoints;
        waypoints.push_back(target_pose);

        move_group_->setStartStateToCurrentState();
        moveit_msgs::msg::RobotTrajectory trajectory;
        const bool avoid_collisions = true;
        const double fraction = move_group_->computeCartesianPath(
            waypoints, EEF_STEP, cartesian_jump_threshold_, trajectory, avoid_collisions);

        if (fraction < 0.999) {
            // Still planned by MoveIt, so collision free, but not along a straight line
            RCLCPP_WARN(this->get_logger(), "%s: only %.0f%% of the straight line is feasible, planning a free motion",
                        what.c_str(), fraction * 100.0);
            return move_arm(target_pose, what);
        }
        scale_trajectory_speed(trajectory, cartesian_speed_scaling_);

        MoveGroupInterface::Plan my_plan;
        my_plan.trajectory_ = trajectory;
        if (!(move_group_->execute(my_plan) == moveit::core::MoveItErrorCode::SUCCESS)) {
            return {"FAILED", what + ": execution failed"};
        }
        return {"SUCCESS", ""};
    }

    // Cartesian paths are timed by move_group at full speed
    static void scale_trajectory_speed(moveit_msgs::msg::RobotTrajectory& trajectory, double scale) {
        if (scale <= 0.0 || scale >= 1.0) {
            return;
        }
        for (auto& point : trajectory.joint_trajectory.points) {
            const double time = rclcpp::Duration(point.time_from_start).seconds();
            point.time_from_start = rclcpp::Duration::from_seconds(time / scale);
            for (double& velocity : point.velocities) {
                velocity *= scale;
            }
            for (double& acceleration : point.accelerations) {
                acceleration *= scale * scale;
            }
        }
    }

    Result move_gripper(double position, const std::string& what) {
        const std::map<std::string, double> target = {{"finger_joint", position},
                                                      {"right_finger_joint", position}};
        gripper_group_->setStartStateToCurrentState();
        if (!gripper_group_->setJointValueTarget(target)) {
            return {"FAILED", what + ": target is outside of the joint limits"};
        }
        return plan_and_execute(*gripper_group_, what);
    }

    // Fingers which reach the grasp position met no resistance: nothing is between them
    bool object_between_fingers() {
        rclcpp::sleep_for(500ms);
        const std::vector<double> positions = gripper_group_->getCurrentJointValues();
        if (positions.empty()) {
            return true;
        }
        double mean = 0.0;
        for (double position : positions) {
            mean += position / positions.size();
        }
        RCLCPP_INFO(this->get_logger(), "Fingers stopped at %.4f (commanded %.4f)", mean, gripper_grasp_position_);
        return mean < gripper_grasp_position_ - 0.001;
    }

    // The object slipped out of the gripper: nobody knows where it is now
    Result object_lost() {
        const std::string lost = held_object_;
        release_attached_object(lost);
        planning_scene_->removeCollisionObjects({lost});
        objects_.at(lost).location = LOCATION_UNKNOWN;
        held_object_.clear();
        return {"OBJECT_LOST", "'" + lost + "' slipped out of the gripper, its position is unknown"};
    }

    // -------------------------------------------------------------------- skills

    static double footprint_radius(const Box& box) {
        return std::hypot(box.size_x, box.size_y) / 2.0;
    }

    static bool overlaps_zone(double x, double y, double radius, const Zone& zone) {
        const double dx = std::max(std::abs(x - zone.x) - zone.size_x / 2.0, 0.0);
        const double dy = std::max(std::abs(y - zone.y) - zone.size_y / 2.0, 0.0);
        return std::hypot(dx, dy) <= radius;
    }

    std::vector<std::string> missing_observed_objects(
            const ur_task_planner::msg::ObservedScene& observation) const {
        std::vector<std::string> missing;
        for (const auto& entry : objects_) {
            if (entry.first != held_object_ &&
                std::find(observation.object_names.begin(), observation.object_names.end(),
                          entry.first) == observation.object_names.end()) {
                missing.push_back(entry.first);
            }
        }
        return missing;
    }

    ur_task_planner::ObservationResult wait_for_complete_observation(
            ur_task_planner::msg::ObservedScene::SharedPtr& observation) {
        const auto started = this->now();
        const auto deadline = std::chrono::steady_clock::now() +
            std::chrono::duration<double>(observation_timeout_);
        std::unique_lock<std::mutex> lock(observation_mutex_);
        const auto fresh = [&]() {
            return latest_observation_ &&
                rclcpp::Time(latest_observation_->header.stamp, started.get_clock_type()) > started;
        };
        // Each attempt starts after any recovery motion; pre-motion images cannot satisfy it.
        if (observation_changed_.wait_until(lock, deadline, [&]() {
                return fresh() && missing_observed_objects(*latest_observation_).empty();
            })) {
            observation = latest_observation_;
            return {"SUCCESS", ""};
        }
        if (!fresh()) {
            return {"OBSERVATION_FAILED", "no fresh RGB-D observation; check camera, TF and /clock"};
        }
        if (latest_observation_->header.frame_id != "gazebo_world" ||
            latest_observation_->object_names.size() != latest_observation_->poses.size()) {
            return {"OBSERVATION_FAILED", "invalid observation frame or pose list"};
        }
        std::ostringstream detail;
        detail << "fresh RGB-D image is missing cubes: ";
        bool first = true;
        for (const auto& name : missing_observed_objects(*latest_observation_)) {
            detail << (first ? "" : ", ") << name;
            first = false;
        }
        detail << "; holding: " << (held_object_.empty() ? "none" : held_object_);
        RCLCPP_WARN(this->get_logger(), "%s", detail.str().c_str());
        return {"OBSERVATION_FAILED", detail.str(), true};
    }

    ur_task_planner::ObservationResult clear_camera_view() {
        // Home changes only arm joints. Keep the gripper closed and its collision object attached.
        for (const auto& entry : objects_) {
            if (entry.first != held_object_ && entry.second.location == LOCATION_UNKNOWN) {
                return {"OBSERVATION_FAILED", "cannot clear camera view: no confirmed pose for " + entry.first};
            }
        }
        RCLCPP_INFO(this->get_logger(), "Moving arm to home to clear camera view (holding: '%s')",
                    held_object_.c_str());
        if (!held_object_.empty() && verify_grasp_ && !object_between_fingers()) {
            const auto lost = object_lost();
            return {lost.status, lost.message};
        }
        const Result movement = execute_home();
        if (!movement.ok()) {
            return {movement.status, "could not clear camera view: " + movement.message};
        }
        if (!held_object_.empty() && verify_grasp_ && !object_between_fingers()) {
            const auto lost = object_lost();
            return {lost.status, lost.message};
        }
        return {"SUCCESS", ""};
    }

    Result execute_observe() {
        if (!use_perception_) {
            // Legacy world: the configured geometry and successful skills own the state.
            return {"SUCCESS", "configured scene (camera observation disabled)"};
        }
        observed_ = false;
        ur_task_planner::msg::ObservedScene::SharedPtr observation;
        const auto result = ur_task_planner::observe_with_recovery(
            [&]() { return wait_for_complete_observation(observation); },
            [&]() { return clear_camera_view(); });
        if (!result.ok()) {
            return {result.status, result.message};
        }
        if (observation->header.frame_id != "gazebo_world" ||
            observation->object_names.size() != observation->poses.size()) {
            return {"OBSERVATION_FAILED", "invalid observation frame or pose list"};
        }
        std::map<std::string, Object> measured = objects_;
        for (auto& entry : measured) {
            if (entry.first != held_object_) entry.second.location = LOCATION_UNKNOWN;
        }
        std::set<std::string> seen;
        for (size_t i = 0; i < observation->object_names.size(); ++i) {
            const auto& name = observation->object_names[i];
            if (!objects_.count(name) || name == held_object_) continue;
            if (!seen.insert(name).second) return {"OBSERVATION_FAILED", "duplicate object observation"};
            const auto& pose = observation->poses[i];
            auto& object = measured.at(name);
            const auto& q = pose.orientation;
            if (!std::isfinite(pose.position.x) || !std::isfinite(pose.position.y) ||
                !std::isfinite(pose.position.z) || !std::isfinite(q.z) || !std::isfinite(q.w) ||
                std::hypot(q.z, q.w) < 0.9) {
                return {"OBSERVATION_FAILED", "invalid measured pose"};
            }
            object.box.x = pose.position.x - base_[0];
            object.box.y = pose.position.y - base_[1];
            object.box.z = pose.position.z - base_[2];
            object.box.yaw = 2.0 * std::atan2(q.z, q.w);
            object.location = LOCATION_TABLE;
            for (const auto& zone : zones_) {
                if (overlaps_zone(object.box.x, object.box.y, footprint_radius(object.box), zone.second)) {
                    object.location = zone.first;
                    break;
                }
            }
        }
        objects_ = measured;
        std::vector<moveit_msgs::msg::CollisionObject> boxes;
        std::vector<moveit_msgs::msg::ObjectColor> colors;
        std::vector<std::string> missing;
        for (const auto& entry : objects_) {
            if (entry.first == held_object_) continue;
            if (entry.second.location == LOCATION_UNKNOWN) missing.push_back(entry.first);
            else {
                boxes.push_back(make_object(entry.first));
                colors.push_back(make_color(entry.first));
            }
        }
        if (!boxes.empty() && !planning_scene_->applyCollisionObjects(boxes, colors)) {
            return {"OBSERVATION_FAILED", "could not update MoveIt scene"};
        }
        if (!missing.empty()) {
            // Keep last known collision geometry, but never infer that a hidden cube's zone is free.
            return {"OBSERVATION_FAILED", "cube not fully visible: " + missing.front() +
                    "; cannot confirm zone occupancy or table clearance"};
        }
        observed_ = true;
        return {"SUCCESS", ""};
    }

    Result execute_place_on_table(const std::string& object_name, const std::vector<double>& position) {
        if (position.size() != 2 || !std::isfinite(position[0]) || !std::isfinite(position[1])) {
            return {"INVALID_POSITION", "position must be finite [x, y] in gazebo_world"};
        }
        if (held_object_.empty() || object_name != held_object_) {
            return {"NOT_HOLDING_OBJECT", "place_on_table must name the held object"};
        }
        const double r = footprint_radius(objects_.at(object_name).box);
        const double x = position[0] - base_[0], y = position[1] - base_[1];
        if (position[0] < table_bounds_[0] + r || position[0] > table_bounds_[1] - r ||
            position[1] < table_bounds_[2] + r || position[1] > table_bounds_[3] - r ||
            std::hypot(x, y) < robot_keepout_ + r || std::hypot(x, y) > placement_reach_ - r) {
            return {"INVALID_POSITION", "temporary footprint outside placement bounds, keepout or reach"};
        }
        for (const auto& zone : zones_) {
            if (overlaps_zone(x, y, r + placement_gap_, zone.second)) {
                return {"INVALID_POSITION", "temporary position overlaps " + zone.first};
            }
        }
        for (const auto& entry : objects_) {
            if (entry.first == held_object_) continue;
            if (entry.second.location == LOCATION_UNKNOWN) {
                return {"INVALID_POSITION", "cannot check clearance of " + entry.first};
            }
            const auto& box = entry.second.box;
            if (std::hypot(x - box.x, y - box.y) < r + footprint_radius(box) + placement_gap_) {
                return {"INVALID_POSITION", "temporary position too close to " + entry.first};
            }
        }
        return place_at(Zone{x, y, table_surface_z_, 0.0, 0.0}, LOCATION_TABLE, "temporary table position");
    }

    Result execute_home() {
        move_group_->setStartStateToCurrentState();
        if (!move_group_->setNamedTarget("home")) {
            return {"FAILED", "named target 'home' is not defined"};
        }
        return plan_and_execute(*move_group_, "home");
    }

    Result execute_gripper(double position, const std::string& what) {
        if (!held_object_.empty()) {
            return {"ALREADY_HOLDING_OBJECT", "the gripper holds '" + held_object_ + "', use place"};
        }
        return move_gripper(position, what);
    }

    Result execute_move_above(const std::string& object_name) {
        const auto it = objects_.find(object_name);
        if (it == objects_.end()) {
            return {"INVALID_OBJECT", "unknown object '" + object_name + "'"};
        }
        if (object_name == held_object_) {
            return {"FAILED", "'" + object_name + "' is in the gripper"};
        }
        if (it->second.location == LOCATION_UNKNOWN) {
            return {"OBJECT_LOST", "the position of '" + object_name + "' is unknown"};
        }
        const Box& box = it->second.box;
        return move_arm(tool_pose(box.x, box.y, box.z + tcp_offset_ + approach_distance_, grasp_yaw(box)),
                        "move above " + object_name);
    }

    Result execute_move_to_zone(const std::string& zone_name) {
        const auto it = zones_.find(zone_name);
        if (it == zones_.end()) {
            return {"INVALID_ZONE", "unknown zone '" + zone_name + "'"};
        }
        const Zone& zone = it->second;
        const double half_height = held_object_.empty() ? 0.025 : objects_.at(held_object_).box.size_z / 2.0;
        const double z = zone.z + half_height + place_clearance_ + tcp_offset_ + place_approach_distance_;
        return move_arm(tool_pose(zone.x, zone.y, z, 0.0), "move to " + zone_name);
    }

    Result execute_pick(const std::string& object_name) {
        const auto it = objects_.find(object_name);
        if (it == objects_.end()) {
            return {"INVALID_OBJECT", "unknown object '" + object_name + "'"};
        }
        if (!held_object_.empty()) {
            return {"ALREADY_HOLDING_OBJECT", "the gripper holds '" + held_object_ + "'"};
        }
        Object& object = it->second;
        if (object.location == LOCATION_UNKNOWN) {
            return {"OBJECT_LOST", "the position of '" + object_name + "' is unknown"};
        }

        // The finger tips go down to the centre of the object
        const geometry_msgs::msg::Pose grasp_pose =
            tool_pose(object.box.x, object.box.y, object.box.z + tcp_offset_, grasp_yaw(object.box));
        geometry_msgs::msg::Pose hover_pose = grasp_pose;
        hover_pose.position.z += approach_distance_;

        Result result = move_gripper(gripper_open_position_, "open gripper");
        if (!result.ok()) return result;

        result = move_arm(hover_pose, "move above " + object_name);
        if (!result.ok()) return result;

        result = move_arm_cartesian(grasp_pose, "move down to " + object_name);
        if (!result.ok()) return result;

        // Attached before closing: MoveIt only lets the gripper touch an attached object
        if (!attach_object(object_name)) {
            return {"FAILED", "could not attach '" + object_name + "' in the planning scene"};
        }

        result = move_gripper(gripper_grasp_position_, "close gripper");
        if (result.ok() && verify_grasp_ && !object_between_fingers()) {
            result = {"GRASP_FAILED", "the gripper closed without holding '" + object_name + "'"};
        }
        if (!result.ok()) {
            // Opened while still attached: the closed fingers overlap the object in the
            // planning scene, nothing could be planned once it is a world object again
            move_gripper(gripper_open_position_, "open gripper");
            detach_object(object_name);
            move_arm_cartesian(hover_pose, "move up");
            return result;
        }

        held_object_ = object_name;
        object.location = LOCATION_GRIPPER;

        result = move_arm_cartesian(hover_pose, "lift " + object_name);
        if (result.ok() && verify_grasp_ && !object_between_fingers()) {
            return object_lost();
        }
        return result;
    }

    Result execute_place(const std::string& object_name, const std::string& zone_name) {
        const auto it = zones_.find(zone_name);
        if (it == zones_.end()) {
            return {"INVALID_ZONE", "unknown zone '" + zone_name + "'"};
        }
        if (!object_name.empty() && objects_.count(object_name) == 0) {
            return {"INVALID_OBJECT", "unknown object '" + object_name + "'"};
        }
        if (held_object_.empty()) {
            return {"NOT_HOLDING_OBJECT", "the gripper is empty, pick an object first"};
        }
        if (!object_name.empty() && object_name != held_object_) {
            return {"NOT_HOLDING_OBJECT", "the gripper holds '" + held_object_ + "', not '" + object_name + "'"};
        }
        const std::string occupant = zone_occupant(zone_name);
        if (!occupant.empty()) {
            return {"ZONE_OCCUPIED", "'" + zone_name + "' already holds '" + occupant + "'"};
        }

        return place_at(it->second, zone_name, zone_name);
    }

    Result place_at(const Zone& zone, const std::string& location, const std::string& label) {
        const std::string held = held_object_;
        Object& object = objects_.at(held);

        // Height of the centre of the object once it rests in the zone
        const double rest_z = zone.z + object.box.size_z / 2.0;
        const geometry_msgs::msg::Pose place_pose =
            tool_pose(zone.x, zone.y, rest_z + place_clearance_ + tcp_offset_, 0.0);
        geometry_msgs::msg::Pose hover_pose = place_pose;
        hover_pose.position.z += place_approach_distance_;

        Result result = move_arm(hover_pose, "move to " + label);
        if (!result.ok()) return result;

        result = move_arm_cartesian(place_pose, "move down to " + label);
        if (!result.ok()) return result;

        if (verify_grasp_ && !object_between_fingers()) {
            result = object_lost();
            move_arm_cartesian(hover_pose, "move up");
            return result;
        }

        result = move_gripper(gripper_open_position_, "open gripper");
        if (!result.ok()) return result;

        object.box.x = zone.x;
        object.box.y = zone.y;
        object.box.z = rest_z;
        object.box.yaw = 0.0;
        object.location = location;
        held_object_.clear();
        if (!detach_object(held)) {
            return {"FAILED", "could not detach '" + held + "' in the planning scene"};
        }

        return move_arm_cartesian(hover_pose, "move up");
    }
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::NodeOptions node_options;
    node_options.automatically_declare_parameters_from_overrides(true);

    std::shared_ptr<SkillExecutorNode> node;
    try {
        node = std::make_shared<SkillExecutorNode>(node_options);
    } catch (const std::exception& e) {
        RCLCPP_FATAL(rclcpp::get_logger("skill_executor_node"), "Invalid configuration: %s", e.what());
        rclcpp::shutdown();
        return 1;
    }

    // We need a multi-threaded executor for MoveIt
    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(node);

    // Initialize move_group after executor is setup (run in a separate thread so node is spinning)
    std::atomic<bool> initialization_failed{false};
    std::thread init_thread([&]() {
        try {
            node->init_move_group();
        } catch (const std::exception& e) {
            initialization_failed = true;
            RCLCPP_FATAL(node->get_logger(), "Initialization failed: %s", e.what());
            rclcpp::shutdown();
        }
    });

    executor.spin();
    init_thread.join();
    if (rclcpp::ok()) {
        rclcpp::shutdown();
    }
    return initialization_failed ? 1 : 0;
}
