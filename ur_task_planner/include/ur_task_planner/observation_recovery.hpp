#ifndef UR_TASK_PLANNER__OBSERVATION_RECOVERY_HPP_
#define UR_TASK_PLANNER__OBSERVATION_RECOVERY_HPP_

#include <string>

namespace ur_task_planner {

struct ObservationResult {
    std::string status;
    std::string message;
    // Only fresh, valid images with missing cube detections warrant moving the arm.
    bool incomplete{false};
    bool ok() const { return status == "SUCCESS"; }
};

// A camera blocked by the arm may never see a full scene from the current pose.
// Move to a clear view once, then require a new complete image after that motion.
// Camera/TF outages and invalid images must not trigger speculative arm motion.
template <typename WaitForImage, typename ClearView>
ObservationResult observe_with_recovery(WaitForImage wait_for_image, ClearView clear_view) {
    const auto initial = wait_for_image();
    if (initial.status != "OBSERVATION_FAILED" || !initial.incomplete) {
        return initial;
    }
    const auto movement = clear_view();
    if (!movement.ok()) {
        return movement;
    }
    return wait_for_image();
}

}  // namespace ur_task_planner

#endif  // UR_TASK_PLANNER__OBSERVATION_RECOVERY_HPP_
