#include <gtest/gtest.h>
#include <ur_task_planner/observation_recovery.hpp>

#include <string>
#include <vector>

using ur_task_planner::ObservationResult;
using ur_task_planner::observe_with_recovery;

TEST(ObservationRecovery, FullSceneNeedsNoArmMotion) {
    int movements = 0;
    const auto result = observe_with_recovery(
        []() { return ObservationResult{"SUCCESS", ""}; },
        [&]() { ++movements; return ObservationResult{"SUCCESS", ""}; });
    EXPECT_TRUE(result.ok());
    EXPECT_EQ(movements, 0);
}

TEST(ObservationRecovery, PlaceAfterPickRecapturesAfterClearingView) {
    // A successful pick leaves the arm over the scene, hiding blue_cube.
    std::string held = "red_cube";
    std::vector<std::string> events;
    bool camera_clear = false;
    const auto result = observe_with_recovery(
        [&]() {
            events.push_back("capture");
            if (!camera_clear) {
                return ObservationResult{"OBSERVATION_FAILED", "missing blue_cube; holding red_cube", true};
            }
            EXPECT_EQ(held, "red_cube");
            return ObservationResult{"SUCCESS", ""};
        },
        [&]() {
            events.push_back("home");
            camera_clear = true;
            return ObservationResult{"SUCCESS", ""};
        });
    EXPECT_TRUE(result.ok());
    EXPECT_EQ(events, (std::vector<std::string>{"capture", "home", "capture"}));
    EXPECT_EQ(held, "red_cube");
}

TEST(ObservationRecovery, PersistentOcclusionStopsAfterOneRecovery) {
    int captures = 0, movements = 0;
    const auto result = observe_with_recovery(
        [&]() { ++captures; return ObservationResult{"OBSERVATION_FAILED", "missing blue_cube", true}; },
        [&]() { ++movements; return ObservationResult{"SUCCESS", ""}; });
    EXPECT_EQ(result.status, "OBSERVATION_FAILED");
    EXPECT_EQ(result.message, "missing blue_cube");
    EXPECT_EQ(captures, 2);
    EXPECT_EQ(movements, 1);
}

TEST(ObservationRecovery, CameraOutageOrInvalidFramesNeverMovesArm) {
    for (const std::string message : {"no fresh image", "invalid TF frame", "invalid pose list"}) {
        int movements = 0;
        const auto result = observe_with_recovery(
            [&]() { return ObservationResult{"OBSERVATION_FAILED", message}; },
            [&]() { ++movements; return ObservationResult{"SUCCESS", ""}; });
        EXPECT_EQ(result.message, message);
        EXPECT_EQ(movements, 0);
    }
}

TEST(ObservationRecovery, MotionFailureNeverContinuesToPlacement) {
    int captures = 0;
    const auto result = observe_with_recovery(
        [&]() { ++captures; return ObservationResult{"OBSERVATION_FAILED", "missing cube", true}; },
        []() { return ObservationResult{"PLANNING_FAILED", "no clear path to home"}; });
    EXPECT_EQ(result.status, "PLANNING_FAILED");
    EXPECT_EQ(captures, 1);
}

TEST(ObservationRecovery, LostGraspAbortsRecovery) {
    int captures = 0;
    const auto result = observe_with_recovery(
        [&]() { ++captures; return ObservationResult{"OBSERVATION_FAILED", "missing cube", true}; },
        []() { return ObservationResult{"OBJECT_LOST", "red_cube slipped"}; });
    EXPECT_EQ(result.status, "OBJECT_LOST");
    EXPECT_EQ(captures, 1);
}

TEST(ObservationRecovery, CameraOutageAfterMotionStillFails) {
    bool moved = false;
    const auto result = observe_with_recovery(
        [&]() {
            return moved ? ObservationResult{"OBSERVATION_FAILED", "no image after home"}
                         : ObservationResult{"OBSERVATION_FAILED", "cube occluded", true};
        },
        [&]() { moved = true; return ObservationResult{"SUCCESS", ""}; });
    EXPECT_EQ(result.status, "OBSERVATION_FAILED");
    EXPECT_EQ(result.message, "no image after home");
}
