# Điều khiển UR3e bằng LLM

Người dùng ra lệnh bằng tiếng Việt hoặc tiếng Anh, ví dụ *"Đưa khối màu đỏ vào vùng B"*.
LLM chuyển câu lệnh thành một kế hoạch JSON, chương trình kiểm tra kế hoạch rồi thực thi
từng skill bằng MoveIt 2 trên tay máy UR3e trong Gazebo.

```
Câu lệnh  →  LLM (Gemini)  →  JSON plan  →  Plan Validator  →  Skill Executor  →  MoveIt 2  →  UR3e
```

LLM không điều khiển robot trực tiếp. Kế hoạch có skill, vật thể hoặc vùng không hợp lệ
bị từ chối và robot đứng yên.

## Các package

| Package | Vai trò |
|---|---|
| `ur_description` | Mô hình URDF của các dòng tay máy UR |
| `ur_onrobot` | UR3e gắn tay kẹp hai ngón, cấu hình controller |
| `ur_simulation_gz` | Thế giới Gazebo (bàn, 3 khối, 3 khay A/B/C) và launch mô phỏng |
| `ur_task_planner` | Robot skill, LLM planner, plan validator |
| `ur_trajectory_drawer` | Vẽ hình tròn và chữ S, xem README ở nhánh `main` |

## Các node chính

| Node | Nhiệm vụ | Giao tiếp |
|---|---|---|
| `llm_planner_node` | Gọi LLM, kiểm tra plan, gọi lần lượt từng skill | Nhận `/user_command`, `/scene_state`; phát `/task_status` |
| `skill_executor_node` | Thực thi skill qua MoveIt 2, quản lý vật cản, ghi nhớ vị trí vật | Service `/execute_skill`; phát `/scene_state` |
| `move_group` | Lập quỹ đạo, kiểm tra va chạm và giới hạn khớp | Action tới các controller |

Các file quan trọng trong `ur_task_planner`:

| File | Nội dung |
|---|---|
| `src/skill_executor_node.cpp` | Các skill |
| `scripts/llm_planner_node.py` | Nhận lệnh, gọi LLM, điều phối |
| `scripts/plan_validator.py` | Luật kiểm tra plan |
| `config/scene.yaml` | Vật thể, vùng, vật cản; phải khớp với `ur_simulation_gz/world/ur_table.sdf` |
| `config/joint_limits.yaml` | Giới hạn khớp dùng cho MoveIt |

## Các skill

| Skill | Tham số |
|---|---|
| `home` | không |
| `pick` | `object` |
| `place` | `object`, `zone` |
| `move_above` | `object` |
| `move_to_zone` | `zone` |
| `open_gripper`, `close_gripper` | không |

Vật thể: `red_cube`, `blue_cube`, `yellow_cube`. Vùng: `zone_a`, `zone_b`, `zone_c` và
vùng tạm `zone_temp` trên mặt bàn. Mỗi vùng chứa một vật.

Trạng thái trả về: `SUCCESS`, `FAILED`, `PLANNING_FAILED`, `GRASP_FAILED`, `OBJECT_LOST`,
`INVALID_SKILL`, `INVALID_OBJECT`, `INVALID_ZONE`, `NOT_HOLDING_OBJECT`,
`ALREADY_HOLDING_OBJECT`, `ZONE_OCCUPIED`.

## Yêu cầu

- Ubuntu 22.04, ROS 2 Humble, Gazebo (Ignition) Fortress
- API Key của Gemini trong biến môi trường `GEMINI_API_KEY`

```bash
sudo apt install ros-humble-moveit ros-humble-ur ros-humble-ros-gz \
  ros-humble-ign-ros2-control ros-humble-ros2-controllers
```

## Build

Đặt repo này vào thư mục `src` của workspace:

```bash
mkdir -p ~/workspaces/ur_gz
cd ~/workspaces/ur_gz
git clone -b assignment_2 https://github.com/TNSang25/UniversalRobotDemo.git src
source /opt/ros/humble/setup.bash
colcon build
```

## Thứ tự chạy

**Terminal 1: mô phỏng**

```bash
cd ~/workspaces/ur_gz
source install/setup.bash
ros2 launch ur_simulation_gz ur3e_rg2_table.launch.py launch_rviz:=false
```

Đợi dòng `Successfully started gripper_trajectory_controller`.

**Terminal 2: MoveIt, RViz, skill executor, LLM planner**

```bash
cd ~/workspaces/ur_gz
source install/setup.bash
export GEMINI_API_KEY=<key>
ros2 launch ur_task_planner task_planner.launch.py
```

Đợi dòng `Skill Executor Node ready.`

**Terminal 3: theo dõi kết quả**

```bash
source ~/workspaces/ur_gz/install/setup.bash
ros2 topic echo /task_status --full-length
```

**Terminal 4: gửi lệnh**, từng lệnh một, đợi lệnh trước xong:

```bash
source ~/workspaces/ur_gz/install/setup.bash
ros2 topic pub --once /user_command std_msgs/msg/String "{data: 'Nội dung lệnh'}"
```

Dừng bằng Ctrl+C ở terminal 2 rồi terminal 1. Khởi động lại để đưa các khối về chỗ cũ.

## Lệnh phối hợp nhiều vật thể

Ghi mã sinh viên và cách sắp xếp tương ứng vào `user_context` trong
`ur_task_planner/config/scene.yaml`, build lại, rồi gửi:

```bash
ros2 topic pub --once /user_command std_msgs/msg/String \
  "{data: 'Arrange all objects according to my student ID.'}"
```

LLM tự sinh kế hoạch nhiều bước. Khi vùng đích đang có vật khác, kế hoạch đưa vật đó sang
`zone_temp` trước. Plan bị validator từ chối được gửi lại cho LLM kèm lý do để sửa một lần.

## Tuỳ chọn

Gọi thẳng một skill, không qua LLM:

```bash
ros2 service call /execute_skill ur_task_planner/srv/ExecuteSkill \
  "{skill: 'pick', object_name: 'red_cube', zone: ''}"
```

Chọn model. Mặc định là `$GEMINI_MODEL`, nếu không đặt thì `gemini-flash-latest`; khi model
quá tải hoặc hết hạn mức, node tự chuyển sang `gemini-flash-lite-latest`:

```bash
ros2 launch ur_task_planner task_planner.launch.py llm_model:=gemini-flash-lite-latest
```

Chạy unit test của validator:

```bash
colcon test --packages-select ur_task_planner
colcon test-result --verbose
```
