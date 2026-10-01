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

Vật thể: `red_cube`, `blue_cube`, `yellow_cube`. Vùng: `zone_a`, `zone_b`, `zone_c`;
mỗi vùng chứa một vật. Cấu hình hiện tại chưa có `zone_temp`.

Trạng thái trả về: `SUCCESS`, `FAILED`, `PLANNING_FAILED`, `GRASP_FAILED`, `OBJECT_LOST`,
`INVALID_SKILL`, `INVALID_OBJECT`, `INVALID_ZONE`, `NOT_HOLDING_OBJECT`,
`ALREADY_HOLDING_OBJECT`, `ZONE_OCCUPIED`.

## Yêu cầu trên máy mới

Luồng chạy này dùng **Ubuntu 22.04 + ROS 2 Humble + Gazebo Fortress (Ignition 6)**.
Đây là tổ hợp được hỗ trợ cho nhánh này; các lệnh dưới đây không áp dụng trực tiếp cho
Windows/macOS, ROS Jazzy hoặc Gazebo Classic/Harmonic. Máy ảo cần chạy Ubuntu 22.04
và có hỗ trợ đồ hoạ OpenGL nếu mở Gazebo/RViz. Có thể tắt giao diện để chạy headless.

- Trình biên dịch C/C++, CMake, Git, `colcon`, `rosdep`, Python 3 và PyYAML.
- MoveIt 2, cấu hình UR MoveIt, `ros_gz`, `ign_ros2_control`, các ROS 2 controller.
- Internet và API Key của Gemini trong biến môi trường `GEMINI_API_KEY`, có quyền
  sử dụng model, nếu dùng lệnh ngôn ngữ tự nhiên.
  Planner gọi REST bằng thư viện chuẩn Python; không cần cài Gemini SDK bằng pip.
- Các package trong repo phải được clone đầy đủ và build cùng nhau. Không cần tải
  thêm các repo tên `ur_onrobot_description` hay `ur_onrobot_moveit_config`.

### Cài ROS và dependency

Thiết lập repository apt của ROS 2 theo
[hướng dẫn cài ROS 2 Humble trên Ubuntu](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debs.html)
trước khi chạy các lệnh apt dưới đây. Nếu đã cài Humble từ apt thì bỏ qua bước thiết lập
repository. Gazebo Fortress được cài cùng `ros-humble-ros-gz` theo
[tổ hợp ROS/Gazebo chính thức](https://gazebosim.org/docs/fortress/ros_installation/).

```bash
sudo apt update
sudo apt install -y build-essential cmake git \
  python3-colcon-common-extensions python3-rosdep python3-pytest python3-yaml \
  ros-humble-desktop ros-humble-moveit \
  ros-humble-ur-moveit-config ros-humble-ur-controllers \
  ros-humble-ros-gz ros-humble-ign-ros2-control ros-humble-ros2-controllers
```

Khởi tạo `rosdep` **một lần trên mỗi máy**, rồi cập nhật database:

```bash
sudo rosdep init
rosdep update
```

Nếu `rosdep init` báo `default sources list file already exists`, máy đã được khởi tạo;
chỉ cần chạy `rosdep update` bằng tài khoản người dùng.

## Clone và build

Dùng terminal mới, source Humble; tránh source workspace cũ trước khi build.
Không dùng virtualenv/Conda của dự án khác; ROS Humble từ apt dùng Python 3.10 hệ thống. Ví dụ bên
dưới đặt repo trong `src/UniversalRobotDemo`. Repo cũng có thể nằm trực tiếp ở `src`
hoặc ở đường dẫn workspace khác; code không phụ thuộc tên tài khoản hay vị trí clone.

```bash
mkdir -p ~/workspaces/ur_gz/src
cd ~/workspaces/ur_gz
git clone --branch assignment_2 https://github.com/TNSang25/UniversalRobotDemo.git src/UniversalRobotDemo
source /opt/ros/humble/setup.bash

# Cài đầy đủ dependency đã khai báo trong package.xml, không bỏ qua lỗi rosdep.
rosdep install --from-paths src --ignore-src --rosdistro humble -y

# Build tuần tự và giới hạn 2 tác vụ compiler để giảm mức sử dụng RAM.
MAKEFLAGS=-j2 CMAKE_BUILD_PARALLEL_LEVEL=2 colcon build \
  --symlink-install --executor sequential --allow-overriding ur_description \
  --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash

# Kiểm tra launch imports, URDF/meshes, SRDF, controller, YAML và service đã cài.
ros2 run ur_task_planner check_install.py
```

`--allow-overriding ur_description` cho phép dùng mô hình UR trong repo thay cho bản
được apt cài gián tiếp cùng UR MoveIt. Khi bước kiểm tra thành công, sẽ có dòng
`Installation OK`. Kiểm tra này không mở Gazebo và không gọi Gemini.

Nếu đã từng build repo ở vị trí khác, đổi ROS distro hoặc copy cả workspace từ máy cũ,
hãy dùng workspace mới để build lại; không dùng lại `build/`, `install/`, `log/` cũ.
Không clone thêm một bản repo vào workspace đã chứa cùng các package vì `colcon` sẽ
báo trùng tên package. Nếu đã có clone thì dùng `git pull --ff-only` trên `assignment_2`.

## Thứ tự chạy

**Terminal 1: mô phỏng**

```bash
cd ~/workspaces/ur_gz
source install/setup.bash
ros2 launch ur_simulation_gz ur3e_rg2_table.launch.py launch_rviz:=false
```

Đợi cả ba controller `joint_state_broadcaster`, `joint_trajectory_controller` và
`gripper_trajectory_controller` báo `Configured and activated`. Chúng được khởi động
sau khi robot được tạo trong Gazebo, với thời gian chờ mặc định 120 giây.

Nếu không có giao diện đồ hoạ:

```bash
ros2 launch ur_simulation_gz ur3e_rg2_table.launch.py launch_rviz:=false gazebo_gui:=false
```

Máy khởi động chậm có thể thêm `controller_spawner_timeout:=240`.

**Terminal 2: MoveIt, RViz, skill executor, LLM planner**

```bash
cd ~/workspaces/ur_gz
source install/setup.bash
export GEMINI_API_KEY='DAN_KHOA_API_CUA_BAN_VAO_DAY'
ros2 launch ur_task_planner task_planner.launch.py
```

Đợi dòng `Skill Executor Node ready.`. Node đợi MoveIt và joint state thực tế, không
phụ thuộc một khoảng sleep cố định. Có thể tăng `startup_timeout_sec:=240.0` nếu máy
khởi động chậm. Thêm `launch_rviz:=false` khi chạy headless.

Để kiểm tra MoveIt và skill executor khi chưa có Gemini API key:

```bash
ros2 launch ur_task_planner task_planner.launch.py launch_llm:=false launch_rviz:=false
```

Chế độ này chỉ nhận service `/execute_skill`, chưa xử lý `/user_command`.

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

Có thể gửi lệnh chỉ rõ vật thể và vùng đích, ví dụ:

```bash
ros2 topic pub --once /user_command std_msgs/msg/String \
  "{data: 'Đưa khối đỏ vào vùng B, rồi đưa khối xanh dương vào vùng C.'}"
```

Mỗi vùng chứa một vật; validator kiểm tra cả chuỗi trước khi thực thi. Cấu hình hiện
chỉ có A/B/C. `user_context`, vùng tạm `zone_temp` và việc tự sửa plan bị từ chối chưa
được triển khai, nên không dùng lệnh sắp xếp theo mã sinh viên trong phiên bản này.
Plan không hợp lệ được báo `REJECTED` trên `/task_status`.

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

Chạy test validator và kiểm tra cài đặt:

```bash
colcon test --packages-select ur_task_planner \
  --ctest-args -R 'test_plan_validator|test_installation'
colcon test-result --verbose
```

## Chẩn đoán khi chạy trên máy khác

| Lỗi / triệu chứng | Kiểm tra và cách xử lý |
|---|---|
| `Unable to locate package ros-humble-...` | Kiểm tra Ubuntu 22.04 và repository apt của ROS Humble; chạy `sudo apt update`. |
| `Cannot locate rosdep definition` | Dùng nhánh `assignment_2` mới nhất, `rosdep update`, rồi chạy lại lệnh cài dependency. |
| `Package ... not found`, `ModuleNotFoundError` | Source `/opt/ros/humble/setup.bash` và `install/setup.bash` của đúng workspace trong mỗi terminal; chạy `check_install.py`. |
| `file INSTALL cannot find .../ur_onrobot/launch` | Đây là lỗi của phiên bản cũ; pull bản sửa và build lại trong workspace sạch. |
| Pytest lỗi do plugin trong `~/.local` | Thử `PYTHONNOUSERSITE=1 colcon test ...` để dùng các module Python hệ thống thay cho bản cài pip của người dùng. |
| `cc1plus` bị `Killed` | Thiếu RAM khi biên dịch; giảm `MAKEFLAGS=-j1`, `CMAKE_BUILD_PARALLEL_LEVEL=1`, giữ `--executor sequential`. |
| Không có `/controller_manager`, `/clock` hoặc `/joint_states` | Kiểm tra lỗi Gazebo/plugin ở terminal 1, đủ `ign_ros2_control` và đúng Fortress; chưa chạy terminal 2. |
| `No current robot state` | Kiểm tra controller đang active, `/clock` có dữ liệu, mọi terminal có cùng `ROS_DOMAIN_ID`; đợi mô phỏng trước khi chạy planner. |
| Gazebo/RViz lỗi OpenGL | Kiểm tra driver và đồ hoạ máy ảo; dùng `gazebo_gui:=false` và `launch_rviz:=false` để kiểm tra headless. |
| `LLM_ERROR`, HTTP 403/404/429 | Kiểm tra Gemini key, quyền truy cập model, hạn mức và Internet; đổi `llm_model` nếu cần. Lỗi API không phải lỗi build. |

Sau khi source workspace, kiểm tra ROS ở terminal riêng:

```bash
ros2 control list_controllers -c /controller_manager
ros2 topic echo /clock --once
ros2 topic echo /joint_states --once
ros2 service list | grep execute_skill
```

Ba controller nói trên phải ở trạng thái `active`. Không cần một tiến trình
`ros2_control_node` riêng: plugin `ign_ros2_control` trong Gazebo tạo controller manager.

### Phạm vi kiểm thử và lỗi còn lại

Bản sửa được kiểm tra trên Ubuntu 22.04 với source sạch ở hai cấu trúc thư mục
workspace khác nhau, gồm build thường và `--symlink-install`, chỉ dùng ROS Humble
hệ thống làm underlay. Các kiểm tra đã đạt:

- Build cả 5 package và `rosdep check`.
- Xacro/URDF, assets đã cài, service types và validator; `colcon test-result` báo
  107 tests, 0 errors, 0 failures (bao gồm số đếm của CTest và pytest).
- Gazebo headless, ba controller active, `/clock`, `/joint_states`, MoveIt và
  LLM planner khởi động; các skill `home`, `open_gripper`, `close_gripper` thành công.

Chưa kiểm thử gọi Gemini API thực tế, toàn bộ chuỗi gắp/đặt hoặc GPU/giao diện trên
máy thứ hai. Model, API key và hạn mức cần được kiểm tra riêng trên máy chạy.

Trong kiểm thử với MoveIt **2.5.10** và rclcpp **16.0.21**, `move_group` có thể báo
segfault **khi dừng bằng Ctrl+C**, với trace trong destructor của
`TrajectoryExecutionManager`/rclcpp. Lỗi lúc tắt này chưa được sửa trong repo; build,
khởi động và các skill nói trên vẫn đạt. Không dùng kết quả smoke test để kết luận
mọi phiên bản thư viện đều dừng sạch.

Log `No 3D sensor plugin(s) defined for octomap updates` cũng xuất hiện vì demo không
cấu hình camera 3D; trong phiên kiểm thử này, nó không ngăn planning/execution với
các vật cản được skill executor nạp từ `scene.yaml`.
