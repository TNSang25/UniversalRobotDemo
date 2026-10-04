# Điều khiển UR3e bằng LLM

Ra lệnh bằng tiếng Việt hoặc tiếng Anh để robot UR3e gắp và đặt các khối trong Gazebo.
Ví dụ: **"Đưa khối màu đỏ vào vùng B."**

## Các package

| Package | Vai trò |
|---|---|
| `ur_description` | Mô hình URDF của các dòng tay máy UR |
| `ur_onrobot` | UR3e gắn tay kẹp hai ngón, cấu hình controller |
| `ur_simulation_gz` | Thế giới Gazebo (bàn, 3 khối, 3 khay A/B/C) và launch mô phỏng |
| `ur_task_planner` | Robot skill, LLM planner, plan validator |
| `ur_trajectory_drawer` | Vẽ hình tròn và chữ S, xem README ở nhánh `main` |

## 1. Cài phần mềm cần thiết

Máy cần **Ubuntu 22.04**, **ROS 2 Humble** và **Gemini API key**.
Nếu chưa có ROS, cài theo [hướng dẫn ROS 2 Humble](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debs.html).

Sau khi cài ROS, mở terminal và chạy:

```bash
sudo apt update
sudo apt install -y build-essential cmake git \
  python3-colcon-common-extensions python3-rosdep python3-pytest python3-yaml \
  ros-humble-moveit ros-humble-ur-moveit-config ros-humble-ur-controllers \
  ros-humble-ros-gz ros-humble-ign-ros2-control ros-humble-ros2-controllers

sudo rosdep init
rosdep update
```

Nếu `rosdep init` báo đã tồn tại, bỏ qua lệnh đó. Chạy `rosdep update` như bình thường.

## 2. Clone repo

Chỉ làm bước này nếu chưa có repo trong workspace:

```bash
mkdir -p ~/workspaces/ur_gz/src
cd ~/workspaces/ur_gz
git clone -b assignment_3 https://github.com/TNSang25/UniversalRobotDemo.git src
```

## 3. Build

Mở terminal mới và chạy:

```bash
cd ~/workspaces/ur_gz
source /opt/ros/humble/setup.bash
export PYTHONNOUSERSITE=1

rosdep install --from-paths src --ignore-src --rosdistro humble -y
colcon build --allow-overriding ur_description --cmake-force-configure
source install/setup.bash
ros2 run ur_task_planner check_install.py
```

Thấy `Installation OK` là có thể chuyển sang bước chạy.
`PYTHONNOUSERSITE=1` giúp tránh lỗi xung đột Python/pytest.

## 4. Chạy chương trình

**Terminal 1 — mở mô phỏng:**

```bash
cd ~/workspaces/ur_gz
export PYTHONNOUSERSITE=1
source install/setup.bash
ros2 launch ur_simulation_gz ur3e_rg2_table.launch.py launch_rviz:=false
```

Đợi ba controller báo `Configured and activated`, rồi mở terminal 2.

**Terminal 2 — mở bộ điều khiển:**

Thay `YOUR_API_KEY` bằng Gemini API key của bạn:

```bash
cd ~/workspaces/ur_gz
export PYTHONNOUSERSITE=1
source install/setup.bash
export GEMINI_API_KEY=''
ros2 launch ur_task_planner task_planner.launch.py
```

Đợi dòng `Skill Executor Node ready.`

**Terminal 3 — gửi lệnh:**

```bash
export PYTHONNOUSERSITE=1
source ~/workspaces/ur_gz/install/setup.bash
ros2 topic pub --once /user_command std_msgs/msg/String \
  "{data: 'Đưa khối màu đỏ vào vùng B.'}"
```

Có thể đổi câu lệnh. Robot có khối **đỏ, xanh dương, vàng** và vùng **A, B, C**.
Gửi từng lệnh một và đợi robot làm xong trước khi gửi lệnh tiếp theo.

Muốn xem kết quả, mở thêm terminal và chạy:

```bash
export PYTHONNOUSERSITE=1
source ~/workspaces/ur_gz/install/setup.bash
ros2 topic echo /task_status --full-length
```

Dừng chương trình bằng **Ctrl+C ở terminal 2, rồi terminal 1**.

## World với 5 cube và camera RGB-D

World mới `ur_simulation_gz/world/ur_table_depth.sdf` được tạo từ `ur_table.sdf`,
giữ bàn, 3 cube cũ; dịch 3 zone A/B/C cùng chữ tương ứng ra xa robot 5 cm
(+X, tâm zone ở X=0.10 m); thêm cube **xanh lá**, **hồng** và một camera
RGB-D cố định ở `(0, 0, 1.6)` m, nhìn xuống mặt bàn cao 0.8 m. Các cube đều có
cạnh 5 cm. Launch bên dưới spawn một UR3e kèm gripper RG2 và bật bridge camera:

```bash
cd ~/workspaces/ur_gz
source /opt/ros/humble/setup.bash
export PYTHONNOUSERSITE=1
colcon build --packages-select ur_simulation_gz
source install/setup.bash
ros2 launch ur_simulation_gz ur3e_rg2_table_depth.launch.py
```

Camera phát dữ liệu 640×480 ở 15 Hz theo thời gian mô phỏng:

| ROS topic | Dữ liệu |
|---|---|
| `/depth_camera/image` | Ảnh màu RGB |
| `/depth_camera/depth_image` | Ảnh depth `32FC1`, đơn vị mét |
| `/depth_camera/camera_info` | Thông số nội tại camera |
| `/depth_camera/points` | Point cloud màu `PointCloud2` |

Mỗi lần chạy launch này, 5 cube được đặt ngẫu nhiên cả X/Y và góc yaw trong vùng
giữa robot và các zone: **X ∈ [-0.25, 0.015], Y ∈ [-0.34, 0.34] m**, theo gốc
`gazebo_world`. Toàn bộ khối (kể cả khi xoay) nằm trong vùng, tránh khu vực bán
kính 12 cm quanh chân robot; khoảng hở giữa các cube ít nhất 1.5 cm. Cube nằm
trên mặt bàn ở Z=0.825 m. Camera và robot giữ nguyên vị trí.

Launch tạo một file SDF tạm riêng cho mỗi lần chạy và xóa khi dừng; không thay
đổi SDF nguồn, `scene.yaml` hay truyền tọa độ spawn cho skill. Đổi giới hạn bằng
`cube_x_min`, `cube_x_max`, `cube_y_min`, `cube_y_max` (đơn vị mét). Muốn lặp lại
một bố cục để debug, thêm `random_seed:=42`; mặc định seed để trống để sinh bố
cục mới. Ví dụ:

```bash
ros2 launch ur_simulation_gz ur3e_rg2_table_depth.launch.py random_seed:=42
```

Kiểm tra dữ liệu: `ros2 topic echo /depth_camera/camera_info --once`.
TF gồm `gazebo_world → depth_camera_link → depth_camera_optical_frame` và
`gazebo_world → world` (gốc URDF tại vị trí spawn robot `(-0.35, 0, 0.81)` m).
Ảnh và CameraInfo dùng `depth_camera_optical_frame`: +Z hướng nhìn, +X sang phải,
+Y xuống dưới. Point cloud của Gazebo Fortress dùng `depth_camera_link`: +X hướng
nhìn, +Y sang trái, +Z lên trên; bridge riêng sửa frame trong header của point
cloud và TF cho phép đổi giữa hai frame. Khi đổi pose
camera trong SDF, cần cập nhật TF tương ứng trong launch mới.

## Quan sát zone và tự dọn cube chiếm chỗ

Planner hỗ trợ world RGB-D với 5 cube. Node `scene_observer_node.py` ghép ảnh RGB
và depth theo timestamp, nhận diện màu, đo tâm/yaw cube và chuyển qua TF về
`gazebo_world`. Skill `observe` cập nhật collision scene của MoveIt và trạng thái
zone dựa trên footprint cube. Tọa độ spawn ngẫu nhiên không được truyền vào planner.

Build lại vì `ExecuteSkill` đã thêm trường `position` và snapshot `scene_state`:

```bash
cd ~/workspaces/ur_gz
source /opt/ros/humble/setup.bash
export PYTHONNOUSERSITE=1
rosdep install --from-paths src --ignore-src --rosdistro humble -y
colcon build --packages-select ur_task_planner ur_simulation_gz
source install/setup.bash
```

Terminal 1 chạy mô phỏng camera như phần trên. Terminal 2 chạy planner:

```bash
source ~/workspaces/ur_gz/install/setup.bash
export GEMINI_API_KEY='YOUR_API_KEY'
ros2 launch ur_task_planner task_planner.launch.py \
  scene_file:=scene_depth.yaml launch_observer:=true
```

Ví dụ gửi lần lượt hai lệnh, đợi `/task_status` báo hoàn tất từng lệnh:

```bash
ros2 topic pub --once /user_command std_msgs/msg/String \
  "{data: 'Đưa cube đỏ vào zone B.'}"
ros2 topic pub --once /user_command std_msgs/msg/String \
  "{data: 'Đưa cube xanh dương vào zone B.'}"
```

Ở lệnh thứ hai, LLM nhận trạng thái zone B có cube đỏ và phải lập kế hoạch:
`pick(red_cube)` → `place_on_table(red_cube, [x, y])` → `pick(blue_cube)` →
`place(blue_cube, zone_b)` → `home`. **LLM tự chọn `[x, y]`**, đơn vị mét trong
`gazebo_world`; không có vị trí tạm được chọn sẵn. Cube đỏ ở lại chỗ tạm để nhường
zone B. Luồng này vẫn dùng được khi cả A/B/C đang có cube. Nếu cube được yêu cầu
đã ở zone đích, LLM chỉ cần trả robot về home.

Validator mô phỏng toàn bộ kế hoạch, kiểm tra footprint trong
`table_placement_bounds`, khoảng hở `placement_gap`, vùng tránh chân robot,
giới hạn tầm với và cập nhật vị trí sau từng bước. Nếu kế hoạch sai, planner gửi
lỗi lại cho LLM để sửa tối đa hai lần trước khi thực thi. Executor kiểm tra lại
ảnh mới trước mỗi thao tác gắp/đặt; `place_on_table` cũng kiểm tra khoảng hở tại
thời điểm thực thi. MoveIt kiểm tra đường đi và khả năng đến pose cụ thể.
Sau khi home, planner quan sát lần cuối để xác nhận các đích đặt trước khi báo
`SUCCESS`. Xem trạng thái bằng `ros2 topic echo /scene_state --full-length` và
`ros2 topic echo /task_status --full-length`.

Nhận diện hiện dành cho 5 cube màu đã biết, cạnh 5 cm, đặt riêng trên bàn hoặc
khay, với camera RGB/depth đã đăng ký cùng frame. Zone lấy kích thước/tọa độ khay
đã hiệu chuẩn trong `scene_depth.yaml`. Cube bị che khuất, depth/TF thiếu hoặc
không đủ quan sát sẽ trả `OBSERVATION_FAILED` và dừng kế hoạch; không suy ra zone
trống từ việc thiếu cube. Chưa hỗ trợ vật lạ, cube xếp chồng hay tự chuyển camera
để tìm cube bị che. Nếu thay hình học bàn/camera/zone, cần cập nhật cấu hình/TF.

World cũ vẫn chạy bằng `task_planner.launch.py` mặc định với `scene.yaml`: hỗ trợ
đặt tạm và dọn zone theo trạng thái skill, nhưng không có đo camera.
