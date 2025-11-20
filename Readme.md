# GIẢI NÉN FILE VÀ CHUYỂN FOLDER AGV VÀO HOME

# TRƯỚC KHI BUILD COPY TOÀN BỘ FOLDER Ở ~/AGV/src/ros_motion_planning/src/sim_env/models/RIO_model VÀO ~/.gazebo/models/

# BUILD VÀ CHẠY THUẬT TOÁN:
cd ~/AGV
pip install conan==1.59.0
conan remote add conancenter https://center.conan.io
# (NẾU KHÔNG ĐƯỢC THÌ cd ~/AGV/src/ros_motion_planning/3rd RỒI CHẠY LẠI DÒNG 2)

sudo apt install python-is-python3 \
ros-noetic-amcl \
ros-noetic-base-local-planner \
ros-noetic-map-server \
ros-noetic-move-base \
ros-noetic-navfn \
libgoogle-glog-dev

# CHUYỂN ĐỔI THUẬT TOÁN Ở FILE ~/AGV/src/ros_motion_planning/src/user_config/user_config.yaml

cd ~/AGV/src/ros_motion_planning/scripts
./build.sh
./main.sh

# ĐỂ XUẤT LOG MỞ TERMINAL MỚI SAU KHI CHẠY ./main.sh

# --- ASTAR ---
cd ~/AGV/src/ros_motion_planning/src/sim_env/script
chmod +x path_logger_astar.py
rosrun sim_env path_logger_astar.py \
  _plan_topic:=/move_base/PathPlanner/plan \
  _csv_path:=/home/$USER/ros_logs/DUONG_DI_ASTAR.csv \
  _write_period:=0.5 _plan_stride:=1 _plan_dedupe:=true

# --- DSTAR ---
cd ~/AGV/src/ros_motion_planning/src/sim_env/script
chmod +x path_logger_dstar.py
rosrun sim_env path_logger_dstar.py \
  _plan_topic:=/move_base/DstarPlanner/plan \
  _csv_path:=/home/$USER/ros_logs/DUONG_DI_DSTAR.csv \
  _write_period:=0.5 _plan_stride:=1 _plan_dedupe:=true

# SAU KHI TÌM ĐƯỜNG ĐI XONG THÌ NHẤN Crtl+C ĐỂ DỪNG

# --- XUẤT FILE SO SÁNH ---
cd ~/AGV/src/ros_motion_planning/src/sim_env/script
chmod +x compare_logs.py
./compare_logs.py ~/ros_logs/DUONG_DI_ASTAR*.csv ~/ros_logs/DUONG_DI_DSTAR*.csv \
  --out ~/ros_logs/TONG_HOP_SO_SANH.csv

# TẤT CẢ CÁC FILE LOG ĐƯỢC LƯU VÀO ~/ros_logs/

# ==========================================================
# CHẠY XE AGV (CHƯA HOÀN THIỆN):
cd ~/AGV/
catkin_make
source devel/setup.bash
# gazebo:
roslaunch agv_description gazebo_xacro.launch
# Rviz:
roslaunch agv_description display_xacro.launch