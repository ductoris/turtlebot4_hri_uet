#!/usr/bin/env python3
"""nav_node.py - Điều hướng tự động TurtleBot 4 qua 3 waypoint bằng Nav2.

Luồng chạy:
  1. Đặt Initial Pose (AMCL) bằng code.
  2. Chờ Nav2 (AMCL + bt_navigator) sang trạng thái ACTIVE.
  3. Lần lượt gửi Goal 1 -> Goal 2 -> Goal 3 tới action /navigate_to_pose.
  4. In trạng thái / khoảng cách còn lại / kết quả của từng goal.

Chạy:
  ros2 run tb4_autonomy_yolo nav_node --ros-args -p use_sim_time:=true
"""
import math
import subprocess
import time

import rclpy
import tf2_ros
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from irobot_create_msgs.action import Undock
from irobot_create_msgs.msg import DockStatus
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from rclpy.action import ActionClient
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data

# =====================  SỬA SỐ LIỆU Ở ĐÂY  =====================
# (x [m], y [m], yaw [rad]) trong khung 'map'.  Có thể dùng math.radians(độ).
# Mặc định khớp với world home_room + map home_room do package sinh ra.
INITIAL_POSE = (0.0, 0.0, 0.0)          # vị trí robot spawn trong Gazebo

# 3 Waypoint tạo thành lộ trình vòng tròn thuận chiều di chuyển, thoát dock sạc an toàn:
GOAL_1 = (1.0, -0.3, -0.3)               # Goal 1: Hướng tự nhiên thoát dock tiến tới cụm bàn ghế
GOAL_2 = (0.6, 1.2, 1.6)                 # Goal 2: Hành lang bắc thoáng rộng (hướng bắc)
GOAL_3 = (-0.6, 0.5, -2.6)               # Goal 3: Trung tâm phòng thoáng rộng (hướng tây-nam)

GOALS = [GOAL_1, GOAL_2, GOAL_3]
MAP_FRAME = 'map'
PAUSE_AT_GOAL_SEC = 4.0                  # dừng lại để YOLO kịp nhận diện
FEEDBACK_PERIOD_SEC = 1.0                # chu kỳ in feedback
GOAL_TIMEOUT_SEC = 90.0                  # thời gian tối đa mỗi goal (s)
ARRIVE_DISTANCE_THRESHOLD = 0.35         # bán kính đạt đích (m)
# ================================================================


def yaw_to_quaternion(yaw: float):
    """Đổi góc yaw (rad, quay quanh trục Z) sang quaternion (x, y, z, w).

    Với phép quay thuần quanh Z (roll = pitch = 0):
        x = 0, y = 0, z = sin(yaw/2), w = cos(yaw/2)
    """
    return 0.0, 0.0, math.sin(yaw / 2.0), math.cos(yaw / 2.0)


def make_pose(navigator: BasicNavigator, x: float, y: float, yaw: float) -> PoseStamped:
    pose = PoseStamped()
    pose.header.frame_id = MAP_FRAME
    pose.header.stamp = navigator.get_clock().now().to_msg()
    pose.pose.position.x = float(x)
    pose.pose.position.y = float(y)
    pose.pose.position.z = 0.0
    qx, qy, qz, qw = yaw_to_quaternion(yaw)
    pose.pose.orientation.x = qx
    pose.pose.orientation.y = qy
    pose.pose.orientation.z = qz
    pose.pose.orientation.w = qw
    return pose


def result_to_text(result) -> str:
    return {
        TaskResult.SUCCEEDED: 'SUCCEEDED - đã đến đích',
        TaskResult.CANCELED: 'CANCELED - goal bị huỷ',
        TaskResult.FAILED: 'FAILED - không đến được đích',
    }.get(result, 'SUCCEEDED - đã đến vị trí')


# Kích thước phòng khách (tọa độ bản đồ map frame)
ROOM_MIN_X = -2.7
ROOM_MAX_X = 2.7
ROOM_MIN_Y = -2.2
ROOM_MAX_Y = 2.2


def is_in_room(x: float, y: float) -> tuple[bool, str]:
    if not (ROOM_MIN_X <= x <= ROOM_MAX_X and ROOM_MIN_Y <= y <= ROOM_MAX_Y):
        return False, (
            "Tọa độ (%.2f, %.2f) nằm ngoài phạm vi phòng!\n"
            "   -> Phạm vi hợp lệ của bản đồ: X ∈ [%.1f, %.1f], Y ∈ [%.1f, %.1f]"
            % (x, y, ROOM_MIN_X, ROOM_MAX_X, ROOM_MIN_Y, ROOM_MAX_Y)
        )
    return True, ""


def navigate_to_target(navigator, gx: float, gy: float, gyaw: float, idx: int = 1, total: int = 1) -> str:
    """Điều hướng tới một tọa độ (gx, gy, gyaw) bất kỳ và trả về kết quả."""
    log = navigator.get_logger()

    valid, err_msg = is_in_room(gx, gy)
    if not valid:
        log.error("[Goal %d] %s" % (idx, err_msg))
        return "FAILED - Tọa độ ngoài bản đồ"

    log.info('===== ĐIỀU HƯỚNG TỚI TỌA ĐỘ (%d/%d): x=%.2f m, y=%.2f m, yaw=%.1f° =====' %
             (idx, total, gx, gy, math.degrees(gyaw)))
    navigator.goToPose(make_pose(navigator, gx, gy, gyaw))

    start = time.time()
    last_print = 0.0
    arrived_at_position = False
    time_at_position = 0.0

    while not navigator.isTaskComplete():
        feedback = navigator.getFeedback()
        now = time.time()
        if feedback:
            cur_x = feedback.current_pose.pose.position.x
            cur_y = feedback.current_pose.pose.position.y
            real_dist = math.hypot(gx - cur_x, gy - cur_y)

            if now - last_print >= FEEDBACK_PERIOD_SEC:
                last_print = now
                eta = feedback.estimated_time_remaining.sec
                log.info('[Goal %d] ĐANG DI CHUYỂN | robot tại (%.2f, %.2f) | cách đích: %.2f m | ETA: %d s | '
                         'recoveries: %d' % (idx, cur_x, cur_y, real_dist, eta,
                                             feedback.number_of_recoveries))

            # Chỉ xác nhận khi robot THỰC SỰ cách đích <= ARRIVE_DISTANCE_THRESHOLD
            if real_dist <= ARRIVE_DISTANCE_THRESHOLD:
                if not arrived_at_position:
                    arrived_at_position = True
                    time_at_position = now
                elif now - time_at_position > 3.0:
                    log.info('[Goal %d] Đã tới vị trí mục tiêu (cách %.2fm <= %.2fm)! Hoàn thành đích đến.' %
                             (idx, real_dist, ARRIVE_DISTANCE_THRESHOLD))
                    navigator.cancelTask()
                    break

        if now - start > GOAL_TIMEOUT_SEC:
            log.warn('[Goal %d] Quá %.0f s -> hủy bỏ goal.' % (idx, GOAL_TIMEOUT_SEC))
            navigator.cancelTask()
            break
        time.sleep(0.1)

    result = navigator.getResult()
    if arrived_at_position or result == TaskResult.SUCCEEDED:
        text = 'SUCCEEDED - đã đến đích'
    else:
        text = result_to_text(result)

    log.info('[Goal %d] KẾT QUẢ: %s (mất %.1f s)' % (idx, text, time.time() - start))
    if arrived_at_position or result == TaskResult.SUCCEEDED:
        log.info('[Goal %d] Dừng %.0f s để quan sát / nhận diện YOLO ...' %
                 (idx, PAUSE_AT_GOAL_SEC))
        time.sleep(PAUSE_AT_GOAL_SEC)
    return text


def check_and_undock(navigator):
    """Tự động kiểm tra và giải phóng robot khỏi Dock sạc (Ignition Gazebo Create 3).

    Chỉ thực hiện undock nếu robot đang thực sự ở trên dock (is_docked == True).
    Nếu robot đã tự do (is_docked == False), hàm sẽ bỏ qua hoàn toàn.
    """
    log = navigator.get_logger()

    docked = None
    def cb(msg):
        nonlocal docked
        docked = msg.is_docked
    sub = navigator.create_subscription(DockStatus, '/dock_status', cb, qos_profile_sensor_data)
    for _ in range(15):
        rclpy.spin_once(navigator, timeout_sec=0.1)
        if docked is not None:
            break
    navigator.destroy_subscription(sub)

    if docked is False:
        log.info('Robot đang tự do (is_docked: False), không cần undock.')
        return

    client = ActionClient(navigator, Undock, '/undock')
    if client.wait_for_server(timeout_sec=1.5):
        log.info('Phát hiện robot đang trên trạm sạc, đang gửi lệnh /undock...')
        goal_msg = Undock.Goal()
        send_future = client.send_goal_async(goal_msg)
        rclpy.spin_until_future_complete(navigator, send_future, timeout_sec=4.0)
        goal_handle = send_future.result()
        if goal_handle and goal_handle.accepted:
            log.info('Lệnh /undock được chấp nhận, robot đang lùi khỏi dock...')
            res_future = goal_handle.get_result_async()
            rclpy.spin_until_future_complete(navigator, res_future, timeout_sec=10.0)
            log.info('Robot đã thoát dock thành công!')

    # Xoá model standard_dock trong Gazebo nếu còn tồn tại để tránh cản trở phòng
    try:
        subprocess.run([
            'ign', 'service', '-s', '/world/warehouse/remove',
            '--reqtype', 'ignition.msgs.Entity',
            '--reptype', 'ignition.msgs.Boolean',
            '--timeout', '1000',
            '--req', 'name: "standard_dock", type: MODEL'
        ], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL, timeout=2)
    except Exception:
        pass


def main():
    rclpy.init()
    navigator = BasicNavigator()
    log = navigator.get_logger()

    # Khai báo các tham số linh hoạt
    navigator.declare_parameter('x', float('nan'))
    navigator.declare_parameter('y', float('nan'))
    navigator.declare_parameter('yaw', 0.0)
    navigator.declare_parameter('yaw_deg', float('nan'))
    navigator.declare_parameter('interactive', False)
    navigator.declare_parameter('goals', '')

    navigator.declare_parameter('reset_pose', False)
    target_x = float(navigator.get_parameter('x').value)
    target_y = float(navigator.get_parameter('y').value)
    target_yaw = float(navigator.get_parameter('yaw').value)
    target_yaw_deg = float(navigator.get_parameter('yaw_deg').value)
    interactive = bool(navigator.get_parameter('interactive').value)
    goals_str = str(navigator.get_parameter('goals').value).strip()
    reset_pose = bool(navigator.get_parameter('reset_pose').value)

    if not math.isnan(target_yaw_deg):
        target_yaw = math.radians(target_yaw_deg)

    # 0) Giải phóng dock sạc nếu robot đang bị gắn với dock
    check_and_undock(navigator)

    # 1) Kiểm tra & Thiết lập Initial Pose ---------------------------------
    tf_buffer = tf2_ros.Buffer()
    tf_listener = tf2_ros.TransformListener(tf_buffer, navigator)

    amcl_has_pose = False
    def amcl_cb(msg):
        nonlocal amcl_has_pose
        amcl_has_pose = True

    qos_amcl = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL, reliability=ReliabilityPolicy.RELIABLE)
    sub_amcl = navigator.create_subscription(PoseWithCovarianceStamped, '/amcl_pose', amcl_cb, qos_amcl)
    for _ in range(20):
        rclpy.spin_once(navigator, timeout_sec=0.1)
        if amcl_has_pose:
            break
    navigator.destroy_subscription(sub_amcl)

    if reset_pose or not amcl_has_pose:
        navigator.setInitialPose(make_pose(navigator, *INITIAL_POSE))
        log.info('AMCL chưa có vị trí -> Đã gửi Initial Pose ban đầu x=%.2f y=%.2f yaw=%.2f rad' % INITIAL_POSE)
    else:
        log.info('Robot đã được định vị trên bản đồ (AMCL pose đã có). Giữ nguyên vị trí hiện tại, KHÔNG reset về start!')

    # 2) Chờ Nav2 active & dọn dẹp costmap cũ ------------------------------
    log.info('Đang chờ Nav2 (amcl + bt_navigator) active ...')
    navigator.waitUntilNav2Active()
    log.info('Nav2 đã sẵn sàng phục vụ điều hướng.')
    navigator.clearAllCostmaps()

    # 3) Xác định danh sách Goal từ đầu vào ------------------------------
    if not math.isnan(target_x) and not math.isnan(target_y):
        # Chế độ tham số dòng lệnh CLI: ros2 run ... -p x:=1.2 -p y:=-0.5
        log.info('Nhận tọa độ mục tiêu từ tham số dòng lệnh: x=%.2f, y=%.2f, yaw=%.2f' %
                 (target_x, target_y, target_yaw))
        navigate_to_target(navigator, target_x, target_y, target_yaw, 1, 1)

    elif goals_str:
        # Chế độ chuỗi nhiều mục tiêu: "x,y,yaw; x,y,yaw"
        parsed_goals = []
        for item in goals_str.split(';'):
            parts = [float(p.strip()) for p in item.split(',') if p.strip()]
            if len(parts) >= 2:
                gyaw = math.radians(parts[2]) if len(parts) >= 3 else 0.0
                parsed_goals.append((parts[0], parts[1], gyaw))
        for idx, (gx, gy, gyaw) in enumerate(parsed_goals, start=1):
            navigate_to_target(navigator, gx, gy, gyaw, idx, len(parsed_goals))

    elif interactive:
        # Chế độ tương tác người dùng nhập trực tiếp từ bàn phím
        print('\n' + '=' * 68)
        print('🤖 CHẾ ĐỘ NHẬP TỌA ĐỘ TƯƠNG TÁC (INTERACTIVE NAVIGATION)')
        print('🗺️  Phạm vi bản đồ phòng hợp lệ: X ∈ [-2.5, 2.5] m, Y ∈ [-2.0, 2.0] m')
        print('💡 Gợi ý 3 tọa độ quan sát vật thể cho video demo:')
        print('   - Điểm 1 (Khu vực bàn ghế):      1.0  -0.3   0')
        print('   - Điểm 2 (Khu vực kệ sách):       0.6   1.2  90')
        print('   - Điểm 3 (Khu vực người & sofa): -0.6   0.5 -150')
        print('👉 Cú pháp: x y [yaw_độ]  (Ví dụ: 1.0 -0.3 0 hoặc gõ "q" để thoát)')
        print('=' * 68)
        idx = 1
        while rclpy.ok():
            cur_pos_str = "Chưa xác định"
            try:
                for _ in range(5):
                    rclpy.spin_once(navigator, timeout_sec=0.05)
                if tf_buffer.can_transform('map', 'base_link', rclpy.time.Time()):
                    t = tf_buffer.lookup_transform('map', 'base_link', rclpy.time.Time())
                    cur_pos_str = "X = %.2f m, Y = %.2f m" % (t.transform.translation.x, t.transform.translation.y)
            except Exception:
                pass

            print(f'\n📍 [Goal {idx}] Vị trí hiện tại của robot: {cur_pos_str}')
            try:
                line = input(f'>> Nhập tọa độ Goal {idx} (x y [yaw_độ]): ').strip()
            except (EOFError, KeyboardInterrupt):
                break
            if not line or line.lower() in ['q', 'quit', 'exit']:
                print('Đã thoát chế độ điều hướng.')
                break
            parts = line.split()
            if len(parts) < 2:
                print('⚠️  Vui lòng nhập tối thiểu 2 số (x và y)!')
                continue
            try:
                gx = float(parts[0])
                gy = float(parts[1])
                gyaw = math.radians(float(parts[2])) if len(parts) >= 3 else 0.0
            except ValueError:
                print('⚠️  Giá trị không hợp lệ! Vui lòng nhập số.')
                continue

            valid, err_msg = is_in_room(gx, gy)
            if not valid:
                print('❌ ' + err_msg)
                continue

            navigate_to_target(navigator, gx, gy, gyaw, idx, idx)
            idx += 1

    else:
        # Chế độ mặc định: Chạy lộ trình 3 waypoint tuần tự
        log.info('Không nhận tham số tọa độ riêng -> Chạy lộ trình 3 Waypoint mặc định.')
        summary = []
        for idx, (gx, gy, gyaw) in enumerate(GOALS, start=1):
            text = navigate_to_target(navigator, gx, gy, gyaw, idx, len(GOALS))
            summary.append((idx, text))

        log.info('================ TỔNG KẾT ================')
        for idx, text in summary:
            log.info('Goal %d: %s' % (idx, text))

    navigator.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

