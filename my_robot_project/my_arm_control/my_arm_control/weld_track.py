#!/usr/bin/env python3
# encoding: utf-8
# 標題：基於 YOLO ROS 2 服務與深度資訊的輕觸控制與軌跡預測 ROS 2 節點

import cv2
import time
import copy
import rclpy
import threading
import numpy as np

from rclpy.node import Node
from cv_bridge import CvBridge

# 移除 ultralytics 直接調用，改採用 ROS 2 自訂訊息與 Trigger 服務
from std_srvs.srv import Trigger
from sensor_msgs.msg import Image, CameraInfo
from servo_controller_msgs.msg import ServosPosition
from kinematics_msgs.srv import SetRobotPose
from kinematics.kinematics_control import set_pose_target
from servo_controller.bus_servo_control import set_servo_position
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup

from interfaces.msg import ObjectsInfo  # YOLO 介面訊息


class YoloTouchPlannerNode(Node):
    def __init__(self, name):
        # 初始化 ROS 2 客戶端與節點
        super().__init__(name, allow_undeclared_parameters=True, automatically_declare_parameters_from_overrides=True)
        self.name = name

        # 1. 影像與相機內參初始化
        self.bridge = CvBridge()
        self.rgb_image = None      # 儲存來自 yolov8/object_image 的影像
        self.depth_image = None    # 儲存深度影像
        self.lock = threading.RLock()
        
        self.fx = 525.0
        self.fy = 525.0
        self.cx = 320.0
        self.cy = 240.0

        # 2. YOLO 狀態與目標儲存
        self.target_object_info = None
        self.target_list = ['weldboard_s', 'weldboard_l', 'al_plane_s', 'al_plane_m', 'al_plane_l']  # 允許偵測的目標白名單

        # 3. 錄影與軌跡預測狀態管理
        self.is_recording = False
        self.is_executing = False
        self.recording_start_time = 0.0
        self.recorded_trajectories = []
        self.current_status_text = "Idle"

        # 4. Callback Group 與 Client 初始化
        self.cb_group = ReentrantCallbackGroup()

        self.start_yolov8_client = self.create_client(Trigger, 'yolov8/start', callback_group=self.cb_group)
        self.stop_yolov8_client = self.create_client(Trigger, 'yolov8/stop', callback_group=self.cb_group)

        self.get_logger().info('等待 YOLO 服務啟動...')

        # 5. 發布者與運動學服務
        self.joints_pub = self.create_publisher(ServosPosition, '/servo_controller', 1)
        self.result_pub = self.create_publisher(Image, '~/image_result', 1)
        
        self.kinematics_client = self.create_client(SetRobotPose, '/kinematics/set_pose_target', callback_group=self.cb_group)

        # 6. 訂閱者設定 (影像與 YOLO 偵測數據)
        self.depth_sub = self.create_subscription(Image, '/depth_cam/depth/image_raw', self.depth_callback, 1)
        self.info_sub = self.create_subscription(CameraInfo, '/depth_cam/rgb/camera_info', self.camera_info_callback, 1)
        
        # 使用 waste_classification 的 YOLO 話題
        self.image_sub = self.create_subscription(Image, 'yolov8/object_image', self.rgb_callback, 1)
        self.object_sub = self.create_subscription(ObjectsInfo, 'yolov8/object_detect', self.get_object_callback, 1)

        # 啟動主處理背景線程
        threading.Thread(target=self.main_loop, daemon=True).start()
        self.get_logger().info('\033[1;32m系統初始化完成，請在 OpenCV 畫面中按下 [空白鍵] 開始/停止錄影軌跡\033[0m')

    def _wait_for_service(self, client, timeout_sec=2.0):
        if client.wait_for_service(timeout_sec=timeout_sec):
            return True
        self.get_logger().error(f'Service unavailable: {client.srv_name}')
        return False

    def send_service_request(self, client, request, timeout_sec=5.0):
        """安全發送 Async 服務請求，避免無限期阻塞""" 
        if not self._wait_for_service(client):
            self.get_logger().error(f"服務 {client.srv_name} 未就緒！")
            return None
        
        future = client.call_async(request)
        start_time = time.time()
    
        while rclpy.ok():
            if future.done():
                return future.result()
            if time.time() - start_time > timeout_sec:
                self.get_logger().error(f"調用服務 {client.srv_name} 超時！")
                return None
            time.sleep(0.01)
        return None

    def camera_info_callback(self, msg):
        """讀取相機內參"""
        self.fx = msg.k[0]
        self.fy = msg.k[4]
        self.cx = msg.k[2]
        self.cy = msg.k[5]

    def rgb_callback(self, msg):
        """接收 YOLO 標註後的影像"""
        with self.lock:
            self.rgb_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")

    def depth_callback(self, msg):
        """深度影像訂閱"""
        with self.lock:
            self.depth_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")

    def get_object_callback(self, msg):
        """修正並簡化 YOLO 數據接收與解析邏輯"""
        objects = msg.objects
        local_target_object_info = None
        local_objects_list = []
        selected_target = None

        if objects:
            for i in objects:
                # 角度修正
                angle = 90 - abs(i.angle) if i.angle < 0 else i.angle
                target = [i.class_name, 0, (int(i.box[0]), int(i.box[1])), (int(i.box[2]), int(i.box[3])), angle]
                local_objects_list.append(target)

                # 挑選第一個符合白名單的目標
                if i.class_name in self.target_list and selected_target is None:
                    selected_target = target

            if selected_target is not None:
                local_target_object_info = [selected_target, local_objects_list]

        with self.lock:
            self.target_object_info = copy.deepcopy(local_target_object_info)

    def pixel_to_3d(self, u, v, depth_map):
        """像素座標結合深度圖轉為相機空間 3D 座標"""
        if depth_map is None:
            return None
        
        h, w = depth_map.shape
        u_min, u_max = max(0, int(u) - 2), min(w, int(u) + 3)
        v_min, v_max = max(0, int(v) - 2), min(h, int(v) + 3)
        
        depth_patch = depth_map[v_min:v_max, u_min:u_max]
        valid_depths = depth_patch[depth_patch > 0]
        
        if len(valid_depths) == 0:
            return None
        
        z = float(np.median(valid_depths))
        if z > 10.0:
            z /= 1000.0
            
        if z <= 0.0:
            return None

        x = (u - self.cx) * z / self.fx
        y = (v - self.cy) * z / self.fy
        return np.array([x, y, z])

    def send_kinematics_request(self, pose_t, pitch=-90.0, timeout_sec=5.0):
        """發送 IK 請求並包含驗證輸出"""
        if not self._wait_for_service(self.kinematics_client):
            return None
        msg = set_pose_target(pose_t, pitch, [-180.0, 180.0], 1.0)
        future = self.kinematics_client.call_async(msg)
        deadline = time.monotonic() + timeout_sec
        while rclpy.ok():
            if future.done() and future.result():
                res = future.result()
                # 測試點 2 驗證：印出目標空間點與 IK 反解出的脈衝訊號，確保解存在
                if res and res.pulse:
                    self.get_logger().info(f"[IK 驗證] 姿態 {np.round(pose_t, 3)} -> 脈衝: {res.pulse}")
                else:
                    self.get_logger().error(f"[IK 驗證失敗] 空間座標 {np.round(pose_t, 3)} 超出工作空間或無解！")
                return res
            if time.monotonic() >= deadline:
                self.get_logger().error('IK request timed out.')
                return None
            time.sleep(0.01)

    def _send_servo_command(self, duration, servo_tuple):
        """安全發送舵機指令（含脈衝邊界防護與轉角驗證）"""
        valid_servos = []
        for servo_id, pulse in servo_tuple:
            # 測試點 3 驗證：檢查匯流排舵機 Pulse 是否在安全範圍 (0~1000 脈衝對應 0~240 度)
            clamped_pulse = max(0, min(1000, int(pulse)))
            angle_deg = (clamped_pulse / 1000.0) * 240.0
            self.get_logger().debug(f"[馬達控制] Servo ID: {servo_id} | 脈衝: {clamped_pulse} | 目標角度: {angle_deg:.1f}°")
            valid_servos.append((servo_id, clamped_pulse))

        set_servo_position(self.joints_pub, duration, tuple(valid_servos))

    def execute_touch_action(self, target_3d_pos):
        """執行表面輕觸動作"""
        self.current_status_text = "Executing Touch Action..."
        self.get_logger().info(f"開始執行表面輕觸任務，目標空間座標: {target_3d_pos}")

        touch_target = list(target_3d_pos)
        touch_target[2] += 0.02

        approach_target = list(touch_target)
        approach_target[2] += 0.05
        
        res_approach = self.send_kinematics_request(approach_target)
        if res_approach and res_approach.pulse:
            servo_data = res_approach.pulse
            self._send_servo_command(1.5, (
                (1, servo_data[0]), (2, servo_data[1]), 
                (3, servo_data[2]), (4, servo_data[3]), (5, 500)
            ))
            time.sleep(1.8)

        if not (res_approach and res_approach.pulse):
            self.current_status_text = "Error: Approach IK Failed"
            return

        res_touch = self.send_kinematics_request(touch_target)
        if res_touch and res_touch.pulse:
            servo_data = res_touch.pulse
            self._send_servo_command(1.0, (
                (1, servo_data[0]), (2, servo_data[1]), 
                (3, servo_data[2]), (4, servo_data[3])
            ))
            time.sleep(1.2)

        if res_approach and res_approach.pulse:
            servo_data = res_approach.pulse
            self._send_servo_command(1.0, (
                (1, servo_data[0]), (2, servo_data[1]), 
                (3, servo_data[2]), (4, servo_data[3])
            ))
            time.sleep(1.0)

        self.current_status_text = "Touch Completed"
    
    def predict_and_execute_trajectory(self):
        """軌跡預測與執行"""
        if self.is_executing:
            self.get_logger().warn("軌跡正在執行中，忽略重複請求！")
            return

        self.is_executing = True
        try:
            if len(self.recorded_trajectories) < 5:
                self.get_logger().warn("錄影軌跡點過少，無法生成有效運動軌跡")
                self.current_status_text = "Error: Too Few Points"
                return
            
            self.current_status_text = "Predicting Trajectory..."
            self.get_logger().info("開始整合深度圖與 YOLO 歷史點陣，計算移動軌跡...")

            raw_pts = np.array(self.recorded_trajectories)
            t_steps = np.linspace(0, 1, len(raw_pts))
            t_smooth = np.linspace(0, 1, 10)

            smooth_x = np.interp(t_smooth, t_steps, raw_pts[:, 0])
            smooth_y = np.interp(t_smooth, t_steps, raw_pts[:, 1])
            smooth_z = np.interp(t_smooth, t_steps, raw_pts[:, 2])

            predicted_trajectory = np.column_stack((smooth_x, smooth_y, smooth_z))

            self.current_status_text = "Executing Trajectory..."
            for idx, pt in enumerate(predicted_trajectory):
                self.get_logger().info(f"執行軌跡點 [{idx+1}/10]: {pt}")
                res = self.send_kinematics_request(pt)
                if res and res.pulse:
                    servo_data = res.pulse
                    self._send_servo_command(0.3, (
                        (1, servo_data[0]), (2, servo_data[1]), 
                        (3, servo_data[2]), (4, servo_data[3])
                        ))
                    time.sleep(0.35)
                else:
                    self.get_logger().error(f"軌跡點 [{idx+1}/10] 運動學解算失敗，跳過該點以維護手臂安全！")

            self.execute_touch_action(predicted_trajectory[-1])

        finally:
            self.is_executing = False
    
    def main_loop(self):
        """主循環線程"""
        cv2.namedWindow("YOLO Touch & Trajectory Control", cv2.WINDOW_AUTOSIZE)

        while rclpy.ok():
            with self.lock:
                if self.rgb_image is None:
                    time.sleep(0.03)
                    continue
                frame = self.rgb_image.copy()
                depth_map = self.depth_image.copy() if self.depth_image is not None else None
                target_info = copy.deepcopy(self.target_object_info)

            current_target_3d = None

            # 解析 YOLO 傳回的目標中心座標
            if target_info is not None:
                cx, cy = target_info[0][2]
                
                # 計算 3D 空間座標
                if depth_map is not None:
                    coord_3d = self.pixel_to_3d(cx, cy, depth_map)
                    if coord_3d is not None:
                        current_target_3d = coord_3d
                        coord_text = f"X:{coord_3d[0]:.2f}m Y:{coord_3d[1]:.2f}m Z:{coord_3d[2]:.2f}m"
                        cv2.putText(frame, coord_text, (cx - 50, cy + 20), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 0), 2)
                        
                        # 測試點 1 驗證：印出視覺感知轉換出的 3D 座標
                        self.get_logger().debug(f"[視覺 3D 驗證] 像素 ({cx}, {cy}) -> 相機座標 {coord_3d}")

            # 錄影與軌跡採樣
            elapsed_time = 0.0
            if self.is_recording:
                elapsed_time = time.time() - self.recording_start_time
                self.current_status_text = f"Recording... ({elapsed_time:.1f}s)"
                
                if current_target_3d is not None:
                    self.recorded_trajectories.append(current_target_3d)

            # 繪製 UI 面板
            cv2.rectangle(frame, (10, 10), (320, 90), (0, 0, 0), -1)
            cv2.putText(frame, f"Status: {self.current_status_text}", (20, 35), 
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)
            cv2.putText(frame, f"Rec Time: {elapsed_time:.1f} s", (20, 65), 
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)

            if self.is_recording:
                cv2.circle(frame, (300, 35), 8, (0, 0, 255), -1)

            cv2.imshow("YOLO Touch & Trajectory Control", frame)
            key = cv2.waitKey(30) & 0xFF

            # 空白鍵：控制錄影與發送 yolov8/start/stop
            if key == 32:
                if not self.is_recording:
                    # 開啟 YOLO 推論並開始錄影
                    self.send_service_request(self.start_yolov8_client, Trigger.Request())
                    self.is_recording = True
                    self.recording_start_time = time.time()
                    self.recorded_trajectories = []
                    self.get_logger().info("開始錄影：啟動 YOLO 服務並捕捉軌跡...")
                else:
                    # 停止錄影並關閉 YOLO 服務節省資源
                    self.is_recording = False
                    self.send_service_request(self.stop_yolov8_client, Trigger.Request())
                    total_rec_time = time.time() - self.recording_start_time
                    self.get_logger().info(f"錄影結束！總錄影時間: {total_rec_time:.2f} 秒，共收集 {len(self.recorded_trajectories)} 個點")
                    
                    threading.Thread(target=self.predict_and_execute_trajectory, daemon=True).start()

            self.result_pub.publish(self.bridge.cv2_to_imgmsg(frame, "bgr8"))

        cv2.destroyAllWindows()


def main():
    rclpy.init()
    node = YoloTouchPlannerNode('yolo_touch_planner')
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
