#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
任务状态机节点 - Python版本
功能：复制0816.cpp的完整状态机逻辑
安全优先：保持与原C++代码完全相同的状态转换逻辑
"""

import rospy
import time
from std_msgs.msg import Bool, Int32
from mavros_msgs.msg import State, PositionTarget
from mavros_msgs.srv import CommandBool, SetMode, CommandLong
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PointStamped
from std_srvs.srv import Empty
import math

# 临时服务消息定义（实际应该从srv文件生成）
class MissionControlRequest:
    def __init__(self):
        self.function_name = ""
        self.params = []
        self.string_param = ""

class MissionControlResponse:
    def __init__(self):
        self.success = False
        self.message = ""

class MissionStateMachine:
    def __init__(self):
        """初始化状态机节点"""
        rospy.init_node('mission_state_machine_python', anonymous=True)
        
        # ============ 状态变量 (对应C++全局变量) ============
        self.mission_num = 0
        self.target_qr_id = 1
        self.current_qr_id = 0
        self.qr_land_done_flag = False
        self.last_published_target_qr_id = -1
        
        # 时间记录变量
        self.tmp_time_record_start_flag = False
        self.tmp_mission_success_time_record = None
        self.tmp_x = 0.0
        self.tmp_y = 0.0
        
        # 窗口识别变量  
        self.object_data_cleared = False
        self.window_search_start_time = None
        
        # 飞机状态变量
        self.current_state = None
        self.local_pos = None
        self.object_pos = None
        
        # 常量
        self.ALTITUDE = 0.8
        
        # ============ ROS订阅者 ============
        self.state_sub = rospy.Subscriber(
            "mavros/state", State, self.state_cb, queue_size=10)
        self.local_pos_sub = rospy.Subscriber(
            "/mavros/local_position/odom", Odometry, self.local_pos_cb, queue_size=10)
        self.object_pos_sub = rospy.Subscriber(
            "object_position", PointStamped, self.object_pos_cb, queue_size=100)
        self.qr_done_sub = rospy.Subscriber(
            "qr_land_done", Bool, self.qr_land_cb, queue_size=10)
        self.qr_id_sub = rospy.Subscriber(
            "qr_code_id", Int32, self.qr_id_cb, queue_size=10)
        
        # ============ ROS发布者 ============
        self.setpoint_pub = rospy.Publisher(
            "/mavros/setpoint_raw/local", PositionTarget, queue_size=100)
        self.target_qr_id_pub = rospy.Publisher(
            "/target_qr_id", Int32, queue_size=10)
        
        # ============ ROS服务客户端 ============
        self.arming_client = rospy.ServiceProxy("mavros/cmd/arming", CommandBool)
        self.set_mode_client = rospy.ServiceProxy("mavros/set_mode", SetMode)
        self.ctrl_pwm_client = rospy.ServiceProxy("mavros/cmd/command", CommandLong)
        
        # 任务控制服务客户端
        # self.mission_control_client = rospy.ServiceProxy("/mission_control", MissionControl)
        self.reset_window_flags_client = rospy.ServiceProxy("/reset_window_flags", Empty)
        
        # ============ 等待服务启动 ============
        rospy.loginfo("等待MAVROS服务启动...")
        self.arming_client.wait_for_service(timeout=30)
        self.set_mode_client.wait_for_service(timeout=30)
        self.ctrl_pwm_client.wait_for_service(timeout=30)
        
        # 等待任务控制服务
        rospy.loginfo("等待任务控制服务启动...")
        # self.mission_control_client.wait_for_service(timeout=30)
        self.reset_window_flags_client.wait_for_service(timeout=30)
        
        rospy.loginfo("所有服务已连接")
        
        # ============ 时间记录辅助函数 ============
        self.lib_time_record_start_flag = False
        self.lib_mission_success_time_record = None
        
    def lib_time_record_func(self, time_duration):
        """时间记录函数 - 对应C++的lib_time_record_func"""
        current_time = rospy.Time.now()
        
        if not self.lib_time_record_start_flag:
            self.lib_mission_success_time_record = current_time
            self.lib_time_record_start_flag = True
            return False
        
        if (current_time - self.lib_mission_success_time_record).to_sec() > time_duration:
            self.lib_time_record_start_flag = False
            return True
        
        return False
    
    def lib_pwm_control(self, pwm1, pwm2):
        """PWM控制函数 - 对应C++的lib_pwm_control"""
        try:
            cmd = CommandLong()
            cmd.request.command = 183  # MAV_CMD_DO_SET_SERVO
            cmd.request.param1 = 9     # 舵机通道9
            cmd.request.param2 = pwm1  # PWM值
            cmd.request.param3 = 10    # 舵机通道10  
            cmd.request.param4 = pwm2  # PWM值
            
            response = self.ctrl_pwm_client.call(cmd)
            return response.success
        except rospy.ServiceException as e:
            rospy.logerr(f"PWM控制服务调用失败: {e}")
            return False
    
    # ============ 回调函数 ============
    def state_cb(self, msg):
        """飞机状态回调"""
        self.current_state = msg
    
    def local_pos_cb(self, msg):
        """位置信息回调"""
        self.local_pos = msg
    
    def object_pos_cb(self, msg):
        """目标检测回调"""
        self.object_pos = msg
    
    def qr_land_cb(self, msg):
        """二维码降落标志回调"""
        self.qr_land_done_flag = msg.data
    
    def qr_id_cb(self, msg):
        """二维码ID回调"""
        self.current_qr_id = msg.data
    
    # ============ 任务函数封装 (调用C++服务) ============
    def call_mission_service(self, function_name, params, string_param=""):
        """调用任务控制服务"""
        try:
            # 这里需要导入服务消息类型
            # 暂时用简单的方法模拟服务调用
            rospy.logdebug(f"调用服务: {function_name} 参数: {params}")
            
            # 实际服务调用代码应该是：
            # req = MissionControlRequest()
            # req.function_name = function_name
            # req.params = params  
            # req.string_param = string_param
            # response = self.mission_control_client.call(req)
            # return response.success
            
            # 暂时返回False，表示任务进行中
            return False
            
        except Exception as e:
            rospy.logerr(f"服务调用失败: {e}")
            return False
    
    def mission_pos_cruise(self, x, y, z, yaw, error_max):
        """位置巡航"""
        return self.call_mission_service("mission_pos_cruise", [x, y, z, yaw, error_max])
    
    def object_recognize_track(self, target_str, yaw, reverse, altitude, error_max, ctrl_coef):
        """目标跟踪"""
        return self.call_mission_service("object_recognize_track", 
                                       [yaw, reverse, altitude, error_max, ctrl_coef], 
                                       target_str)
    
    def target_through(self, x, y, altitude, yaw):
        """穿越目标"""
        return self.call_mission_service("target_through", [x, y, altitude, yaw])
    
    def current_position_cruise(self, x, y, z, yaw, error_max):
        """当前位置巡航"""
        return self.call_mission_service("current_position_cruise", [x, y, z, yaw, error_max])
    
    def mission_obs_avoid(self, x, y, altitude, yaw):
        """避障导航"""
        return self.call_mission_service("mission_obs_avoid", [x, y, altitude, yaw])
    
    def reset_window_flags(self):
         """重置窗口识别标志位"""
         try:
             # 调用重置服务
             response = self.reset_window_flags_client.call()
             rospy.loginfo("重置窗口识别标志位")
             return True
         except Exception as e:
             rospy.logerr(f"重置窗口标志位失败: {e}")
             return False
    
    # ============ 主状态机 ============
    def wait_for_connection(self):
        """等待与飞控连接"""
        rospy.loginfo("等待与飞控连接...")
        rate = rospy.Rate(20)
        
        while rospy.ok() and (self.current_state is None or not self.current_state.connected):
            rospy.spinOnce()
            rate.sleep()
        
        rospy.loginfo("已连接到飞控")
    
    def pre_flight_setup(self):
        """起飞前设置"""
        rospy.loginfo("起飞前设置...")
        
        # 发布初始setpoint
        setpoint = PositionTarget()
        setpoint.type_mask = 64 + 128 + 256 + 512
        setpoint.coordinate_frame = 1
        setpoint.position.x = 0
        setpoint.position.y = 0  
        setpoint.position.z = self.ALTITUDE
        setpoint.yaw = 0
        
        # 发布100次稳定setpoint
        for i in range(100):
            if rospy.is_shutdown():
                return False
            self.setpoint_pub.publish(setpoint)
            rospy.sleep(0.05)  # 20Hz
        
        return True
    
    def arm_and_takeoff(self):
        """解锁和起飞"""
        rospy.loginfo("开始解锁和起飞...")
        
        last_request = rospy.Time.now()
        rate = rospy.Rate(20)
        
        # 舵机复位
        self.lib_pwm_control(0, 0)
        
        while rospy.ok():
            current_time = rospy.Time.now()
            
            # 切换到OFFBOARD模式
            if (self.current_state.mode != "OFFBOARD" and 
                (current_time - last_request).to_sec() > 5.0):
                try:
                    result = self.set_mode_client.call(custom_mode="OFFBOARD")
                    if result.mode_sent:
                        rospy.loginfo("OFFBOARD模式已启用")
                    last_request = current_time
                except rospy.ServiceException as e:
                    rospy.logerr(f"设置模式失败: {e}")
            
            # 解锁
            elif (not self.current_state.armed and 
                  (current_time - last_request).to_sec() > 5.0):
                try:
                    result = self.arming_client.call(value=True)
                    if result.success:
                        rospy.loginfo("无人机已解锁")
                    last_request = current_time
                except rospy.ServiceException as e:
                    rospy.logerr(f"解锁失败: {e}")
            
            # 检查是否到达目标高度
            if (self.local_pos is not None and 
                abs(self.local_pos.pose.pose.position.z - self.ALTITUDE) < 0.1):
                if (current_time - last_request).to_sec() > 4.0:
                    rospy.loginfo("到达目标高度，开始稳定悬停")
                    self.mission_num = 1
                    break
            
            # 继续发布起飞指令
            setpoint = PositionTarget()
            setpoint.type_mask = 64 + 128 + 256 + 512
            setpoint.coordinate_frame = 1
            setpoint.position.x = 0
            setpoint.position.y = 0
            setpoint.position.z = self.ALTITUDE
            setpoint.yaw = 0
            self.setpoint_pub.publish(setpoint)
            
            rate.sleep()
    
    def run_state_machine(self):
        """运行主状态机"""
        rospy.loginfo("开始运行状态机...")
        rate = rospy.Rate(20)  # 20Hz
        tmp_mission = 1
        
        while rospy.ok():
            # 状态变化日志
            if tmp_mission != self.mission_num:
                tmp_mission = self.mission_num
                rospy.loginfo(f"mission_num = {self.mission_num}")
            
            # ============ 状态机主逻辑 ============
            if self.mission_num == 1:
                # 任务1：稳定悬停阶段
                if (self.mission_pos_cruise(0, 0, self.ALTITUDE, 0, 0.2) and 
                    self.lib_time_record_func(0.5)):
                    rospy.loginfo("稳定悬停完成，准备开始窗口识别")
                    self.object_data_cleared = False
                    self.window_search_start_time = rospy.Time.now()
                    self.mission_num = 2
            
            elif self.mission_num == 2:
                # 任务2：窗口识别
                if self.object_recognize_track("square", 0, 1, self.ALTITUDE, 35, 0.1):
                    self.mission_num = 3
            
            elif self.mission_num == 3:
                # 任务3：穿过窗口
                if self.target_through(2.7, 0.0, self.ALTITUDE, 0):
                    self.mission_num = 4
            
            elif self.mission_num == 4:
                # 第1个任务点
                if self.mission_pos_cruise(2.7, -0.2, self.ALTITUDE, 0, 0.2):
                    self.mission_num = 41
            
            elif self.mission_num == 41:
                if self.current_position_cruise(0.0, 0.0, self.ALTITUDE, 0, 0.2):
                    self.mission_num = 42
            
            elif self.mission_num == 42:
                # 第1个任务点：二维码对准
                self.handle_qr_code_mission(target_id=1, next_mission=5, timeout=5.0)
            
            elif self.mission_num == 5:
                # 第2个任务点
                if self.mission_obs_avoid(2.7, 1.35, self.ALTITUDE, 0):
                    self.mission_num = 51
            
            elif self.mission_num == 51:
                if self.current_position_cruise(0.0, 0.0, self.ALTITUDE, 0, 0.2):
                    self.mission_num = 52
            
            elif self.mission_num == 52:
                # 第2个任务点：二维码对准
                self.handle_qr_code_mission(target_id=4, next_mission=6, timeout=5.0)
            
            elif self.mission_num == 6:
                # 第3个任务点
                if self.mission_obs_avoid(2.7, 1.35, self.ALTITUDE, 0):  # 取消明显飞行动作
                    self.mission_num = 61
            
            elif self.mission_num == 61:
                if self.current_position_cruise(0.0, 0.0, self.ALTITUDE, 0, 0.2):
                    self.mission_num = 62
            
            elif self.mission_num == 62:
                # 第3个任务点：二维码对准
                self.handle_qr_code_mission(target_id=3, next_mission=7, timeout=2.0)
            
            elif self.mission_num == 7:
                # 第4个任务点 + 出窗
                if self.mission_obs_avoid(3.1, 2.7, self.ALTITUDE, 0):
                    self.mission_num = 71
            
            elif self.mission_num == 71:
                if self.current_position_cruise(0.0, 0.0, self.ALTITUDE, 3.14, 0.2):
                    self.mission_num = 711
            
            elif self.mission_num == 711:
                # 第二次穿窗前稳定悬停
                if (self.current_position_cruise(0.0, 0.0, self.ALTITUDE, 3.14, 0.2) and 
                    self.lib_time_record_func(1.0)):
                    rospy.loginfo("第二次穿窗前稳定悬停完成，开始窗口识别")
                    self.mission_num = 72
            
            elif self.mission_num == 72:
                # 第二次窗口识别
                self.handle_second_window_detection()
            
            elif self.mission_num == 73:
                # 第二次穿窗
                if self.target_through(-2.6, 0, self.ALTITUDE, 3.14):
                    self.mission_num = 8
            
            elif self.mission_num == 8:
                # 终点1
                if self.mission_pos_cruise(0.5, 1.35, self.ALTITUDE, 3.14, 0.2):
                    self.mission_num = 81
            
            elif self.mission_num == 81:
                if self.current_position_cruise(0.0, 0.0, self.ALTITUDE, 3.14, 0.2):
                    self.mission_num = 82
            
            elif self.mission_num == 82:
                # 最终二维码对准和降落准备
                self.handle_final_landing()
            
            elif self.mission_num == 9:
                # 下降0.2m
                if (self.mission_pos_cruise(self.tmp_x, self.tmp_y, -0.2, 3.14, 0.2) and 
                    self.lib_time_record_func(1.0)):
                    self.mission_num = 10
            
            elif self.mission_num == 10:
                # 自动降落
                self.auto_land()
            
            # 发布setpoint（如果有的话）
            rate.sleep()
    
    def handle_qr_code_mission(self, target_id, next_mission, timeout):
        """处理二维码识别任务"""
        self.target_qr_id = target_id
        
        # 发布目标ID（防止重复发布）
        if self.target_qr_id != self.last_published_target_qr_id:
            target_msg = Int32()
            target_msg.data = self.target_qr_id
            self.target_qr_id_pub.publish(target_msg)
            self.last_published_target_qr_id = self.target_qr_id
            rospy.loginfo(f"发布新的目标二维码ID: {self.target_qr_id}")
        
        # 检查是否识别到目标ID
        if self.current_qr_id == self.target_qr_id and self.qr_land_done_flag:
            if not self.tmp_time_record_start_flag:
                self.tmp_time_record_start_flag = True
                self.lib_pwm_control(100, 100)
                rospy.loginfo(f"识别到目标二维码ID: {self.target_qr_id}，开启激光笔")
                self.tmp_mission_success_time_record = rospy.Time.now()
            
            if (rospy.Time.now() - self.tmp_mission_success_time_record).to_sec() > 1.0:
                rospy.loginfo("关闭激光笔")
                self.lib_pwm_control(0, 0)
                self.tmp_time_record_start_flag = False
                self.qr_land_done_flag = False
                self.mission_num = next_mission
        
        elif self.lib_time_record_func(timeout):
            self.mission_num = next_mission
            rospy.loginfo(f"二维码超时或ID不匹配，当前ID: {self.current_qr_id}, 目标ID: {self.target_qr_id}")
    
    def handle_second_window_detection(self):
        """处理第二次窗口检测"""
        # 重置窗口识别标志位
        if not self.object_data_cleared:
            # 调用C++服务重置标志位
            self.reset_window_flags()
            
            self.window_search_start_time = rospy.Time.now()
            self.object_data_cleared = True
            rospy.loginfo("准备第二次穿窗，重置窗口识别标志位")
        
        # 检查是否检测到窗口
        if (self.object_pos is not None and 
            self.object_pos.header.frame_id == "square"):
            rospy.loginfo("第二次穿窗检测到窗口，准备穿越")
            
            # 检测到窗口，立即开始跟踪和穿越
            if self.object_recognize_track("sakura", 3.14, -1, self.ALTITUDE, 35, 0.2):
                rospy.loginfo("第二次穿窗窗口识别成功，开始穿越")
                self.mission_num = 73
        
        # 超时保护
        if (rospy.Time.now() - self.window_search_start_time).to_sec() > 20.0:
            rospy.logwarn("第二次穿窗识别超时（20秒），强制继续")
            self.mission_num = 10  # 直接降落
    
    def handle_final_landing(self):
        """处理最终降落"""
        self.target_qr_id = 4
        
        # 发布目标ID
        if self.target_qr_id != self.last_published_target_qr_id:
            target_msg = Int32()
            target_msg.data = self.target_qr_id
            self.target_qr_id_pub.publish(target_msg)
            self.last_published_target_qr_id = self.target_qr_id
            rospy.loginfo(f"发布新的目标二维码ID: {self.target_qr_id}")
        
        if self.current_qr_id == self.target_qr_id and self.qr_land_done_flag:
            if self.lib_time_record_func(0.1):
                self.tmp_x = self.local_pos.pose.pose.position.x
                self.tmp_y = self.local_pos.pose.pose.position.y
                self.qr_land_done_flag = False
                self.mission_num = 9
                rospy.loginfo(f"识别到目标二维码ID: {self.target_qr_id}，准备降落")
        
        elif self.lib_time_record_func(5.0):
            self.mission_num = 9
            rospy.loginfo(f"二维码超时或ID不匹配，当前ID: {self.current_qr_id}, 目标ID: {self.target_qr_id}")
    
    def auto_land(self):
        """自动降落"""
        rospy.loginfo("AUTO.LAND")
        try:
            result = self.set_mode_client.call(custom_mode="AUTO.LAND")
            if result.mode_sent:
                rospy.loginfo("自动降落模式已启用")
        except rospy.ServiceException as e:
            rospy.logerr(f"设置降落模式失败: {e}")
    
    def run(self):
        """主运行函数"""
        try:
            # 等待连接
            self.wait_for_connection()
            
            # 起飞前设置
            if not self.pre_flight_setup():
                rospy.logerr("起飞前设置失败")
                return
            
            # 解锁和起飞
            self.arm_and_takeoff()
            
            # 运行状态机
            self.run_state_machine()
            
        except rospy.ROSInterruptException:
            rospy.loginfo("任务被中断")
        except Exception as e:
            rospy.logerr(f"任务执行出错: {e}")
        finally:
            rospy.loginfo("任务状态机退出")

if __name__ == "__main__":
    try:
        mission = MissionStateMachine()
        mission.run()
    except rospy.ROSInterruptException:
        pass 