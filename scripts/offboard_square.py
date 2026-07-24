#!/usr/bin/env python3
import rospy
import math
from geometry_msgs.msg import PoseStamped
from mavros_msgs.msg import State
from mavros_msgs.srv import CommandBool, SetMode, CommandLong

class SquareFlight:
    def __init__(self):
        self.fc_state = State()
        self.cur_pos = PoseStamped()

        rospy.Subscriber("/mavros/state", State, self.state_callback)
        rospy.Subscriber("/mavros/local_position/pose", PoseStamped, self.pos_callback)

        self.target_pub = rospy.Publisher("/mavros/setpoint_position/local", PoseStamped, queue_size=10)

        self.arm_service = rospy.ServiceProxy("/mavros/cmd/arming", CommandBool)
        self.mode_service = rospy.ServiceProxy("/mavros/set_mode", SetMode)
        self.command_service = rospy.ServiceProxy("/mavros/cmd/command", CommandLong)

        self.rate = rospy.Rate(20)

    def state_callback(self, msg):
        self.fc_state = msg

    def pos_callback(self, msg):
        self.cur_pos = msg

    def get_distance(self, target_point):
        dx = target_point.pose.position.x - self.cur_pos.pose.position.x
        dy = target_point.pose.position.y - self.cur_pos.pose.position.y
        dz = target_point.pose.position.z - self.cur_pos.pose.position.z
        return math.sqrt(dx**2 + dy**2 + dz**2)

    def wait_reach(self, target_pos, err_limit=0.15, hover_time=0.0, timeout=30):
        start_time = rospy.Time.now()
        reached = False
        hover_start = rospy.Time.now()

        while not rospy.is_shutdown():
            target_pos.header.stamp = rospy.Time.now()
            self.target_pub.publish(target_pos)

            dist = self.get_distance(target_pos)

            if dist < err_limit and not reached:
                rospy.loginfo("抵达航点 (距离: %.2f 米)，悬停 %.1f 秒", dist, hover_time)
                reached = True
                hover_start = rospy.Time.now()

            if reached:
                elapsed = (rospy.Time.now() - hover_start).to_sec()
                if elapsed >= hover_time:
                    rospy.loginfo("悬停完成 (%.1f 秒)", elapsed)
                    return True
            else:
                if (rospy.Time.now() - start_time).to_sec() > timeout:
                    rospy.logwarn("到达航点超时！当前距离: %.2f 米", dist)
                    return False

            self.rate.sleep()

        return False

    def flight_task(self):
        while not self.fc_state.connected:
            rospy.loginfo("等待MAVROS连接飞控...")
            self.rate.sleep()
        rospy.loginfo("飞控连接成功！")

        # ====== 航点定义 ======
        takeoff = PoseStamped()
        takeoff.header.frame_id = "map"
        takeoff.pose.position.x = 0.0
        takeoff.pose.position.y = 0.0
        takeoff.pose.position.z = 2.0

        wp1 = PoseStamped()
        wp1.header.frame_id = "map"
        wp1.pose.position.x = 3.0
        wp1.pose.position.y = 0.0
        wp1.pose.position.z = 2.0

        wp2 = PoseStamped()
        wp2.header.frame_id = "map"
        wp2.pose.position.x = 3.0
        wp2.pose.position.y = 3.0
        wp2.pose.position.z = 2.0

        wp3 = PoseStamped()
        wp3.header.frame_id = "map"
        wp3.pose.position.x = 0.0
        wp3.pose.position.y = 3.0
        wp3.pose.position.z = 2.0

        wp4 = PoseStamped()
        wp4.header.frame_id = "map"
        wp4.pose.position.x = 0.0
        wp4.pose.position.y = 0.0
        wp4.pose.position.z = 2.0

        # ====== 预推送 ======
        rospy.loginfo("预推送目标点...")
        for _ in range(40):
            takeoff.header.stamp = rospy.Time.now()
            self.target_pub.publish(takeoff)
            self.rate.sleep()

        # ====== OFFBOARD 模式 ======
        rospy.loginfo("切换OFFBOARD模式...")
        while self.fc_state.mode != "OFFBOARD" and not rospy.is_shutdown():
            try:
                self.mode_service(custom_mode="OFFBOARD")
            except rospy.ServiceException as e:
                rospy.logwarn("模式切换失败: %s", e)
            takeoff.header.stamp = rospy.Time.now()
            self.target_pub.publish(takeoff)
            self.rate.sleep()
        rospy.loginfo("OFFBOARD模式切换完成")

        # ====== 解锁 ======
        rospy.loginfo("解锁无人机...")
        while not self.fc_state.armed and not rospy.is_shutdown():
            try:
                result = self.arm_service(True)
                if result.success:
                    break
            except rospy.ServiceException as e:
                rospy.logwarn("解锁失败: %s", e)
            takeoff.header.stamp = rospy.Time.now()
            self.target_pub.publish(takeoff)
            self.rate.sleep()
        rospy.loginfo("无人机解锁成功！")

        # ====== 起飞 ======
        rospy.loginfo("起飞至2米高度（到达后悬停5秒）")
        if not self.wait_reach(takeoff, err_limit=0.15, hover_time=5.0):
            rospy.logerr("起飞失败！")
            return

        # ====== 正方形航线 ======
        rospy.loginfo("飞往正方形第1个顶点 (3,0,2)")
        self.wait_reach(wp1, err_limit=0.3, hover_time=0.5)

        rospy.loginfo("飞往正方形第2个顶点 (3,3,2)")
        self.wait_reach(wp2, err_limit=0.3, hover_time=0.5)

        rospy.loginfo("飞往正方形第3个顶点 (0,3,2)")
        self.wait_reach(wp3, err_limit=0.3, hover_time=0.5)

        rospy.loginfo("飞回原点 (0,0,2)")
        self.wait_reach(wp4, err_limit=0.3, hover_time=0.5)

        # ====== 发送降落命令 (MAV_CMD_NAV_LAND = 21) ======
        rospy.loginfo("发送降落命令...")
        try:
            self.command_service(command=21, param1=0.0, param2=0.0, param3=0.0,
                                param4=0.0, param5=0.0, param6=0.0, param7=0.0)
            rospy.loginfo("降落命令已发送，等待降落...")
        except rospy.ServiceException as e:
            rospy.logwarn("降落命令发送失败: %s", e)

        # 等待降落完成
        rospy.sleep(15.0)

        # ====== 尝试上锁 ======
        rospy.loginfo("尝试上锁...")
        try:
            result = self.arm_service(False)
            if result.success:
                rospy.loginfo("飞机已上锁！")
            else:
                rospy.logwarn("上锁失败，稍后重试...")
                rospy.sleep(2.0)
                result = self.arm_service(False)
                if result.success:
                    rospy.loginfo("飞机已上锁！")
                else:
                    rospy.logwarn("上锁仍然失败，请手动上锁")
        except rospy.ServiceException as e:
            rospy.logwarn("上锁服务异常: %s", e)

        rospy.loginfo("全部飞行任务执行完毕！")

if __name__ == "__main__":
    rospy.init_node("offboard_square_node")
    flight = SquareFlight()
    try:
        flight.flight_task()
    except rospy.ROSInterruptException:
        rospy.loginfo("检测到终止指令，安全退出")
