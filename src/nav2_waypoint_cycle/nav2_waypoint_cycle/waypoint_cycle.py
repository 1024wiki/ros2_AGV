import rclpy,os
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, PointStamped
from visualization_msgs.msg import Marker, MarkerArray
from action_msgs.msg import GoalStatus,GoalStatusArray

import threading
import sys
import tty
import termios

class waypoint_cycle(Node):
    def __init__(self):
        super().__init__('waypoint_cycle')
        # ---------- 路径点数据存储 ----------
        self.markerArray = MarkerArray()          # 箭头 Marker 数组：存每个路径点的位置 + 朝向（用于 RViz 显示）
        self.markerArray_number = MarkerArray()   # 数字 Marker 数组：存每个路径点的序号（0,1,2...，用于 RViz 显示）
        self.count = 0          # 已添加的路径点总数（即 markerArray 里的元素个数，只增不减，除非按 c 清空）
        self.index = 0          # 当前"下一个要去的点"的下标（0 ~ count），导航进度指针
        self.try_again = True   # 失败重试开关：某点导航失败时先重试 1 次（True），再失败就跳过该点（False）

        # ---------- 话题/发布-订阅 ----------
        # 发布 /path_point：把箭头+数字 Marker 数组发给 RViz 显示路径点，RViz通过手动添加订阅话题可以在map中进行显示
        self.mark_pub = self.create_publisher(MarkerArray, '/path_point', 100)
        # 发布 /goal_pose：把导航目标（PoseStamped，frame=map）发给 bt_navigator，驱动机器人去这个点
        self.goal_pub = self.create_publisher(PoseStamped, '/goal_pose', 10)
        # 订阅 /clicked_point：接收 RViz "2D Nav Goal" 点出来的坐标（PointStamped），每点一下加一个路径点
        self.create_subscription(PointStamped, '/clicked_point', self.click_callback, 10)
        # 订阅 navigate_to_pose/_action/status 话题：每次 bt_navigator 在导航一个目标点成功/失败/取消时，会向该话题GoalStatusArray添加一个导航结果GoalStatus
        # 订阅该话题，可以得知当前导航目标是否成功/失败，从而决定发下一个点还是重试（这是"循环巡逻"的核心反馈信号）bt_navigator 就
        self.navigation_goal_status_sub_ = self.create_subscription(GoalStatusArray,"navigate_to_pose/_action/status",self.pose_callback,10)
        # 启动键盘监听线程
        self.keyboard_thread = threading.Thread(target=self.keyboard_listener, daemon=True)
        self.keyboard_thread.start()
    
    # 键盘监听线程
    def keyboard_listener(self):
        if not os.isatty(sys.stdin.fileno()):
            sys.stdin = open('/dev/tty','r')
        fd = sys.stdin.fileno()
        old_settings = termios.tcgetattr(fd)
        old_settings[3] =old_settings[3] |termios.ECHO
        tty.setcbreak(fd)
        os.system("stty -echo")
        try:
            # 在正常运行时，持续获取键盘输入，输入c字符时，清除所有路径点
            while rclpy.ok():
                key = sys.stdin.read(1)
                if key == 'c':
                    self.clear_markers()
        finally:
            os.system("stty echo")
            print("termino recoverd.............")
            termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)

    # 清除所有路径点
    def clear_markers(self):
        self.get_logger().info("Clearing all waypoints...")
        self.markerArray.markers.clear()
        self.markerArray_number.markers.clear()
        self.count = 0
        self.index = 0
        self.try_again = True

        self.mark_pub.publish(self.markerArray)
        self.mark_pub.publish(self.markerArray_number)
        self.get_logger().info("Waypoints cleared.")

    # ======================================================================
    # pose_callback：导航状态回调（整个"循环巡逻"的决策核心）
    #
    # 触发时机：bt_navigator 每次处理完一个目标后，
    #          会向 /navigate_to_pose/_action/status 发布 GoalStatusArray，
    #          本函数收到消息后根据【最新一条】状态做决策。
    #
    # 状态枚举（来自 action_msgs/GoalStatus）：
    #   STATUS_UNKNOWN=0  STATUS_ACCEPTED=1  STATUS_EXECUTING=2（执行中）
    #   STATUS_CANCELED=3 STATUS_SUCCEEDED=4（成功） STATUS_ABORTED=5（失败）
    # ----------------------------------------------------------------------
    # 逻辑总览：
    #   状态不是"执行中"时（说明上一个目标已有结果了）：
    #     ├─ 成功(>=4) 且 有点(count>0)：发下一个点；走完最后一个就回绕到 0 循环
    #     └─ 非成功(3/5)且有点：先重试 1 次（try_again），再失败就跳过该点
    # ======================================================================
    def pose_callback(self, msg):
        # status_list[-1]：取最新一条目标状态（[-1] = 列表最后一个）
        # 状态为 EXECUTING(执行中) 时直接忽略，等它出结果再处理
        if msg.status_list[-1].status != GoalStatus.STATUS_EXECUTING:
            # ============ 分支一：上一个目标【成功】了 ============
            # status >= STATUS_SUCCEEDED(4)，即成功(4)或之后的状态；
            if msg.status_list[-1].status >= GoalStatus.STATUS_SUCCEEDED and self.count > 0:
                # 说明已经成功到达该导航目标，此时可以启动相机传感器进行巡检拍照等操作
                # ....,，其他业务逻辑

                # 准备下个导航点进行发布到/goal_pose话题，等待nav2接收后进行导航
                self.try_again = True   # 成功到达，重置重试开关（下次失败还能再试一次）
                # --- 情况 1：index == count，说明全部点都走完了 ---
                if self.index == self.count:
                    self.print_marker_info(self.index-1)   # 打印最后到达的点的坐标
                    if self.count > 1:
                        self.get_logger().info('Complete instructions!')  # 提示一圈完成
                    self.index = 0           # ★ 关键：下标回绕到 0，实现无限循环巡逻
                    self.publish_pose(self.index)   # 发布第 0 个点（重新出发）的导航目标
                    self.index += 1          # 指针指向下一个
                # --- 情况 2：还没走完，发下一个点 ---
                elif self.index < self.count:
                    self.get_logger().info(f'Reach the target point {self.index-1}:')
                    self.print_marker_info(self.index-1)
                    self.publish_pose(self.index)    # 发第 index 个点
                    self.index += 1                  # 指针后移
            # ============ 分支二：上一个目标【失败/被取消】了 ============
            elif self.count > 0:
                self.get_logger().warn(f'Can not reach the target point {self.index-1}:')
                self.print_marker_info(self.index-1)
                # --- 第 1 次失败：重试当前点 ---
                if self.try_again:
                    self.get_logger().warn(f'trying reach the target point {self.index-1} again!')
                    self.publish_pose(self.index - 1)   # 重新发布同一个点，重试一次
                    self.try_again = False              # 关闭重试开关（只重试一次）
                # --- 第 2 次失败：放弃当前点，跳到下一个 ---
                elif self.index < len(self.markerArray.markers):
                    self.get_logger().warn(f'try reach the target point {self.index-1} failed! reach next point:')
                    self.print_marker_info(self.index-1)
                    if self.index == self.count:        # 若当前是最后一个点，回绕到 0
                        self.index = 0
                    self.publish_pose(self.index)       # 发下一个点
                    self.index += 1
                    self.try_again = True               # 重置重试开关

    # 添加一个数字和箭头的导航点，并发布到话题/path_point等待RViz订阅该话题在map显示；同时向/goal_pose话题发布一个导航点，bt_naviation订阅去导航
    def click_callback(self, msg):
        self.add_marker(msg.point.x, msg.point.y, 0.0, 1.0)
        self.add_marker_number(msg.point.x, msg.point.y, 0.0, 1.0)
        # 添加一个数字和箭头的导航点，并发布到话题/path_point等待RViz订阅该话题在map显示
        self.mark_pub.publish(self.markerArray)
        self.mark_pub.publish(self.markerArray_number)
        if self.count == 0:
            self.publish_pose(self.count)   # 向/goal_pose话题发布一个导航点
            self.index += 1
        self.count += 1

    # 添加一个箭头消息到self.markerArray
    def add_marker(self, x, y, z, w):
        marker = Marker()
        marker.header.frame_id = 'map'
        marker.ns = 'arrow'
        marker.id = self.count
        marker.type = Marker.ARROW
        marker.action = Marker.ADD
        marker.scale.x = 0.5
        marker.scale.y = 0.05
        marker.scale.z = 0.05
        marker.color.a = 1.0
        marker.color.r = 1.0
        marker.pose.position.x = x
        marker.pose.position.y = y
        marker.pose.position.z = 0.1
        marker.pose.orientation.z = z
        marker.pose.orientation.w = w
        self.markerArray.markers.append(marker)

    # 添加一个数字消息到self.markerArray_number
    def add_marker_number(self, x, y, z, w):
        marker_number = Marker()
        marker_number.header.frame_id = 'map'
        marker_number.ns = 'number'
        marker_number.id = self.count
        marker_number.type = Marker.TEXT_VIEW_FACING
        marker_number.action = Marker.ADD
        marker_number.scale.x = 0.5
        marker_number.scale.y = 0.5
        marker_number.scale.z = 0.5
        marker_number.color.a = 1.0
        marker_number.color.r = 1.0
        marker_number.pose.position.x = x
        marker_number.pose.position.y = y
        marker_number.pose.position.z = 0.1
        marker_number.pose.orientation.z = z
        marker_number.pose.orientation.w = w
        marker_number.text = str(self.count)
        self.markerArray_number.markers.append(marker_number)

    # 向/goal_pose话题中发布一个导航目标（PoseStamped，frame=map）发给 bt_navigator 进行订阅导航
    def publish_pose(self, index):
        pose = PoseStamped()
        pose.header.frame_id = 'map'
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x = self.markerArray.markers[index].pose.position.x
        pose.pose.position.y = self.markerArray.markers[index].pose.position.y
        pose.pose.orientation.z = self.markerArray.markers[index].pose.orientation.z
        pose.pose.orientation.w = self.markerArray.markers[index].pose.orientation.w
        self.goal_pub.publish(pose)

    def print_marker_info(self, index):
        self.get_logger().info(
            f"x: {self.markerArray.markers[index].pose.position.x}, "
            f"y: {self.markerArray.markers[index].pose.position.y}, "
            f"z: {self.markerArray.markers[index].pose.orientation.z}, "
            f"w: {self.markerArray.markers[index].pose.orientation.w}"
        )

def main(args=None):
    rclpy.init(args=args)
    node = waypoint_cycle()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Shutting down')
    finally:
        old_settings = termios.tcgetattr(sys.stdin.fileno())
        old_settings[3] =old_settings[3] |termios.ECHO
        termios.tcsetattr(sys.stdin.fileno(), termios.TCSADRAIN, old_settings)
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

