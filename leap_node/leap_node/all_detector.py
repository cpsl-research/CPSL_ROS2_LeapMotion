import rclpy
from rclpy.node import Node
from leap_msgs.msg import Hand, Finger, Bone, Arm  # カスタムメッセージ型をインポート
import leap
import time
import curses

class AllDetector(Node):
    '''
    Leap Motionから手のすべての情報を取得し、ROS2トピックにパブリッシュするクラス
    '''
    def __init__(self, stdscr):
        super().__init__('all_detector')
        self.left_hand_publisher = self.create_publisher(Hand, 'left_hand', 10)
        self.right_hand_publisher = self.create_publisher(Hand, 'right_hand', 10)
        self.stdscr = stdscr
        self.listener = LeapMotionListener(self.handle_hand_data)
        self.connection = leap.Connection()
        self.connection.add_listener(self.listener)

    def handle_hand_data(self, hands):
        # 検出された手を処理
        self.stdscr.clear()
        left_hand_detected = False
        right_hand_detected = False

        for hand in hands:
            msg = Hand()
            msg.type = "Left" if str(hand.type) == "HandType.Left" else "Right"
            msg.position.x = hand.palm.position.x
            msg.position.y = -hand.palm.position.z
            msg.position.z = hand.palm.position.y
            msg.velocity.x = hand.palm.velocity.x
            msg.velocity.y = -hand.palm.velocity.z
            msg.velocity.z = hand.palm.velocity.y
            msg.normal.x = hand.palm.normal.x
            msg.normal.y = -hand.palm.normal.z
            msg.normal.z = hand.palm.normal.y
            msg.orientation.x = hand.palm.orientation.x
            msg.orientation.y = -hand.palm.orientation.z
            msg.orientation.z = hand.palm.orientation.y
            msg.orientation.w = hand.palm.orientation.w
            msg.grab_strength = hand.grab_strength
            msg.pinch_strength = hand.pinch_strength

            # # 指の情報を追加
            # for finger in hand.fingers:
            #     finger_msg = Finger()
            #     finger_msg.id = finger.id
            #     finger_msg.type = finger.type
            #     finger_msg.tip_position.x = finger.tip_position.x
            #     finger_msg.tip_position.y = -finger.tip_position.z
            #     finger_msg.tip_position.z = finger.tip_position.y
            #     finger_msg.direction.x = finger.direction.x
            #     finger_msg.direction.y = -finger.direction.z
            #     finger_msg.direction.z = finger.direction.y
            #     finger_msg.velocity.x = finger.velocity.x
            #     finger_msg.velocity.y = -finger.velocity.z
            #     finger_msg.velocity.z = finger.velocity.y
            #     finger_msg.length = finger.length
            #     finger_msg.width = finger.width

            #     # 骨の情報を追加
            #     for bone in finger.bones:
            #         bone_msg = Bone()
            #         bone_msg.type = bone.type
            #         bone_msg.prev_joint.x = bone.prev_joint.x
            #         bone_msg.prev_joint.y = -bone.prev_joint.z
            #         bone_msg.prev_joint.z = bone.prev_joint.y
            #         bone_msg.next_joint.x = bone.next_joint.x
            #         bone_msg.next_joint.y = -bone.next_joint.z
            #         bone_msg.next_joint.z = bone.next_joint.y
            #         bone_msg.direction.x = bone.direction.x
            #         bone_msg.direction.y = -bone.direction.z
            #         bone_msg.direction.z = bone.direction.y
            #         bone_msg.length = bone.length
            #         bone_msg.width = bone.width
            #         finger_msg.bones.append(bone_msg)

            #     msg.fingers.append(finger_msg)

            # # 腕の情報を追加
            # arm_msg = Arm()
            # arm_msg.elbow_position.x = hand.arm.elbow_position.x
            # arm_msg.elbow_position.y = -hand.arm.elbow_position.z
            # arm_msg.elbow_position.z = hand.arm.elbow_position.y
            # arm_msg.wrist_position.x = hand.arm.wrist_position.x
            # arm_msg.wrist_position.y = -hand.arm.wrist_position.z
            # arm_msg.wrist_position.z = hand.arm.wrist_position.y
            # arm_msg.direction.x = hand.arm.direction.x
            # arm_msg.direction.y = -hand.arm.direction.z
            # arm_msg.direction.z = hand.arm.direction.y
            # arm_msg.length = hand.arm.length
            # arm_msg.width = hand.arm.width
            # msg.arm = arm_msg

            # トピックにパブリッシュ
            if str(hand.type) == "HandType.Left":
                left_hand_detected = True
                self.left_hand_publisher.publish(msg)
                # self.display_hand_data("Left Hand", msg)
            elif str(hand.type) == "HandType.Right":
                right_hand_detected = True
                self.right_hand_publisher.publish(msg)
                # self.display_hand_data("Right Hand", msg)

        # 未検出時の処理
        if not left_hand_detected:
            self.left_hand_publisher.publish(Hand())
            # self.stdscr.addstr("Left hand is not detected\n")
        if not right_hand_detected:
            self.right_hand_publisher.publish(Hand())
            # self.stdscr.addstr("Right hand is not detected\n")

        self.stdscr.refresh()

    def display_hand_data(self, hand_label, msg):
        # 手の情報を詳細に表示
        self.stdscr.addstr(f"{hand_label}:\n")
        self.stdscr.addstr(f"  Position: x={msg.position.x:.2f}, y={msg.position.y:.2f}, z={msg.position.z:.2f}\n")
        self.stdscr.addstr(f"  Velocity: x={msg.velocity.x:.2f}, y={msg.velocity.y:.2f}, z={msg.velocity.z:.2f}\n")
        self.stdscr.addstr(f"  Normal: x={msg.normal.x:.2f}, y={msg.normal.y:.2f}, z={msg.normal.z:.2f}\n")
        self.stdscr.addstr(f"  Orientation: x={msg.orientation.x:.2f}, y={msg.orientation.y:.2f}, z={msg.orientation.z:.2f}, w={msg.orientation.w:.2f}\n")
        self.stdscr.addstr(f"  Grab Strength: {msg.grab_strength:.2f}\n")
        self.stdscr.addstr(f"  Pinch Strength: {msg.pinch_strength:.2f}\n\n")

    def run(self):
        with self.connection.open():
            self.connection.set_tracking_mode(leap.TrackingMode.Desktop)
            while rclpy.ok():
                rclpy.spin_once(self)
                time.sleep(1)

class LeapMotionListener(leap.Listener):
    '''
    Leap Motionのイベント処理を行うクラス
    '''
    def __init__(self, callback):
        super().__init__()
        self.callback = callback  # コールバック関数を保持

    def on_tracking_event(self, event):
        self.callback(event.hands)  # 検出された手をコールバック関数に渡す

def curses_main(stdscr):
    rclpy.init()
    node = AllDetector(stdscr)
    node.run()
    rclpy.shutdown()

def main(args=None):
    curses.wrapper(curses_main)

if __name__ == '__main__':
    main()