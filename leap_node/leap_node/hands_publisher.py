import rclpy
from rclpy.node import Node
from leap_msgs.msg import Hands, Hand
import leap


class LeapMotionListener(leap.Listener):
    '''
    Leap Motionのイベント処理を行うクラス
    '''
    def __init__(self, callback):
        super().__init__()
        self.callback = callback  # コールバック関数を保持

    def on_tracking_event(self, event):

        print(f"Frame {event.tracking_frame_id} with {len(event.hands)} hands.")
        for hand in event.hands:
            hand_type = "left" if str(hand.type) == "HandType.Left" else "right"
            print(
                f"Hand id {hand.id} is a {hand_type} hand with position ({hand.palm.position.x}, {hand.palm.position.y}, {hand.palm.position.z})."
            )

            for digit_name, digit in zip(["thumb", "index", "middle", "ring", "pinky"], hand.digits):
                print(f"  {digit_name.capitalize()}:")
                for bone_name, bone in zip(["metacarpal", "proximal", "intermediate", "distal"], digit.bones):
                    print(
                        f"    {bone_name.capitalize()} joint positions: Prev ({bone.prev_joint.x}, {bone.prev_joint.y}, {bone.prev_joint.z}), Next ({bone.next_joint.x}, {bone.next_joint.y}, {bone.next_joint.z})"
                    )

        self.callback(event.hands)  # 検出された手をコールバック関数に渡す
    
    def on_device_event(self, event):
        try:
            with event.device.open():
                info = event.device.get_info()
        except leap.LeapCannotOpenDeviceError:
            info = event.device.get_info()

        print(f"Found device {info.serial}")


class HandsPublisher(Node):
    '''
    Leap Motionから手のすべての情報を取得し、ROS2トピックにパブリッシュするクラス
    '''
    def __init__(self):
        super().__init__('all_detector')
        self.hands_publsiher = self.create_publisher(Hands, 'leap_hands', 10)
        self.connection = leap.Connection()
        self.connection.add_listener(LeapMotionListener(self.handle_hand_data))

    def handle_hand_data(self, hands):
        hands_msg = Hands()
        hands_msg.header.stamp = self.get_clock().now().to_msg()
        hands_msg.header.frame_id = "leap_link"

        for hand in hands:
            msg = Hand()
            msg.type = "Left" if str(hand.type) == "HandType.Left" else "Right"
            msg.position.x = -hand.palm.position.z * 1.0e-3
            msg.position.y = -hand.palm.position.x * 1.0e-3
            msg.position.z = hand.palm.position.y * 1.0e-3
            msg.velocity.x = -hand.palm.velocity.z * 1.0e-3
            msg.velocity.y = -hand.palm.velocity.x * 1.0e-3
            msg.velocity.z = hand.palm.velocity.y * 1.0e-3
            msg.normal.x = -hand.palm.normal.z
            msg.normal.y = -hand.palm.normal.x
            msg.normal.z = hand.palm.normal.y
            msg.orientation.x = -hand.palm.orientation.z
            msg.orientation.y = -hand.palm.orientation.x
            msg.orientation.z = hand.palm.orientation.y
            msg.orientation.w = hand.palm.orientation.w
            msg.grab_strength = hand.grab_strength
            msg.pinch_strength = hand.pinch_strength
            hands_msg.hands.append(msg)

        self.hands_publsiher.publish(hands_msg)

    def run(self):
        with self.connection.open():
            self.connection.set_tracking_mode(leap.TrackingMode.Desktop)
            rclpy.spin(self)

def main(args=None):
    rclpy.init()
    hands_publisher = HandsPublisher()
    hands_publisher.run()
    hands_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()