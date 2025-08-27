import rclpy
from rclpy.node import Node
from leap_msgs.msg import Hands, Hand
import leap
import numpy as np
from sensor_msgs.msg import PointCloud2,PointField
from std_msgs.msg import Header
import sensor_msgs_py.point_cloud2 as pc2


class LeapMotionListener(leap.Listener):

    def __init__(self, callback):
        super().__init__()
        self.callback = callback

    def on_tracking_event(self, event):

        if len(event.hands) > 0:

            #get the hands array
            hands = event.hands

            left_hand_joints = np.zeros((26, 3))
            right_hand_joints = np.zeros((26, 3))

            for hand in hands:
                hand_type = "left" if str(hand.type) == "HandType.Left" else "right"

                palm_orientation = np.array([
                    hand.palm.orientation.x,
                    hand.palm.orientation.y,
                    hand.palm.orientation.z,
                    hand.palm.orientation.w
                ])


                joints = np.array([
                    #Palm position
                    [hand.palm.position.x, hand.palm.position.y, hand.palm.position.z],
                    # Thumb joints
                    [hand.thumb.metacarpal.prev_joint.x, hand.thumb.metacarpal.prev_joint.y, hand.thumb.metacarpal.prev_joint.z],
                    [hand.thumb.metacarpal.next_joint.x, hand.thumb.metacarpal.next_joint.y, hand.thumb.metacarpal.next_joint.z],
                    [hand.thumb.proximal.next_joint.x, hand.thumb.proximal.next_joint.y, hand.thumb.proximal.next_joint.z],
                    [hand.thumb.intermediate.next_joint.x, hand.thumb.intermediate.next_joint.y, hand.thumb.intermediate.next_joint.z],
                    [hand.thumb.distal.next_joint.x, hand.thumb.distal.next_joint.y, hand.thumb.distal.next_joint.z],
                    # Index joints
                    [hand.index.metacarpal.prev_joint.x, hand.index.metacarpal.prev_joint.y, hand.index.metacarpal.prev_joint.z],
                    [hand.index.metacarpal.next_joint.x, hand.index.metacarpal.next_joint.y, hand.index.metacarpal.next_joint.z],
                    [hand.index.proximal.next_joint.x, hand.index.proximal.next_joint.y, hand.index.proximal.next_joint.z],
                    [hand.index.intermediate.next_joint.x, hand.index.intermediate.next_joint.y, hand.index.intermediate.next_joint.z],
                    [hand.index.distal.next_joint.x, hand.index.distal.next_joint.y, hand.index.distal.next_joint.z],
                    # Middle joints
                    [hand.middle.metacarpal.prev_joint.x, hand.middle.metacarpal.prev_joint.y, hand.middle.metacarpal.prev_joint.z],
                    [hand.middle.metacarpal.next_joint.x, hand.middle.metacarpal.next_joint.y, hand.middle.metacarpal.next_joint.z],
                    [hand.middle.proximal.next_joint.x, hand.middle.proximal.next_joint.y, hand.middle.proximal.next_joint.z],
                    [hand.middle.intermediate.next_joint.x, hand.middle.intermediate.next_joint.y, hand.middle.intermediate.next_joint.z],
                    [hand.middle.distal.next_joint.x, hand.middle.distal.next_joint.y, hand.middle.distal.next_joint.z],
                    # Ring joints
                    [hand.ring.metacarpal.prev_joint.x, hand.ring.metacarpal.prev_joint.y, hand.ring.metacarpal.prev_joint.z],
                    [hand.ring.metacarpal.next_joint.x, hand.ring.metacarpal.next_joint.y, hand.ring.metacarpal.next_joint.z],
                    [hand.ring.proximal.next_joint.x, hand.ring.proximal.next_joint.y, hand.ring.proximal.next_joint.z],
                    [hand.ring.intermediate.next_joint.x, hand.ring.intermediate.next_joint.y, hand.ring.intermediate.next_joint.z],
                    [hand.ring.distal.next_joint.x, hand.ring.distal.next_joint.y, hand.ring.distal.next_joint.z],
                    # Pinky joints
                    [hand.pinky.metacarpal.prev_joint.x, hand.pinky.metacarpal.prev_joint.y, hand.pinky.metacarpal.prev_joint.z],
                    [hand.pinky.metacarpal.next_joint.x, hand.pinky.metacarpal.next_joint.y, hand.pinky.metacarpal.next_joint.z],
                    [hand.pinky.proximal.next_joint.x, hand.pinky.proximal.next_joint.y, hand.pinky.proximal.next_joint.z],
                    [hand.pinky.intermediate.next_joint.x, hand.pinky.intermediate.next_joint.y, hand.pinky.intermediate.next_joint.z],
                    [hand.pinky.distal.next_joint.x, hand.pinky.distal.next_joint.y, hand.pinky.distal.next_joint.z],
                ])

                #convert to meters
                joints = joints * 1.0e-3

                if hand_type == "left":
                    left_hand_joints = joints
                else:
                    right_hand_joints = joints
            
            ret_array = [left_hand_joints, right_hand_joints]

            return self.callback(ret_array)

        else:
            return
    
    def on_device_event(self, event):
        try:
            with event.device.open():
                info = event.device.get_info()
        except leap.LeapCannotOpenDeviceError:
            info = event.device.get_info()

        print(f"Found device {info.serial}")


class JointPublisher(Node):
    def __init__(self):

        super().__init__('all_detector')
        self.left_hand_publisher = self.create_publisher(PointCloud2, 'left_hand_joints', 10)
        self.right_hand_publisher = self.create_publisher(PointCloud2, 'right_hand_joints', 10)

        #setup leap motion connector object
        self.connection = leap.Connection()
        self.connection.add_listener(LeapMotionListener(self.handle_hand_data))

        #setup the point fields
        self.pc_fields = self.pc_fields = [
            PointField(name='x', offset=0,  datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4,  datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8,  datatype=PointField.FLOAT32, count=1)
        ]

    def handle_hand_data(self, joints:list):

        left_hand_joints = joints[0]
        right_hand_joints = joints[1]

        header = Header()
        header.stamp = self.get_clock().now().to_msg()
        header.frame_id = "cpsl_human_movement/leap_link"

        left_hand_msg = self.np_to_pointcloud2(left_hand_joints,header,self.pc_fields)
        right_hand_msg = self.np_to_pointcloud2(right_hand_joints,header,self.pc_fields)

        self.left_hand_publisher.publish(left_hand_msg)
        self.right_hand_publisher.publish(right_hand_msg)

        return

    def run(self):
        with self.connection.open():
            self.connection.set_tracking_mode(leap.TrackingMode.Desktop)
            rclpy.spin(self)
    

    def np_to_pointcloud2(self,points:np.ndarray,header:Header,fields:list)->PointCloud2:
        """Convert a numpy array into a pointcloud2 object in the base_frame

        Args:
            points (np.ndarray): array of points to convert to a numpy array

        Returns:
            PointCloud2: PointCloud2 object of points from the numpy array
        """
        msg = pc2.create_cloud(
            header=header,
            fields=fields,
            points=points
        )

        return msg

def main(args=None):
    rclpy.init()
    joint_publisher = JointPublisher()
    joint_publisher.run()
    joint_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()