import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped,Pose
from tm_msgs.srv import SetPositions
from main_control_if.srv import ArmMove
from scipy.spatial.transform import Rotation as R
import numpy as np


class MovitGrab(Node):
    def __init__(self):
        super().__init__("moveit_grab")
        self.move_step_srv = self.create_service(ArmMove,'/move_step',self.moving_step)
        self.position_pub_srv = self.create_client(SetPositions,'/set_positions')
        # self.arm_ee_pos_srv = self.create_service(PoseStamped,'/tool_pose',self.ee_pose)
        self.ee_pose_sub = self.create_subscription(PoseStamped,'/tool_pose',self.ee_pose_callback,10)
        self.get_point = None
        self.get_logger().info("Moveit grab")
        while not self.position_pub_srv.wait_for_service(timeout_sec=1.0):
            self.get_logger().warn("Waiting for 'set_position' service...")
        self.get_logger().info("service init success")


        #get the current arm ee position
    def ee_pose_callback(self,msg:PoseStamped):

        #get the ee pose
        self.ee_pose = msg.pose

        #pub the receive pose and orientation
        


    def moving_step(self, request: ArmMove.Request, response: ArmMove.Response):
        if self.ee_pose is None:
            self.get_logger().warn("Don't have current EE pose")
            response.ok = False
            return response

        #current ee pose
        ee_pos = self.ee_pose.position
        ee_ori = self.ee_pose.orientation
        R_ee = R.from_quat([ee_ori.x, ee_ori.y, ee_ori.z, ee_ori.w])
        # ee_euler = R_ee.as_euler('xyz', degrees=False)

        #moveing step
        offset_pos = request.pose.position
        offset_ori = request.pose.orientation
        
        R_offset = R.from_quat([offset_ori.x, offset_ori.y, offset_ori.z, offset_ori.w])
        # offset_euler = R_offset.as_euler('xyz', degrees=False)


        #calculate position offset 
        final_pose = [
            ee_pos.x + offset_pos.y,
            ee_pos.y + offset_pos.x,
            ee_pos.z + offset_pos.z,
        ]

        #calculate orientation offset
        R_final = R_ee * R_offset        

        final_euler = R_final.as_euler('xyz',degrees= True)
        self.get_logger().info(f"Receive EE pose x = {self.ee_pose.position.x:.3f},y = {self.ee_pose.position.y:3f},z = {self.ee_pose.position.z:3f}")
        self.get_logger().info(f"Receive EE orientation x = {self.ee_pose.orientation.x},y = {self.ee_pose.orientation.y}, z={self.ee_pose.orientation.z},w={self.ee_pose.orientation.w}")


        self.get_logger().info(f"Final EE pose x = {final_pose[0]:.3f},y = {final_pose[1]:3f},z = {final_pose[2]:3f}")
        self.get_logger().info(f"Final EE Euler: Rx={final_euler[0]:.3f}, Ry={final_euler[1]:.3f}, Rz={final_euler[2]:.3f}")
        
        #set the service srv
        req = SetPositions.Request()
        req.motion_type = 8
        req.positions = final_pose + list(final_euler)
        req.velocity = 0.2
        req.acc_time = 200.0
        req.blend_percentage = 0
        req.fine_goal = True

        future = self.position_pub_srv.call_async(req)

        def done_cb(fut):
            result = fut.result()
            if result and result.ok:
                self.get_logger().info("Motion succeeded")
                response.ok = True
            else:
                self.get_logger().error("Motion failed")
                response.ok = False

        future.add_done_callback(done_cb)
        return response


def main(args=None):
    rclpy.init(args=args)
    node = MovitGrab()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()