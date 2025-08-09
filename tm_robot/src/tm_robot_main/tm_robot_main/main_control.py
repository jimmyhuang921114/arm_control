import rclpy
from rclpy.node import Node
import cv2
import numpy as np
from sensor_msgs.msg import Image
from std_msgs.msg import Header, Bool, Int32
from std_srvs.srv import Trigger
from tm_robot_if.srv import GroundedSAM2Interface,PoseSrv,CaptureImage,SecondCheck,DrugIdentify,SecondCamera,Paddle, TMFlowMode, MedicineOrder
from tm_rail_interface.srv import RailControl
from tm_msgs.msg import SctResponse
from tm_msgs.srv import SendScript, SetPositions
from cv_bridge import CvBridge 
import os
import yaml
from geometry_msgs.msg import Pose, Point, Quaternion
import time
from scipy.spatial.transform import Rotation as R
import threading

WORKFLOW_PATH= "/workspace/tm_robot/src/tm_robot_main/tm_robot_main/workflow.yaml"
ARM_CONTROL_POINT = ""
COMBINE_PIC_PATH = "/workspace/tm_robot/src/visuial/sample_picture/combined.jpg"
MEDICINE_INFO_PATH = "/workspace/tm_robot/src/visuial/visuial/medicine_info2.yaml"
ORDER_FOLDER = ""
NAMED_POSE_YAML_PATH = '/workspace/tm_robot/src/tm_robot_main/tm_robot_main/robot_base_point.yaml'


class MainControl(Node):
    def __init__(self):
        super().__init__("main_control")
        ## 訂閱彩色與深度影像
        self.color_image = self.create_subscription(Image, '/tm_robot/color_image', self.color_image_callback, 10)
        self.depth_image = self.create_subscription(Image, '/tm_robot/depth_image', self.depth_image_callback, 10)
        self.curobo_state = self.create_subscription(Bool, '/curobo/state', self.curobo_state_callback, 10)
        self.sct_sub = self.create_subscription(SctResponse,'/sct_response',self.sct_callback,10)
        # self.io_state = self.create_subscription(Bool, '/io/state', self.io_state_callback, 10)
        self.curobo_pose = self.create_publisher(Pose, '/cube_position', 10)
        self.leave_curobo = self.create_publisher(Bool, '/leave_node', 10)
        # self.curobo_control = self.create_publisher(Pose, '/curobo/control', 10)
        self.grab_pose = self.create_subscription(Pose, '/grab_pose', self.grab_pose_callback, 10)
        self.shelf_level = self.create_publisher(Int32, '/open_floor', 10)
        self.shelf_ignore = self.create_publisher(Bool, '/shelf_status', 10)

        ## 建立服務 client
        self.slider_client = self.create_client(RailControl, 'rail_control')
        self.detect_client = self.create_client(GroundedSAM2Interface, 'grounded_sam2')
        self.grab_client = self.create_client(CaptureImage,'grab_detect')
        self.ocr_client = self.create_client(Paddle, 'paddleocr_check')
        self.llm_client = self.create_client(DrugIdentify, 'drug_identify')
        self.second_camera_client = self.create_client(SecondCamera, 'camera2')
        self.tm_flow_mode_client = self.create_client(TMFlowMode,'tm_flow_mode')
        self.tm_flow_script_client = self.create_client(SendScript, 'send_script')
        self.order_service = self.create_service(MedicineOrder,'medicine_order',self.medicine_order_callback)
        self.curobo_state_client = self.create_client(Trigger,'/check_goal_arrival')
        self.moveit_set_pose_client = self.create_client(SetPositions,'set_positions')
        ## clients dict
        self.service_clients = {
            "grounded_sam2": self.detect_client,
            # "ocr": self.ocr_client,
            # "llm": self.llm_client,
            # "sencond_camera": self.second_camera_client,
            "grab_detect": self.grab_client,
            # "rail_control": self.slider_client,
            # "order": self.order_service,
            "tm_flow_mode": self.tm_flow_mode_client,
            "tm_flow_script": self.tm_flow_script_client,
            "curobo_state": self.curobo_state_client,
            "moveit_set_pose": self.moveit_set_pose_client
        }
        # self.tm_flow_mode_client.wait_for_service()
        ## wait all client activate
        for name, client in self.service_clients.items():
            while not client.wait_for_service(timeout_sec=1.0):
                self.get_logger().info(f"等待服務 [{name}] 啟動中...")
        ## variables
        self.get_logger().info("所有服務已啟動，開始流程")
        self.bridge = CvBridge()
        self.color_image_np = None
        self.depth_image_np = None
        self.curobo_state_flag = False
        self.has_run = False
        self.order_list = ["test"]
        self.sct_listen_flag = False
        self.current_med = None
        self.grab_pose: Pose = None


    ## ======= [Realsense Camera] =======

    def color_image_callback(self, msg) -> None:
        self.color_image_np = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')


    def depth_image_callback(self, msg) -> None:
        self.depth_image_np = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')


    ## ======= [Curobo] =======

    def curobo_state_callback(self, msg) -> None:
        self.curobo_state_flag = msg.data


    def spin_until_curobo_finish(self):
        while True:
            future = self.curobo_state_client.call_async(Trigger.Request())
            rclpy.spin_until_future_complete(self, future)
            res = future.result()
            if res.success == True:
                self.get_logger().info(f"curobo point finish")
                self.leave_curobo.publish(Bool(data=True))
                break
            time.sleep(0.1)


    def send_curobo_pose_and_spin(self, pose: Pose) -> None:
        self.leave_curobo.publish(Bool(data=False))
        time.sleep(1)
        ## clear last point
        # clear_pose = Pose()
        # clear_pose.position = Point(x=0.0,y=0.0,z=40.0)
        # clear_pose.orientation = Quaternion(x=0.0,y=0.0,z=0.0,w=1.0)
        # self.curobo_pose.publish(clear_pose)
        ## normal point
        self.curobo_pose.publish(pose)
        self.get_logger().info(f"Published cube_position pose:\n{pose}")
        time.sleep(2)
        self.spin_until_curobo_finish()


    def vec_pose_to_ros2_pose(self, pose_vec: list[float]) -> Pose:
        x,y,z = pose_vec[:3]
        roll,pitch,yaw = pose_vec[3:]
        quat = R.from_euler('xyz',[roll,pitch,yaw],degrees=True).as_quat()
        pose_msg = Pose()
        pose_msg.position = Point(x=x,y=y,z=z)
        pose_msg.orientation = Quaternion(x=quat[0],y=quat[1],z=quat[2],w=quat[3])
        return pose_msg
    

    ## ======= [Moveit] =======  
    
    def send_moveit_point(self, pose: Pose):
        self.get_logger().info(f"moveit set pose: {pose}")
        setpos_req = SetPositions.Request()
        setpos_req.motion_type = 2  # PTP_T
        r = R.from_quat([pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w])
        euler = r.as_euler('xyz', degrees=False)
        setpos_req.positions = [
            pose.position.x, pose.position.y, pose.position.z,
            euler[0],
            euler[1],
            euler[2]
        ]
        euler = r.as_euler('xyz', degrees=True)
        print(f"euler: {euler}")
        print(setpos_req.positions)
        setpos_req.velocity = 0.2
        setpos_req.acc_time = 0.2
        setpos_req.blend_percentage = 0
        setpos_req.fine_goal = True
        future = self.moveit_set_pose_client.call_async(setpos_req)
        rclpy.spin_until_future_complete(self, future)
        result = future.result()
        if result.ok:
            self.get_logger().info("set_positions success")
            input("press enter after move finished...")
        else:
            self.get_logger().error("set_positions failed")
        pass


    ## ======= [TM Flow] =======

    def sct_callback(self, msg: SctResponse) -> None:
        # self.get_logger().info(f"[SCT] ID: {msg.id} | SCRIPT: {msg.script}")
        if "Listen" in msg.script:
            self.sct_listen_flag = True
            self.get_logger().info("Listen code")
        elif "leave" in msg.script:
            self.get_logger().info("leave the codd")
        else:
            pass
            # self.get_logger().info(f"unknown SCT: {msg.script}")


    def send_tm_flow_mode_and_spin(self, mode:str, argument:str) -> bool:
        self.get_logger().info(f"send tm flow mode: {mode}, argument: {argument}")
        req = TMFlowMode.Request()
        req.mode = mode
        req.argument = argument
        future = self.tm_flow_mode_client.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        if future.result() is not None:
            if future.result().success:
                self.get_logger().info("TM Flow Mode success")
                return True
            else:
                self.get_logger().warn("TM Flow Mode failed")
                return False


    def send_tm_flow_leave_and_spin(self) -> bool:
        req = SendScript.Request()
        req.id = "main"
        req.script = "ScriptExit()"
        future = self.tm_flow_script_client.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        if future.result() is not None:
            if future.result().ok:
                self.get_logger().info("TM flow leave success")
                return True
            else:
                self.get_logger().warn("TM flow leave failed")
        else:
            self.get_logger().error("call TM flow leave service failed")
        return False

    
    def spin_until_sct_listen_flag(self) -> None:
        while not self.sct_listen_flag:
            rclpy.spin_once(self)
        self.sct_listen_flag = False


    ## ======= [Suck] =======

    def grab_pose_callback(self, msg: Pose) -> None:
        self.get_logger().info(f"grab pose receive: {msg}")
        self.grab_pose = msg


    def spin_until_grab_pose_available(self, timeout: float = 8) -> Pose | None:
        start_time = time.time()
        while not self.grab_pose and time.time() - start_time < timeout:
            rclpy.spin_once(self)
        pose = self.grab_pose
        self.grab_pose = None
        return pose
    

    def call_groundsam2_and_spin(self) -> tuple[list[float], np.ndarray] | None:
        if self.color_image_np is None or self.depth_image_np is None:
            self.get_logger().warn("color 或 depth 尚未準備好，略過 GroundedSAM2 呼叫")
            return
        req = GroundedSAM2Interface.Request()
        ros_color_image = self.bridge.cv2_to_imgmsg(self.color_image_np, encoding='bgr8')
        ros_depth_image = self.bridge.cv2_to_imgmsg(self.depth_image_np, encoding='passthrough')
        # now = self.get_clock().now().to_msg()
        # ros_color_image.header = Header(stamp=now, frame_id='camera')
        # ros_depth_image.header = Header(stamp=now, frame_id='camera')
        req.image = ros_color_image
        req.depth = ros_depth_image
        req.prompt = 'White medicine jar with red lid'
        req.prompt = 'White medicine jar with label on lid'
        # req.prompt = 'ointment'
        # req.prompt = "tablet"
        req.confidence_threshold = 0.3
        req.size_threshold = 0.02
        req.selection_mode = "closest"
        future = self.detect_client.call_async(req)
        ## handle result
        rclpy.spin_until_future_complete(self, future)
        res = future.result()
        ## check None result
        if res is None:
            self.get_logger().error("GroundedSAM2 回傳為 None")
            return None
        self.get_logger().info(f"辨識結果: label={res.label}, score={res.score}, bbox={res.bbox}")
        ## check empty mask 
        if not res.binary_image.data:
            self.get_logger().warn("回傳的 binary_image 是空的")
            return None
        ## binarization mask
        mask_raw = self.bridge.imgmsg_to_cv2(res.binary_image, desired_encoding='mono8')
        mask_img = (mask_raw > 0).astype(np.uint8) * 255
        print("原始遮罩 unique 值:", np.unique(mask_raw))  
        print("二值化遮罩 unique 值:", np.unique(mask_img))  
        ## check black mask 
        nonzero = np.count_nonzero(mask_img)
        self.get_logger().info(f"遮罩非零像素數量: {nonzero}")
        if nonzero == 0:
            self.get_logger().error("遮罩為全黑，無有效區域")
        else:
            self.get_logger().info("遮罩有效，可用於後續步驟")
        cv2.imwrite("/workspace/tm_robot/src/visuial/visuial/mask_raw.png", mask_raw)
        ## for debug ?
        # if self.color_image_np is not None:
        #     overlay = self.color_image_np.copy()
        #     mask_bin = (self.mask_img > 0).astype(np.uint8)
        #     mask_color = cv2.applyColorMap(mask_bin * 255, cv2.COLORMAP_JET)
        #     alpha = 0.5
        #     for c in range(3):
        #         overlay[:, :, c] = np.where(
        #             mask_bin,
        #             cv2.addWeighted(overlay[:, :, c], 1 - alpha, mask_color[:, :, c], alpha, 0),
        #             overlay[:, :, c]
        #         )
        for lbl in res.label:
            self.get_logger().info(f"辨識到標籤: {lbl}")
        ## return mask when not black
        if nonzero != 0:
            self.get_logger().info("遮罩有效，呼叫抓取服務")
            return res.bbox, mask_img


    def call_grab_and_spin(self, bbox: list[float], mask_img: np.ndarray) -> bool:
        req = CaptureImage.Request()
        req.mask = self.bridge.cv2_to_imgmsg(mask_img.astype(np.uint8), encoding='mono8')
        req.bbox = bbox
        self.get_logger().info("呼叫 grab_detect(mask 傳入中）...")
        future = self.grab_client.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        res = future.result()
        if res.success:
            self.get_logger().info("抓取位置已處理成功")
            return True
        else:
            self.get_logger().warn("抓取服務回傳失敗")
            return False


    ## ======= [Slider] =======

    def call_slider_and_wait(self, code: int) -> bool:
        req = RailControl.Request()
        req.opt_code = code
        req.rail_name = "arm_rail1"
        future = self.slider_client.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        res = future.result()
        if res.result >= 0:
            self.get_logger().info(f"slider opt: {code} Success: {res.result}")
            return True
        else:
            self.get_logger().error(f"slider opt: {code} Fail: {res.result}")
            return False


    ## ======= [Second Check] =======

    def call_second_camera_and_spin(self) -> bool:

       #==============request service===============
        req = SecondCamera.Request()
        future = self.second_camera_client.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        res = future.result()
        if res.success:
            self.get_logger().info("SecondCamera finish combine picture")
            return True
        else:
            self.get_logger().warn("SecondCamera 啟動失敗")
            return False
        # def handle_camera_response(fut):
        #     try:
        #         res = fut.result()
        #         if res.success:
        #             self.get_logger().info("SecondCamera finish combine picture")
        #         else:
        #             self.get_logger().warn("SecondCamera 啟動失敗")
        #     except Exception as e:
        #         self.get_logger().error(f"SecondCamera callback 錯誤: {str(e)}")

        # future.add_done_callback(handle_camera_response)


    def call_llm_and_spin(self, med_name: str) -> bool:
        req = DrugIdentify.Request()
        img = cv2.imread(COMBINE_PIC_PATH)
        req.image = self.bridge.cv2_to_imgmsg(img, encoding='bgr8')
        req.name = med_name
        future = self.llm_client.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        res = future.result()
        self.get_logger().info(f"LLM 回傳結果: {res.result}")
        if res.result == "yes":
            return True
        else:
            return False


    ## ======= [Shelf] =======  

    def send_shelf_level(self, layer: int) -> None:
        self.shelf_level.publish(Int32(data=layer))


    def send_shelf_ignore(self, ignore: bool) -> None:
        self.shelf_ignore.publish(Bool(data=ignore))


    ## ======= [Other] =======

    def load_order_medicines(self, order_id: str):
        ORDER_PATH = "/workspace/tm_robot/src/tm_robot_main/tm_robot_main/med_order/order_data.yaml"
        with open(ORDER_PATH, 'r', encoding='utf-8') as f:
            order_data = yaml.safe_load(f)

        if order_id not in order_data:
            self.get_logger().error(f"找不到訂單 {order_id}")
            return False

        self.medicine_map = order_data[order_id]  # e.g. {"medicine_1": {...}, "medicine_2": {...}}
        self.medicine_queue = list(self.medicine_map.keys())
        self.get_logger().info(f"載入訂單 {order_id}，藥物列表：{self.medicine_queue}")
        return True
    

    def execute_next_medicine(self):
        if not self.medicine_queue:
            self.get_logger().info("所有藥品已處理完成")
            return

        current_key = self.medicine_queue.pop(0)
        current_item = self.medicine_map[current_key]  # e.g. {"amount": 87, "locate": [1,1]}
        locate_str = f"{current_item['locate'][0]}-{current_item['locate'][1]}"
        self.get_logger().info(f"處理藥物：{current_key}，位置：{locate_str}")

        # 找對應的藥名
        MED_INFO_PATH = "/workspace/tm_robot/src/tm_robot_main/tm_robot_main/med_order/med_info.yaml"
        with open(MED_INFO_PATH, 'r', encoding='utf-8') as f:
            med_info = yaml.safe_load(f)

        found_name = None
        for name, info in med_info.items():
            if info.get("藥物基本資料", {}).get("存取位置", "") == locate_str:
                found_name = name
                break

        if not found_name:
            self.get_logger().warn(f"找不到與位置 {locate_str} 相符的藥物")
            return

        self.current_med_name = found_name
        self.get_logger().info(f"當前藥物名稱為：{self.current_med_name}")
        self.execute_next_medicine()


    def medicine_order_callback(self, request, response):
        self.order_list.append(request.command)
        response.success = True
        return response


    def get_order(self) -> dict | None:
        if not self.order_list:
            return None
        return self.order_list.pop(0)


    def call_ocr(self):
        if self.color_image_np is None:
            self.get_logger().warn("尚未收到 color image，無法執行 OCR")
    #     if not self.order_list:
    #         return None
    #     return self.order_list.pop(0)
            return

        req = Paddle.Request()
        ros_image = self.bridge.cv2_to_imgmsg(self.color_image_np, encoding='bgr8')
        req.image = ros_image

        future = self.ocr_client.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        res = future.result()
        self.get_logger().info(f"OCR 結果: {res.text}")

                    


    def spin_until_grab_finish(self):
        while not self.grab_finished:
            rclpy.spin_once(self)







def main(args=None):
    named_pose: dict = None
    with open(NAMED_POSE_YAML_PATH, 'r', encoding='utf-8') as f:
        named_pose = yaml.safe_load(f)
    if named_pose is None:
        raise("cannot find yaml")
    rclpy.init(args=args)
    node = MainControl()
    node.create_rate(100)   
    ## init slider
    # if not node.call_slider_and_wait(0):
    #     node.get_logger().error("call slider with 0 return failed")
    ## loop
    # node.order_list = []
    while rclpy.ok():
        rclpy.spin_once(node, timeout_sec=0.1)
        order = node.get_order()
        if order is not None:
            print("new order")


            ## slider get bag
            # if not node.call_slider_and_wait(1):
            #     node.get_logger().error("call slider with 1 return failed")
            #     continue

            # ## tm flow: location_shelf
            if not node.send_tm_flow_mode_and_spin("location_shelf", "0"):
                node.get_logger().error("call tm_flow_mode return failed")
                continue
            # ## check SCT
            node.spin_until_sct_listen_flag()
            
            # ## curobo move to shelf photo pose
            print("load")
            node.send_curobo_pose_and_spin(node.vec_pose_to_ros2_pose(named_pose["arm_base_pose"]["first_shelf_pull"]))
            ## set shelf level
            node.send_shelf_level(1)
            print("load1")
            # ## tm flow leave

            if not node.send_tm_flow_leave_and_spin():
                node.get_logger().error("call tm_flow_leave return failed")
                continue
            # time.sleep(10)
            # ## wait until curobo finish
            node.spin_until_curobo_finish()
            print("finish")
            # #  ## tm flow leave
            if not node.send_tm_flow_leave_and_spin():
                node.get_logger().error("call tm_flow_leave return failed")
                continue
            # # ## check SCT pose positions

            # node.send_curobo_point_and_spin(pose)
            node.spin_until_sct_listen_flag()
            # # ## tm flow leave
            if not node.send_tm_flow_leave_and_spin():
                node.get_logger().error("call tm_flow_leave return failed")
                continue
            # # # # ## curobo do not ignore shelf
            node.send_shelf_ignore(False)

            # ## tm flow: med_box
            if not node.send_tm_flow_mode_and_spin("medicine_box", "-1"):
                node.get_logger().error("call tm_flow_mode return failed")
            # ## check SCT
            # node.spin_until_sct_listen_flag()
            # ## groundSAM
            # gsam_result =  node.call_groundsam2_and_spin()
            # if gsam_result is None:
            #     node.get_logger().error("call GroundedSAM2 return failed")
            #     continue
            # bbox, mask_img = gsam_result
            # ## call grab point estimation
            # if not node.call_grab_and_spin(bbox, mask_img):
            #     node.get_logger().error("call grab return failed")
            #     continue
            # ## receive grab pose
            # pose = node.spin_until_grab_pose_available()
            # if not pose:
            #     node.get_logger().error("wait grab pose failed")
            #     continue
            # ## check SCT
            # node.spin_until_sct_listen_flag()
            # ## curobo move to rail landmark
            # # node.curobo_point_and_wait(node.vec_pose_to_ros2_pose(named_pose["rail_photo"]["rail_photo"]))
            # ## check SCT
            # # node.spin_until_sct_listen_flag()
            # #     continue
            # ## moveit move to grab pose
            # node.send_moveit_point(pose)
            # # ## curobo move to grab pose
            # # node.send_curobo_point_and_spin(pose)
            # ## tm flow leave
            # node.send_tm_flow_leave_and_spin()

            # # ## tm flow: grab_and_check
            # if not node.send_tm_flow_mode_and_spin("grab_and_check", "0"):
            #     node.get_logger().error("call tm_flow_mode return failed")
            # # # ## check SCT
            # node.spin_until_sct_listen_flag()
            # # # ## curobo move to rail landmark
            # node.curobo_point_and_wait(node.vec_pose_to_ros2_pose(named_pose["rail_photo"]["rail_photo"]))
            # # ## check SCT
            # node.spin_until_sct_listen_flag()
            # ## tm flow leave
            # node.send_tm_flow_leave_and_spin()
            # ## take picture for medicine
            # if not node.call_second_camera_and_spin():
            #     node.get_logger().error("call second camera return failed")
            #     continue
            # ## send picture to llm
            # if not node.call_llm_and_spin("Andsodhcd"):
            #     node.get_logger().error("call llm return failed")
            #     continue
            # ## tm flow leave
            # node.send_tm_flow_leave_and_spin()
            # ## check SCT
            # node.spin_until_sct_listen_flag()
            # ## tm flow leave
            # node.send_tm_flow_leave_and_spin()


            
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
