import rclpy
from rclpy.node import Node
import cv2
import numpy as np
from sensor_msgs.msg import Image
from std_msgs.msg import Header, Bool, Int32
from std_srvs.srv import Trigger
from tm_robot_if.srv import GroundedSAM2Interface,CaptureImage,DrugIdentify,SecondCamera,Paddle, TMFlowMode
from tm_robot_if.srv import CompleteOrder,Paddle
from std_msgs.msg import String
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
CLASS1_PIC_PATH="/workspace/tm_robot/src/visuial/sample_picture/img1.jpg"
COMBINE_PIC_PATH = "/workspace/tm_robot/src/visuial/sample_picture/combined.jpg"
MEDICINE_INFO_PATH = "/workspace/tm_robot/src/visuial/visuial/medicine_info2.yaml"
ORDER_FOLDER = ""
NAMED_POSE_YAML_PATH = '/workspace/tm_robot/src/tm_robot_main/tm_robot_main/robot_base_point.yaml'
CLASS1 = {"藥膏", "藥片"}
CLASS2 = {"藥罐", "眼藥水"}

class MainControl(Node):
    def __init__(self):
        super().__init__("main_control")
        ## 訂閱彩色與深度影像
        self.color_image = self.create_subscription(Image, '/tm_robot/color_image', self.color_image_callback, 10)
        self.depth_image = self.create_subscription(Image, '/tm_robot/depth_image', self.depth_image_callback, 10)
        self.curobo_state = self.create_subscription(Bool, '/curobo/state', self.curobo_state_callback, 10)
        self.sct_sub = self.create_subscription(SctResponse,'/sct_response',self.sct_callback,10)
        self.curobo_error = self.create_subscription(Bool, '/curobo/plan_fail', self.curobo_error_callback, 10)
        # self.io_state = self.create_subscription(Bool, '/io/state', self.io_state_callback, 10)
        self.curobo_pose = self.create_publisher(Pose, '/cube_position', 10)
        self.leave_curobo = self.create_publisher(Bool, '/leave_node', 10)
        # self.curobo_control = self.create_publisher(Pose, '/curobo/control', 10)
        self.grab_pose = self.create_subscription(Pose, '/grab_pose', self.grab_pose_callback, 10)
        # self.shelf_level = self.create_publisher(Int32, '/open_floor', 10)
        # self.shelf_ignore = self.create_publisher(Bool, '/shelf_status', 10)
        self.order = self.create_subscription(String,'/hospital/new_order', self.new_order_callback,10)
        
        ## 建立服務 client
        self.slider_client = self.create_client(RailControl, 'rail_control')
        self.detect_client = self.create_client(GroundedSAM2Interface, 'grounded_sam2')
        self.grab_client = self.create_client(CaptureImage,'grab_detect')
        self.ocr_client = self.create_client(Paddle, 'paddleocr_check')
        self.llm_client = self.create_client(DrugIdentify, 'drug_identify')
        self.second_camera_client = self.create_client(SecondCamera, 'camera2')
        self.tm_flow_mode_client = self.create_client(TMFlowMode,'tm_flow_mode')
        self.tm_flow_script_client = self.create_client(SendScript, 'send_script')
        self.curobo_state_client = self.create_client(Trigger,'/check_goal_arrival')
        self.moveit_set_pose_client = self.create_client(SetPositions,'set_positions')
        self.complete_order_client = self.create_client(CompleteOrder, 'complete_order')


        ## clients dict
        self.service_clients = {
            "grounded_sam2": self.detect_client,
            "llm": self.llm_client,
            "sencond_camera": self.second_camera_client,
            "grab_detect": self.grab_client,
            "paddleocr_check": self.ocr_client,
            # "rail_control": self.slider_client,
            "tm_flow_mode": self.tm_flow_mode_client,
            "tm_flow_script": self.tm_flow_script_client,
            "curobo_state": self.curobo_state_client,
            "moveit_set_pose": self.moveit_set_pose_client,
            "complete_order": self.complete_order_client,
        }
        for name, client in self.service_clients.items():
            while not client.wait_for_service(timeout_sec=1.0):
                self.get_logger().info(f"等待服務 [{name}] 啟動中...")
        ## variables
        self.get_logger().info("所有服務已啟動，開始流程")
        self.bridge = CvBridge()
        self.color_image_np = None
        self.depth_image_np = None
        self.curobo_state_flag = False
        self.curobo_error_flag = False
        self.has_run = False
        self.order_list: list[dict] = []
        self.sct_listen_flag = False
        self.grab_pose: Pose = None
        self.current_shelf_level = 0


    ## ======= [Realsense Camera] =======

    def color_image_callback(self, msg) -> None:
        self.color_image_np = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')


    def depth_image_callback(self, msg) -> None:
        self.depth_image_np = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')


    ## ======= [Curobo] =======

    def curobo_state_callback(self, msg) -> None:
        self.curobo_state_flag = msg.data

    def curobo_error_callback(self, msg) -> None:
        self.get_logger().info(f"curobo error: {msg.data}")
        self.curobo_error_flag = msg.data

    def spin_until_curobo_finish(self) -> bool:
        ret = False
        while True:
            if self.curobo_error_flag:
                self.get_logger().error("curobo error")
                break
            future = self.curobo_state_client.call_async(Trigger.Request())
            rclpy.spin_until_future_complete(self, future)
            res = future.result()
            if res.success == True:
                self.get_logger().info(f"curobo point finish")
                ret = True
                break
            time.sleep(0.1)
        self.leave_curobo.publish(Bool(data=True))
        time.sleep(0.5)
        return ret


    def send_curobo_pose_and_spin(self, pose: Pose) -> bool:
        MAX_RETRY = 3
        for _ in range(MAX_RETRY):
            self.curobo_error_flag = False
            self.leave_curobo.publish(Bool(data=False))
            time.sleep(0.5)
            ## normal point
            self.curobo_pose.publish(pose)
            self.get_logger().info(f"Publish cube_position:\n{pose}")
            st = time.time()
            while(time.time() - st < 5):
                rclpy.spin_once(self)
            if not self.spin_until_curobo_finish():
                self.send_moveit_joint([0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
                self.get_logger().warn("curobo error, go home with moveit")
                pose.position.z -= 0.002
                ## clear last point
                # self.leave_curobo.publish(Bool(data=False))
                # time.sleep(0.5)
                # clear_pose = Pose()
                # clear_pose.position = Point(x=0.0,y=0.0,z=40.0)
                # clear_pose.orientation = Quaternion(x=0.0,y=0.0,z=0.0,w=1.0)
                # self.curobo_pose.publish(clear_pose)
                # self.leave_curobo.publish(Bool(data=True))
            else:
                return True
            time.sleep(1)
        return False


    def vec_pose_to_ros2_pose(self, pose_vec: list[float]) -> Pose:
        x,y,z = pose_vec[:3]
        roll,pitch,yaw = pose_vec[3:]
        quat = R.from_euler('xyz',[roll,pitch,yaw],degrees=True).as_quat()
        pose_msg = Pose()
        pose_msg.position = Point(x=x,y=y,z=z)
        pose_msg.orientation = Quaternion(x=quat[0],y=quat[1],z=quat[2],w=quat[3])
        return pose_msg
    
    def ros2_pose_rotate_z(self, pose: Pose, angle: float) -> Pose:
        r = R.from_quat([pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w])
        rz_90 = R.from_euler("z", angle, degrees=True)
        r_mat = rz_90.as_matrix() @ r.as_matrix()
        r = R.from_matrix(r_mat)
        quat = r.as_quat()
        ret_pose = pose
        ret_pose.orientation.x = quat[0]
        ret_pose.orientation.y = quat[1]
        ret_pose.orientation.z = quat[2]
        ret_pose.orientation.w = quat[3]
        return ret_pose

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

    def verify_medicine_by_ocr_service(self, med_name: str) -> bool:
        if self.color_image_np is None:
            self.get_logger().warn("尚未有 color image，無法驗證")
            return False
        req = Paddle.Request()
        req.image = self.bridge.cv2_to_imgmsg(self.color_image_np, encoding='bgr8')
        req.med_name = med_name
        future = self.ocr_client.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        res = future.result()
        if res is None:
            self.get_logger().error("ocr_verify_img 回傳 None")
            return False
        return bool(res.matched)

    def send_moveit_joint(self, joints: list[float], op_time: float = 5):
        self.get_logger().info(f"moveit set joint: {joints}")
        setpos_req = SetPositions.Request()
        setpos_req.motion_type = 1  # PTP_J
        setpos_req.positions = joints
        setpos_req.velocity = 1.0
        setpos_req.acc_time = 0.2
        setpos_req.blend_percentage = 0
        setpos_req.fine_goal = False
        future = self.moveit_set_pose_client.call_async(setpos_req)
        rclpy.spin_until_future_complete(self, future)
        result = future.result()
        if result.ok:
            self.get_logger().info("moveit set joint success")
            time.sleep(op_time)
            # input("press enter after move finished...")
        else:
            self.get_logger().error("moveit set joint failed")
        pass


    ## ======= [TM Flow] =======

    def sct_callback(self, msg: SctResponse) -> None:
        # self.get_logger().info(f"[SCT] ID: {msg.id} | SCRIPT: {msg.script}")
        if "Listen" in msg.script:
            self.sct_listen_flag = True
            self.get_logger().info("receive SCT")
        elif "leave" in msg.script:
            self.get_logger().info("leave the code")
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
        self.get_logger().info("Request TM flow leave")
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
        self.get_logger().info("Waiting TM flow SCT")
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
    

    def call_groundsam2_and_spin(self, prompt: str, confidence: float, size_threshold: float) -> tuple[list[float], np.ndarray] | None:
        if self.color_image_np is None or self.depth_image_np is None:
            self.get_logger().warn("color 或 depth 尚未準備好，略過 GroundedSAM2 呼叫")
            return
        req = GroundedSAM2Interface.Request()
        ros_color_image = self.bridge.cv2_to_imgmsg(self.color_image_np, encoding='bgr8')
        ros_depth_image = self.bridge.cv2_to_imgmsg(self.depth_image_np, encoding='passthrough')
        req.image = ros_color_image
        req.depth = ros_depth_image
        # req.prompt = 'White medicine jar with red lid'
        # req.prompt = 'White medicine jar with label on lid'
        req.prompt = prompt
        # req.prompt = 'ointment'
        # req.prompt = "tablet"
        # req.confidence_threshold = 0.3
        req.confidence_threshold = confidence
        req.size_threshold = size_threshold
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
        # cv2.imwrite("/workspace/tm_robot/src/visuial/visuial/mask_raw.png", mask_raw)
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
        req.shelf_level = self.current_shelf_level
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
        self.get_logger().info(f"call slider opt: {code}")

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

    def call_second_camera_and_spin(self, med_class: int) -> bool:
        # class1 -> 1 張；class2 -> 4 張（其餘值一律當 4）
        try:
            shots = 1 if int(med_class) == 1 else 4
        except Exception:
            shots = 4

        req = SecondCamera.Request()
        req.pic_cnt = shots
        future = self.second_camera_client.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        res = future.result()
        if res and res.success:
            self.get_logger().info(f"SecondCamera finish combine picture (shots={shots})")
            return True
        else:
            self.get_logger().warn("SecondCamera 啟動失敗")
            return False

        

    def call_llm_and_spin(self, med_name: str,med_class: int) -> bool:
        time.sleep(0.5)
        req = DrugIdentify.Request()
        if med_class==1:
            img = cv2.imread(CLASS1_PIC_PATH)
        else:
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

    # def send_shelf_level(self, layer: int) -> None:
    #     self.shelf_level.publish(Int32(data=layer))


    # def send_shelf_ignore(self, ignore: bool) -> None:
    #     self.shelf_ignore.publish(Bool(data=ignore))


    ## ======= [Order] =======  

    def new_order_callback(self, msg: String) -> None:
        try:
            parsed = yaml.safe_load(msg.data)
            if isinstance(parsed, dict):
                order_data = parsed
            else:
                raise ValueError(f"YAML 格式錯誤，解析結果是 {type(parsed).__name__}")
        except Exception as e:
            self.get_logger().error(f"解析訂單失敗: {e}")
            return
        ## get medicines
        med_list = order_data.get('medicine', [])
        order_id = order_data.get('order_id')
        self.get_logger().info(f"訂單 {order_id} 共 {len(med_list)} 項藥品")
        ## parse position
        for medicine in med_list:
            medicine['position'] = list(map(int, medicine['position'].split("-")))
        ## sort by position
        med_list = sorted(med_list, key=lambda x: x["position"][0])
        self.order_list.append({
            'id': order_id,
            'medicine': med_list
        })
        self.get_logger().info(f"order list: {self.order_list}")


    def get_order(self) -> dict | None:
        if not self.order_list:
            return None
        return self.order_list.pop(0)
    

    def complete_current_order(self, order_id: str, status="success"):
        req = CompleteOrder.Request()
        req.order_id = order_id
        req.status = status
        future = self.complete_order_client.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        res = future.result()
        if res.success:
            self.get_logger().info(f"訂單 {order_id} 已完成")
        else:
            self.get_logger().error(f"結單失敗: {res.error}")


    ## ======= [Other] =======
    

    def execute_next_medicine(self):
        if not self.medicine_queue:
            self.get_logger().info("所有藥品已處理完成")
            return

        current_key = self.medicine_queue.pop(0)
        current_item = self.medicine_map[current_key]
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


    # def process_order(self):
    #     """拉取並處理所有訂單"""
    #     self.get_next_order_client.wait_for_service()
    #     self.complete_order_clientmed_box.wait_for_service()

    #     # 拉單
    #     req = GetNextOrder.Request()
    #     future = self.get_next_order_client.call(req)
    #     if not future.success:
    #         self.get_logger().info(f"沒有訂單: {future.error}")
    #         return

    #     self.current_id = future.order_id

    #     # 安全解析
    #     try:
    #         raw_data = future.order_yaml

    #         if isinstance(raw_data, dict):
    #             order_data = raw_data
    #         elif isinstance(raw_data, str):
    #             parsed = yaml.safe_load(raw_data)
    #             if isinstance(parsed, dict):
    #                 order_data = parsed
    #             else:
    #                 raise ValueError(f"YAML 格式錯誤，解析結果是 {type(parsed).__name__}")
    #         else:
    #             raise ValueError(f"未知的訂單資料型別: {type(raw_data).__name__}")

    #     except Exception as e:
    #         self.get_logger().error(f"解析訂單失敗: {e}")
    #         self.current_id = None
    #         self.med_list = []
    #         return

    #     self.med_list = order_data.get('medicine', [])
    #     self.get_logger().info(f"訂單 {self.current_id} 共 {len(self.med_list)} 項藥品")

    #     # 逐項處理藥品
    #     for idx, med in enumerate(self.med_list):
    #         self.current_med = med
    #         self.get_logger().info(f"處理第 {idx+1} 項: {med['name']} x{med['amount']}")

    #     self.complete_current_order()






# def main(args=None):
#     rclpy.init(args=args)
#     node = MainControl()
#     node.create_rate(100)   
#     try:
#         while rclpy.ok():
#             rclpy.spin_once(node)
#     except KeyboardInterrupt:
#         pass
#     finally:
#         node.destroy_node()
#         rclpy.shutdown()

def category_to_flag01(category: str, default: str = "0") -> str:
    """
    回傳 '1'（CLASS1）或 '0'（CLASS2），找不到回 default
    """
    if not category:
        return default
    if category in CLASS1:
        return "1"
    if category in CLASS2:
        return "0"
    return default



def process_medicine(node: MainControl, medicine: dict):
    named_pose: dict = None
    with open(NAMED_POSE_YAML_PATH, 'r', encoding='utf-8') as f:
        named_pose = yaml.safe_load(f)
    if named_pose is None:
        raise("cannot find yaml")
    ## get param
    name = medicine['name']
    amount = medicine['amount']
    shelf_level = medicine['position'][0]
    med_box_num = medicine['position'][1]
    med_class = medicine['category']
    med_class = category_to_flag01(med_class)
    area = medicine['area']
    ## calculate medicine box num count from landmark
    if shelf_level == 1:
        med_box_num = -med_box_num + 1
    else:
        med_box_num = 4 - med_box_num
    node.get_logger().info(f"process medicine name: {name}, amount: {amount}, level: {shelf_level}, box: {med_box_num}")
    ## do amount times
    for _ in range(amount):
        ## slider get bag
        # if not node.call_slider_and_wait(1):
        #     node.get_logger().error("call slider with 1 return failed")
        #     continue

        ## ======= [Close Shelf] =======
        node.get_logger().info("======= [Close Shelf] =======")
        if node.current_shelf_level != 0 and node.current_shelf_level != shelf_level:
            ## tm flow: location_shelf
            if not node.send_tm_flow_mode_and_spin("location_shelf", str(-node.current_shelf_level)):
                node.get_logger().error("call tm_flow_mode return failed")
                continue
            ## check SCT
            node.spin_until_sct_listen_flag()
            ## curobo move to landmark 
            if node.current_shelf_level == 1:
                pass
                # node.get_logger().info("first shelf")
                # node.send_curobo_pose_and_spin(node.vec_pose_to_ros2_pose(named_pose["shelf_landmark"]["first_push"]))
            else:
                node.get_logger().info("second push")
                node.send_curobo_pose_and_spin(node.vec_pose_to_ros2_pose(named_pose["shelf_landmark"]["second_push"]))
            ## tm flow leave
            if not node.send_tm_flow_leave_and_spin():
                node.get_logger().error("call tm_flow_leave return failed")
                continue
            ## check SCT
            node.spin_until_sct_listen_flag()
            ## tm flow leave
            if not node.send_tm_flow_leave_and_spin():
                node.get_logger().error("call tm_flow_leave return failed")
                continue
            ## update variable
            node.current_shelf_level = 0

        ## ======= [Open Shelf] =======
        node.get_logger().info("======= [Open Shelf] =======")
        if node.current_shelf_level != shelf_level:
            ## tm flow: location_shelf
            while not node.send_tm_flow_mode_and_spin("location_shelf", str(shelf_level)):
                node.get_logger().error("call tm_flow_mode return failed")
            ## check SCT
            node.spin_until_sct_listen_flag()
            ## curobo move to landmark 
            if shelf_level == 1:
                node.get_logger().info("first shelf")
                # node.send_curobo_pose_and_spin(node.vec_pose_to_ros2_pose(named_pose["shelf_landmark"]["first_safe"]))
                # node.send_curobo_pose_and_spin(node.vec_pose_to_ros2_pose(named_pose["shelf_landmark"]["first_pull"]))
            else:
                node.get_logger().info("second pull")
                node.send_curobo_pose_and_spin(node.vec_pose_to_ros2_pose(named_pose["shelf_landmark"]["second_safe"]))
                node.send_curobo_pose_and_spin(node.vec_pose_to_ros2_pose(named_pose["shelf_landmark"]["second_safe2"]))
                node.send_curobo_pose_and_spin(node.vec_pose_to_ros2_pose(named_pose["shelf_landmark"]["second_pull"]))
            ## tm flow leave
            if not node.send_tm_flow_leave_and_spin():
                node.get_logger().error("call tm_flow_leave return failed")
                continue
            ## check SCT
            node.spin_until_sct_listen_flag()
            ## tm flow leave
            if not node.send_tm_flow_leave_and_spin():
                node.get_logger().error("call tm_flow_leave return failed")
                continue
            ## update variable
            node.current_shelf_level = shelf_level
            ## delay to wait isaac sync world
            time.sleep(3)

        ## ======= [Medicine box] =======
        node.get_logger().info("======= [Medicine box] =======")
        if not node.send_tm_flow_mode_and_spin("medicine_box", str(med_box_num)):
            node.get_logger().error("call tm_flow_mode return failed")
        ## check SCT
        node.spin_until_sct_listen_flag()
        ## curobo move to safe point
        if shelf_level == 1:
            node.send_curobo_pose_and_spin(node.vec_pose_to_ros2_pose(named_pose["med_box_start"]["first_safe"]))
        else:
            node.send_curobo_pose_and_spin(node.vec_pose_to_ros2_pose(named_pose["med_box_start"]["second_safe"]))


        ##move to medicine box for check name
        med_box_pose: list[float] = None
        node.get_logger().info(f"med_box_num: {med_box_num}")
        if shelf_level == 1:
            med_box_pose = named_pose["med_box_start"]["first_med_front"]
            med_box_pose[1] -= 0.185 * med_box_num
        else:
            med_box_pose = named_pose["med_box_start"]["second_med_front"]
            med_box_pose[1] -= 0.185 * med_box_num
        node.send_curobo_pose_and_spin(node.vec_pose_to_ros2_pose(med_box_pose))
        
        if not node.verify_medicine_by_ocr_service(name):
            node.get_logger().warn(f"OCR 驗證未通過：{name}")
            continue
       

        ## curobo move to med_box
        med_box_pose: list[float] = None
        if shelf_level == 1:
            med_box_pose = named_pose["med_box_start"]["first"]
            med_box_pose[1] -= 0.185 * med_box_num
        else:
            med_box_pose = named_pose["med_box_start"]["second"]
            med_box_pose[1] -= 0.185 * med_box_num
        node.send_curobo_pose_and_spin(node.vec_pose_to_ros2_pose(med_box_pose))
        # input("press enter after move to med box...")

        ## groundSAM
        confidence = medicine['confidence'] + 0.05
        pose = None
        time.sleep(0.5)
        for _ in range(3):
            confidence -= 0.05
            gsam_result =  node.call_groundsam2_and_spin(medicine['prompt'], confidence, medicine['area'])
            if gsam_result is None:
                node.get_logger().error("call GroundedSAM2 return failed")
                continue
            bbox, mask_img = gsam_result
            ## call grab point estimation
            if not node.call_grab_and_spin(bbox, mask_img):
                node.get_logger().error("call grab return failed")
                continue
            ## receive grab pose
            pose = node.spin_until_grab_pose_available()
            if not pose:
                node.get_logger().error("wait grab pose failed")
                continue
            ## success
            break
        ## grab pose failed
        if pose is None:
            node.get_logger().error("failed to get grab pose after retry")
            continue
        ## curobo move to grab pose
        if shelf_level == 2:
            pose = node.ros2_pose_rotate_z(pose, 180)
        node.send_curobo_pose_and_spin(pose)
        ## tm flow leave
        if not node.send_tm_flow_leave_and_spin():
            node.get_logger().error("call tm_flow_leave return failed")
            continue
            
        ## ======= [Grab and Check] =======
        node.get_logger().info("======= [Grab and Check] =======")
        # if not node.send_tm_flow_mode_and_spin("grab_and_check", "null"):
        if not node.send_tm_flow_mode_and_spin("grab_and_check", str(med_class)):
            node.get_logger().error("call tm_flow_mode return failed")
        ## check SCT
        node.spin_until_sct_listen_flag()
        
        # curobo move to safe point
        if shelf_level == 1 and med_box_num==-3:
            node.send_curobo_pose_and_spin(node.vec_pose_to_ros2_pose(named_pose["shelf_landmark"]["second_safe"]))
            node.send_curobo_pose_and_spin(node.vec_pose_to_ros2_pose(named_pose["rail"]["first_safe"]))

        elif shelf_level == 1:
            node.send_curobo_pose_and_spin(node.vec_pose_to_ros2_pose(named_pose["rail"]["first_safe"]))
        else:
            node.send_curobo_pose_and_spin(node.vec_pose_to_ros2_pose(named_pose["rail"]["second_safe"]))
        ## curobo move to rail landmark
        node.send_curobo_pose_and_spin(node.vec_pose_to_ros2_pose(named_pose["rail"]["landmark"]))
        ## tm flow leave
        if not node.send_tm_flow_leave_and_spin():
            node.get_logger().error("call tm_flow_leave return failed")
            continue
        
        ## check SCT
        node.spin_until_sct_listen_flag()
        ## tm flow leave
        if not node.send_tm_flow_leave_and_spin():
            node.get_logger().error("call tm_flow_leave return failed")
            continue
        ## take picture for medicine
        if not node.call_second_camera_and_spin(med_class):
            node.get_logger().error("call second camera return failed")
            continue
        ## send picture to llm
        if not node.call_llm_and_spin(name,med_class):
            node.get_logger().warn("call llm return failed")
            # continue

        ## check SCT
        node.spin_until_sct_listen_flag()
        ## tm flow leave
        node.send_tm_flow_leave_and_spin()
        # ## check SCT
        # node.spin_until_sct_listen_flag()
        # ## tm flow leave
        # node.send_tm_flow_leave_and_spin()



def grab_test(node: MainControl):
    ## groundSAM
    # gsam_result =  node.call_groundsam2_and_spin("circle label", 0.4, 0.1)  #1-1
    # gsam_result =  node.call_groundsam2_and_spin("circle label", 0.4, 0.01)  #1-3
    # gsam_result =  node.call_groundsam2_and_spin("backside of aluminum foil pill pack", 0.2, 0.05)  #2-4
    gsam_result =  node.call_groundsam2_and_spin("bottle",0.35 ,0.05)
    # gsam_result =  node.call_groundsam2_and_spin("bottle, Usually 5–15 ml in volume, handheld size",0.2,0.05)


    # gsam_result =  node.call_groundsam2_and_spin("backside of aluminum foil pill pack", 0.15, 0.05)  #1-3
    #  
    # gsam_result =  node.call_groundsam2_and_spin("tube", 0.5, 0.08)  #2-1
    
    # gsam_result =  node.call_groundsam2_and_spin("tube", 0.4, 0.1)  #1-2
    # gsam_result =  node.call_groundsam2_and_spin("White medicine jar with red lid", 0.4)

    if gsam_result is None:
        node.get_logger().error("call GroundedSAM2 return failed")
        return
    bbox, mask_img = gsam_result
    ## call grab point estimation
    if not node.call_grab_and_spin(bbox, mask_img):
        node.get_logger().error("call grab return failed")
        return
    ## receive grab pose
    pose = node.spin_until_grab_pose_available()
    if not pose:
        node.get_logger().error("wait grab pose failed")
        return


def test_med_class(node: MainControl, category: str) -> bool:
    flag = category_to_flag01(category)
    node.get_logger().info(f"[test_med_class] 類別='{category}' -> flag='{flag}'")
    if not node.send_tm_flow_mode_and_spin("grab_and_check", flag):
        node.get_logger().error("call tm_flow_mode (grab_and_check) failed")
        return False
    node.spin_until_sct_listen_flag()
    ok = node.send_tm_flow_leave_and_spin()
    if not ok:
        node.get_logger().warn("TM flow leave failed")
    return ok



def main(args=None):
    rclpy.init(args=args)
    node = MainControl()
    node.create_rate(100)   
    # loop

    # node.order_list = []
    named_pose: dict = None
    with open(NAMED_POSE_YAML_PATH, 'r', encoding='utf-8') as f:
        named_pose = yaml.safe_load(f)
    if named_pose is None:
        raise("cannot find yaml")
    
    # if not node.call_slider_and_wait(0):
    #     node.get_logger().error("call slider with 0 return failed")
    # if not node.call_slider_and_wait(1):
    #     node.get_logger().error("call slider with 1 return failed")
    # if not node.call_slider_and_wait(2):
    #     node.get_logger().error("call slider with 2 return failed")
    node.get_logger().info("wait sct finish")
    node.spin_until_sct_listen_flag()
    if not node.send_tm_flow_leave_and_spin():
        node.get_logger().error("call tm_flow_leave return failed")
    
    while rclpy.ok():
        rclpy.spin_once(node, timeout_sec=0.1)
        # grab_test(node)
        # test_med_class(node,"藥膏")
        order = node.get_order()
        if order is not None:
            order_id = order['id']
            node.get_logger().info(f"process order: {order_id}")
            ## slider get bag
            # if not node.call_slider_and_wait(0):
                # node.get_logger().error("call slider with 0 return failed")
            # if not node.call_slider_and_wait(1):
                # node.get_logger().error("call slider with 1 return failed")
            # else:
                # node.get_logger().info("slider get bag success")
            ## process medicine
            for medicine in order['medicine']:
                process_medicine(node, medicine)
            time.sleep(3)
            # slider put bag
            # if not node.call_slider_and_wait(2):
            #     node.get_logger().error("call slider with 2 return failed")
            ## UI complete
            node.complete_current_order(order_id)
        
            
    node.destroy_node()
    rclpy.shutdown()


# def main(args=None):
#     rclpy.init(args=args)
#     node = MainControl()
#     try:
#         med_name = "Andsodhcd"

#         if not os.path.exists(COMBINE_PIC_PATH):
#             node.get_logger().info("找不到 combined.jpg，先呼叫 SecondCamera 產生圖片")
#             if not node.call_second_camera_and_spin():
#                 node.get_logger().error("SecondCamera 失敗，無法進行 LLM 測試")
#                 return

#         # 只測一次，確保能看到結果
#         if not node.call_llm_and_spin(med_name):
#             node.get_logger().error("call llm return failed")
#         else:
#             node.get_logger().info("LLM 驗證通過（yes）")

#         # 若還要讓 node 繼續跑，再 spin
#         rclpy.spin(node)
#     finally:
#         node.destroy_node()
#         rclpy.shutdown()




if __name__ == '__main__':
    main()
