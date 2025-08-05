import rclpy
from rclpy.node import Node
import cv2
import numpy as np
from sensor_msgs.msg import Image
from std_msgs.msg import Header,Bool
from std_srvs.srv import Trigger
from tm_robot_if.srv import GroundedSAM2Interface,PoseSrv,CaptureImage,SecondCheck,DrugIdentify,SecondCamera,Paddle, TMFlowMode, MedicineOrder
from tm_rail_interface.srv import RailControl
from tm_msgs.msg import SctResponse
from tm_msgs.srv import SendScript
from cv_bridge import CvBridge
import os
import yaml
from geometry_msgs.msg import Pose,Point,Quaternion
import time
from scipy.spatial.transform import Rotation as R
import threading

WORKFLOW_PATH= "/workspace/tm_robot/src/tm_robot_main/tm_robot_main/workflow.yaml"
ARM_CONTROL_POINT = ""
COMBINE_PIC_PATH = "/workspace/tm_robot/data/second_cam"
MEDICINE_INFO_PATH = "/workspace/tm_robot/src/visuial/visuial/medicine_info2.yaml"
ORDER_FOLDER = ""


class MainControl(Node):
    def __init__(self):
        super().__init__("main_control")

        # 訂閱彩色與深度影像
        self.color_image = self.create_subscription(Image, '/tm_robot/color_image', self.color_image_callback, 10)
        self.depth_image = self.create_subscription(Image, '/tm_robot/depth_image', self.depth_image_callback, 10)
        self.curobo_state = self.create_subscription(Bool, '/curobo/state', self.curobo_state_callback, 10)
        self.sct_sub = self.create_subscription(SctResponse,'/sct_response',self.sct_callback,10)
        # self.io_state = self.create_subscription(Bool, '/io/state', self.io_state_callback, 10)
        self.curobo_pose = self.create_publisher(Pose, '/cube_position', 10)
        self.leave_curobo = self.create_publisher(Bool, '/leave_node', 10)
        # self.curobo_control = self.create_publisher(Pose, '/curobo/control', 10)

        # 建立服務 client
        self.detect_client = self.create_client(GroundedSAM2Interface, 'grounded_sam2')
        self.grab_client = self.create_client(CaptureImage,'grab_detect')
        self.ocr_client = self.create_client(Paddle, 'paddleocr_check')
        self.llm_client = self.create_client(DrugIdentify, 'drug_identify')
        self.second_camera_client = self.create_client(SecondCamera, 'camera2')
        self.slider_client = self.create_client(RailControl,'rail_control')
        self.tm_flow_mode_client = self.create_client(TMFlowMode,'tm_flow_mode')
        self.tm_flow_script_client = self.create_client(SendScript, 'send_script')
        self.order_service = self.create_service(MedicineOrder,'medicine_order',self.medicine_order_callback)
        self.curobo_state_client = self.create_client(Trigger,'/check_goal_arrival')
        ## clients dict
        self.service_clients = {
            "grounded_sam2": self.detect_client,
            # "ocr": self.ocr_client,
            # "llm": self.llm_client,
            # "camera2": self.second_camera_client,
            "grab_detect": self.grab_client,
            # "rail_control": self.slider_client,
            # "order": self.order_service,
            "tm_flow_mode": self.tm_flow_mode_client,
            "tm_flow_script": self.tm_flow_script_client,
            "curobo_state": self.curobo_state_client
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



    def execute_next_step(self):
        if self.current_step_index >= len(self.workflow.queue):
            self.get_logger().info("Workflow Finish")
            return
        step = self.workflow_queue[self.current_step_index]
        self.get_logger().info("step {step}")
        self.current_step_index += 1


        #======workflow arm control mode 
        if step == "detect_med":
            self.call_groundsam2()
        elif step == "ocr_service":
            self.call_ocr()
        elif step == "second_camera_check":
            self.call_camera2()
        elif step == "second_check":
            self.call_llm()

        # =====workflow slider mode
        elif step == "slider_init":
            self.call_slider(self.slider_mode["take_package"])
        elif step == "slider_come":
            self.call_slider(self.slider_mode["finish package"])
        elif step == "medicine_finish":
            self.call_slider(self.slider_mode["finish package"])


        else:
            self.get_logger().warn(f"未知步驟：{step}")
            self.execute_next_step()


    def run_once(self):
        pass
        # if self.has_run:
        #     return
        # self.has_run = True

        # self.workflow_data = self.workflow()
        # if not self.workflow_data:
        #     self.get_logger().error("無法載入工作流程")
        #     return

        # self.workflow_queue = self.flatten_workflow(self.workflow_data['workflow'])
        # self.current_step_index = 0
        # self.get_logger().info(f"工作流程：{self.workflow_queue}")
        # self.execute_next_step()
                    

    def workflow(self):
        if os.path.exists(WORKFLOW_PATH):
            with open(WORKFLOW_PATH, 'r', encoding='utf-8') as f:
                workflow = yaml.safe_load(f)
            return workflow
        else:
            self.get_logger().error(f"找不到工作流程檔案：{WORKFLOW_PATH}")
            return None

    
    def color_image_callback(self, msg):
        self.color_image_np = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')


    def depth_image_callback(self, msg):
        self.depth_image_np = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')


    def curobo_state_callback(self, msg):
        self.curobo_state_flag = msg.data


    def io_state_callback(self, msg):
        self.io_state_flag = msg.data


    def slider_state_callback(self, msg):
        self.slider_state_flag = msg.data
    

    def send_tm_flow_mode(self, mode:str, argument:str) -> bool:
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


    def send_tm_flow_leave_node(self):
        req = SendScript.Request()
        req.id = "main"
        req.script = "ScriptExit()"
        future = self.tm_flow_script_client.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        if future.result() is not None:
            if future.result().ok:
                self.get_logger().info("Script success")
            else:
                self.get_logger().warn("Script failed")
        else:
            self.get_logger().error("can't send_script service")


    def sct_callback(self, msg: SctResponse):
        # self.get_logger().info(f"[SCT] ID: {msg.id} | SCRIPT: {msg.script}")
        if "Listen" in msg.script:
            self.sct_listen_flag = True
            self.get_logger().info("Listen code")
        elif "leave" in msg.script:
            self.get_logger().info("leave the codd")
        else:
            pass
            # self.get_logger().info(f"unknown SCT: {msg.script}")


    def spin_until_sct_listen_flag(self):
        while not self.sct_listen_flag:
            rclpy.spin_once(self)
        self.sct_listen_flag = False


    def call_and_wait_groundsam2(self):
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
        req.prompt = 'whilte circle bottle'
        req.confidence_threshold = 0.20
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

        # future.add_done_callback(self.ocr_response_callback) 

    # def ocr_response_callback(self, future):
    #     try:
    #         res = future.result()
    #         self.get_logger().info(f"OCR 結果: {res.text}")
    #     except Exception as e:
    #         self.get_logger(). error(f"OCR callback 發生錯誤: {str(e)}")


    def call_llm(self):
        self.pic_combine()  

        combined_path = os.path.join(COMBINE_PIC_PATH, "combined.jpg")
        if not os.path.exists(combined_path):
            self.get_logger().warn("找不到合成圖片")
            return
        combined_cv2 = cv2.imread(combined_path)
        ros_image = self.bridge.cv2_to_imgmsg(combined_cv2, encoding='bgr8')
        ros_image.header.stamp = self.get_clock().now().to_msg()
        ros_image.header.frame_id = "combined_picture"

        if not os.path.exists(MEDICINE_INFO_PATH):
            self.get_logger().error("找不到 YAML 路徑")
            return
        with open(MEDICINE_INFO_PATH, 'r', encoding='utf-8') as f:
            drug_yaml = f.read()

        req = DrugIdentify.Request()
        req.image = ros_image
        req.drug_yaml = drug_yaml

        future = self.llm_client.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        res = future.result()
        self.get_logger().info(f"LLM 回傳結果: {res.result}")
        # def handle_llm_response(future):
        #     try:
        #         res = future.result()
        #         self.get_logger().info(f"LLM 回傳結果: {res.result}")
        #     except Exception as e:
        #         self.get_logger().error(f"LLM callback 發生錯誤: {str(e)}")

        # future.add_done_callback(handle_llm_response)
                          

    def call_and_wait_camera2(self) -> bool:
        # second_camera_point = [-2.97,-502.80,632.32,178.79,0.75,-0.89]
        # camera_point = [-215.405,804.799,138.406,0.168,-87.438]
        # second_camera_point = [-318.7, -357.26, 650.05, 152.85, -0.26, -89.987]
        # second_camera_point = [-18.9, -498.5, 529.99,178.99 , 0.37, -4.12]
        # second_camera_point = [-45.19, -342.78, 522.97, 127.31, 2.07, 1.36]
        # x, y, z = second_camera_point[:3]
        # x, y, z = [v / 1000.0 for v in second_camera_point[:3]]
        # roll, pitch, yaw = second_camera_point[3:]
        # rotation = R.from_euler('xyz', [roll, pitch, yaw], degrees=True)
        # quat = rotation.as_quat()
        # self.get_logger().info("Published cube_position pose:\n{}".format(quat))
        # pose_msg = Pose()
        # pose_msg.position = Point(x=x, y=y, z=z)
        # pose_msg.orientation = Quaternion(x=quat[0], y=quat[1], z=quat[2], w=quat[3])
        # self.curobo_pose.publish(pose_msg)
        # self.get_logger().info("Published cube_position pose:\n{}".format(pose_msg))


       #==============request service===============
        req = SecondCamera.Request()
        future = self.second_camera_client.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        res = future.result()
        if res.success:
            self.get_logger().info("SecondCamera finish combine picture")
        else:
            self.get_logger().warn("SecondCamera 啟動失敗")
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


    def call_and_wait_slider(self, code: int) -> bool:
        req = RailControl.Request()
        req.opt_code = code
        req.rail_name = "arm_rail1"
        future = self.slider_client.call_async(req)
        # future.add_done_callback(self.slider_callback)
        rclpy.spin_until_future_complete(self, future)
        res = future.result()
        if res.result >= 0:
            self.get_logger().info(f"slider opt: {code} Success: {res.result}")
            return True
        else:
            self.get_logger().error(f"slider opt: {code} Fail: {res.result}")
            return False


    def call_and_wait_grab(self, bbox, mask_img) -> bool:
        req = CaptureImage.Request()
        req.mask = self.bridge.cv2_to_imgmsg(mask_img.astype(np.uint8), encoding='mono8')
        req.bbox = bbox
        self.get_logger().info("呼叫 grab_detect（mask 傳入中）...")
        future = self.grab_client.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        res = future.result()
        if res.success:
            self.get_logger().info("抓取位置已處理成功")
            return True
        else:
            self.get_logger().warn("抓取服務回傳失敗")
            return False


    def spin_until_grab_finish(self):
        while not self.grab_finished:
            rclpy.spin_once(self)

    
    def socket_receive(self, host, port):
        self.host = host
        self.port = port
        threading.Thread(target=self.start_service,daemon=True).start()
    



    def curobo_point_and_wait(self, pose: list[float]) -> None:
        self.leave_curobo.publish(Bool(data=False))
        ## clear last point
        pose_msg = Pose()
        pose_msg.position = Point(x=0.0,y=0.0,z=40.0)
        pose_msg.orientation = Quaternion(x=0.0,y=0.0,z=0.0,w=1.0)
        self.curobo_pose.publish(pose_msg)
        ## normal point
        x,y,z = pose[:3]
        roll,pitch,yaw = pose[3:]
        quat = R.from_euler('xyz',[roll,pitch,yaw],degrees=True).as_quat()
        pose_msg = Pose()
        pose_msg.position = Point(x=x,y=y,z=z)
        pose_msg.orientation = Quaternion(x=quat[0],y=quat[1],z=quat[2],w=quat[3])
        self.curobo_pose.publish(pose_msg)
        self.get_logger().info(f"Published cube_position pose:\n{pose_msg}")
        time.sleep(2)
        while True:
            # print("poll")
            future = self.curobo_state_client.call_async(Trigger.Request())
            rclpy.spin_until_future_complete(self, future)
            res = future.result()
            if res.success == True:
                self.get_logger().info(f"curobo point finish")
                self.leave_curobo.publish(Bool(data=True))
                break
            time.sleep(0.1)




def main(args=None):
    YAML_PATH = '/workspace/tm_robot/src/tm_robot_main/tm_robot_main/robot_base_point.yaml'
    named_pose: dict = None
    with open(YAML_PATH, 'r', encoding='utf-8') as f:
        named_pose = yaml.safe_load(f)
    if named_pose is None:
        raise("cannot find yaml")

    rclpy.init(args=args)
    node = MainControl()
    node.create_rate(100)
    while rclpy.ok():
        rclpy.spin_once(node, timeout_sec=0.1)
        # node.spin_until_sct_listen_flag()
        # print("reach listen")
        # time.sleep(3)
        # node.send_tm_flow_leave_node()
        order = node.get_order()
        if order is not None:
            print("new order")
            # if not node.call_and_wait_slider(0):
            #     node.get_logger().error("call slider with 0 return failed")
            #     continue
            # if not node.call_and_wait_slider(1):
            #     node.get_logger().error("call slider with 1 return failed")
            #     continue
            ## tm flow: med_box
            # if not node.send_tm_flow_mode("medicine_box", "-1"):
            #     node.get_logger().error("call tm_flow_mode return failed")
            # ## check SCT
            # node.spin_until_sct_listen_flag()
            # ## groundSAM
            # gsam_result =  node.call_and_wait_groundsam2()
            # if gsam_result is None:
            #     node.get_logger().error("call GroundedSAM2 return failed")
            #     continue
            # bbox, mask_img = gsam_result
            # ## grab
            # if not node.call_and_wait_grab(bbox, mask_img):
            #     node.get_logger().error("call grab return failed")
            #     continue
            # ## wait move finish
            # input("Press Enter to when move finish...")
            # ## tm flow leave
            # node.send_tm_flow_leave_node()
            # ## tm flow: grab_and_check
            # if not node.send_tm_flow_mode("grab_and_check", "0"):
            #     node.get_logger().error("call tm_flow_mode return failed")
            # ## check SCT
            # node.spin_until_sct_listen_flag()
            # ## move to second camera
            node.curobo_point_and_wait(named_pose["rail_photo"]["rail_photo"])
            print("1")
            node.send_tm_flow_leave_node()
            print("2")
            node.spin_until_sct_listen_flag()
            print("3")
            node.curobo_point_and_wait(named_pose["rail_photo"]["rail_photo"])
            print("4")
            node.send_tm_flow_leave_node()
            print("finish job")
            # node.call_and_wait_camera2()
            # node.call_llm()
            # node.call_slider(2)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()