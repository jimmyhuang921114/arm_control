import rclpy
from rclpy.node import Node
import cv2
import numpy as np
from sensor_msgs.msg import Image
from std_msgs.msg import Header,Bool
from tm_robot_if.srv import GroundedSAM2Interface,PoseSrv,CaptureImage
# from tm_robot_if.srv import GroundedSAM2Interface, Paddle, MedicineOrder, SecondCheck,CapturePicture,SliderControl
from cv_bridge import CvBridge
import os
import yaml
from geometry_msgs.msg import Pose
import time


MED_INFO_PATH= "/workspace/tm_robot/src/tm_robot_main/tm_robot_main/medicine.yaml"
WORKFLOW_PATH= "/workspace/tm_robot/src/tm_robot_main/tm_robot_main/workflow.yaml"



class MainControl(Node):
    def __init__(self):
        super().__init__("main_control")

        # 訂閱彩色與深度影像
        self.color_image = self.create_subscription(Image, '/tm_robot/color_image', self.color_image_callback, 10)
        self.depth_image = self.create_subscription(Image, '/tm_robot/depth_image', self.depth_image_callback, 10)
        self.curobo_state = self.create_subscription(Bool, '/curobo/state', self.curobo_state_callback, 10)
        self.io_state = self.create_subscription(Bool, '/io/state', self.io_state_callback, 10)
        self.slider_state = self.create_subscription(Bool, '/slider/state', self.slider_state_callback, 10)

        #curobo control 
        # self.curobo_control = self.create_publisher(Pose, '/curobo/control', 10)

        # 建立服務 client
        self.detect_client = self.create_client(GroundedSAM2Interface, 'grounded_sam2')
        self.grab_client = self.create_client(CaptureImage,'grab_detect')
        # self.ocr_client = self.create_client(Paddle, 'ocr')
        # self.llm_client = self.create_client(DetectImage, 'llm')
        # self.second_camera_client = self.create_client(SecondCamera, 'camera2')
        # self.slider_client = self.create_client(SliderControl,'slider')
        # self.order_client = self.create_client(, 'order')
        # self.order_report_client = self.create_client(, 'order_report')

        # 建立 GroundedSAM2 服務 client
        self.service_clients = {
            "grounded_sam2": self.detect_client,
            # "ocr": self.ocr_client,
            # "llm": self.llm_client,
            # "camera2": self.second_camera_client,
            "grab_detect": self.grab_client
            # "curobo_control"
            # "slider": self.slider_client,
            # "order": self.order_client,
            # "order_report": self.order_report_client

        }

        for name, client in self.service_clients.items():
            while not client.wait_for_service(timeout_sec=1.0):
                self.get_logger().info(f"等待服務 [{name}] 啟動中...")
        
        self.get_logger().info("所有服務已啟動，開始流程")

        self.bridge = CvBridge()
        self.color_image_np = None
        self.depth_image_np = None
        self.mask_img = None
        self.curobo_state_flag = False
        self.has_run = False
        # 載入 YAML 檔案
        # if os.path.exists(YAML_PATH):
            # with open(YAML_PATH, 'r', encoding='utf-8') as f:
                # self.yaml_text = f.read()
        # else:
            # self.get_logger().error(f"找不到 YAML 檔案：{YAML_PATH}")

        # self.create_timer(1.0, self.main_loop)
        self.timer = self.create_timer(1.0, self.run_once)


    # def main_loop(self):
        # if self.curobo_state_flag:
        # self.call_groundsam2()
        # self.call_ocr()
        # self.call_llm()
        # self.call_camera2()
        # else:
            # self.get_logger().info("等待 curobo 準備好...")
    def run_once(self):
        # if self.has_run == True:
        #     return 
        self.has_run = True
        self.call_groundsam2()

        # 呼叫完畢後取消 timer，避免未來再次執行
        self.timer.cancel()
        self.destroy_timer(self.timer)

                
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
    


    # def call_groundsam2(self):
    #     if self.color_image_np is None or self.depth_image_np is None:
    #         self.get_logger().warn("color 或 depth 尚未準備好，略過 GroundedSAM2 呼叫")
    #         return
    #     time.sleep(2)
    #     req = GroundedSAM2Interface.Request()
    #     ros_color_image = self.bridge.cv2_to_imgmsg(self.color_image_np, encoding='bgr8')
    #     ros_depth_image = self.bridge.cv2_to_imgmsg(self.depth_image_np, encoding='passthrough')

    #     now = self.get_clock().now().to_msg()
    #     ros_color_image.header = Header(stamp=now, frame_id='camera')
    #     ros_depth_image.header = Header(stamp=now, frame_id='camera')

    #     req.image = ros_color_image
    #     req.depth = ros_depth_image
    #     req.prompt = 'medicine'
    #     req.confidence_threshold = 0.2
    #     req.size_threshold = 0.1
    #     req.selection_mode = "closest"

    #     self.get_logger().info("呼叫 grounded_sam2（同步）...")
    #     future = self.detect_client.call_async(req)
    #     rclpy.spin_until_future_complete(self, future)

    #     if future.done():
    #         try:
    #             res = future.result()
    #             self.get_logger().info(f"辨識結果: label={res.label}, score={res.score}, bbox={res.bbox}")
    #             if res.binary_image.data:
    #                 self.mask_img = self.bridge.imgmsg_to_cv2(res.binary_image, desired_encoding='mono8')
    #                 cv2.imshow("Selected Binary Mask", self.mask_img)
    #                 cv2.waitKey(1)
    #         except Exception as e:
    #             self.get_logger().error(f"GroundedSAM2 呼叫失敗: {str(e)}")

    def call_groundsam2(self):
        if self.color_image_np is None or self.depth_image_np is None:
            self.get_logger().warn("color 或 depth 尚未準備好，略過 GroundedSAM2 呼叫")
            return
        req = GroundedSAM2Interface.Request()
        ros_color_image = self.bridge.cv2_to_imgmsg(self.color_image_np, encoding='bgr8')
        ros_depth_image = self.bridge.cv2_to_imgmsg(self.depth_image_np, encoding='passthrough')

        now = self.get_clock().now().to_msg()
        ros_color_image.header = Header(stamp=now, frame_id='camera')
        ros_depth_image.header = Header(stamp=now, frame_id='camera')

        req.image = ros_color_image
        req.depth = ros_depth_image
        req.prompt = 'medicine'
        req.confidence_threshold = 0.2
        req.size_threshold = 0.1
        req.selection_mode = "closest"

        future = self.detect_client.call_async(req)
        future.add_done_callback(self.handle_groundsam2_response)

    def handle_groundsam2_response(self, future):
        try:
            res = future.result()
            if res is None:
                self.get_logger().error("GroundedSAM2 回傳為 None")
                return

            self.get_logger().info(f"辨識結果: label={res.label}, score={res.score}, bbox={res.bbox}")

            if not res.binary_image.data:
                self.get_logger().warn("回傳的 binary_image 是空的")
                return

            
            mask_raw = self.bridge.imgmsg_to_cv2(res.binary_image, desired_encoding='mono8')
            self.mask_img = (mask_raw > 0).astype(np.uint8) * 255
            mask_raw = self.bridge.imgmsg_to_cv2(res.binary_image, desired_encoding='mono8')
            print("原始遮罩 unique 值:", np.unique(mask_raw))  

            self.mask_img = (mask_raw > 0).astype(np.uint8) * 255
            print("二值化遮罩 unique 值:", np.unique(self.mask_img))  

            nonzero = np.count_nonzero(self.mask_img)
            self.get_logger().info(f"遮罩非零像素數量: {nonzero}")

            if nonzero == 0:
                self.get_logger().warn("遮罩為全黑，無有效區域")
            else:
                self.get_logger().info("遮罩有效，可用於後續步驟")
            cv2.imwrite("/workspace/tm_robot/src/visuial/visuial/mask_raw.png", mask_raw)
            cv2.imwrite("/workspace/tm_robot/src/visuial/visuial/mask_output.png", self.mask_img)

            if self.color_image_np is not None:
                overlay = self.color_image_np.copy()
                mask_bin = (self.mask_img > 0).astype(np.uint8)
                mask_color = cv2.applyColorMap(mask_bin * 255, cv2.COLORMAP_JET)
                alpha = 0.5
                for c in range(3):
                    overlay[:, :, c] = np.where(
                        mask_bin,
                        cv2.addWeighted(overlay[:, :, c], 1 - alpha, mask_color[:, :, c], alpha, 0),
                        overlay[:, :, c]
                    )


            # 顯示 label 文字
            for lbl in res.label:
                self.get_logger().info(f"辨識到標籤: {lbl}")
            if nonzero != 0:
                self.get_logger().info("遮罩有效，呼叫抓取服務")
                self.call_grab()


        except Exception as e:
            self.get_logger().error(f"GroundedSAM2 處理 callback 發生錯誤: {str(e)}")
    
    def call_grab(self):
        req =PoseSrv()


    def call_ocr(self):
        req = Paddle.Request()
        req.image_data = "image_data_placeholder"
        future = self.ocr_client.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        if future.done():
            res = future.result()
            self.get_logger().info(f"OCR 結果: {res.text}")
    
    def call_llm(self):
        req = DetectImage.Request()
        req.filename = "example.jpg"
        future = self.llm_client.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        if future.done():
            res = future.result()
            self.get_logger().info(f"LLM 回應: success={res.success}")

    def call_camera2(self):
        req = SecondCheck.Request()
        req.input = "check"
        future = self.second_camera_client.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        if future.done():
            res = future.result()
            self.get_logger().info(f"Camera2 檢查結果: {res.result}")

    def call_slider(self):
        req = SliderControl.Request()
        req.target = "left"
        future = self.slider_client.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        if future.done():
            res = future.result()
            self.get_logger().info(f"Slider 控制狀態: {res.status}")

    def call_grab(self):
        if self.mask_img is None:
            self.get_logger().error("抓取失敗：尚未獲得有效遮罩")
            return

        req = CaptureImage.Request()
        req.mask = self.bridge.cv2_to_imgmsg(self.mask_img.astype(np.uint8), encoding='mono8')

        self.get_logger().info("呼叫 grab_detect（mask 傳入中）...")
        future = self.grab_client.call_async(req)

        def grab_callback(future):
            try:
                res = future.result()
                if res.success:
                    self.get_logger().info("抓取位置已處理成功")
                else:
                    self.get_logger().warn("抓取服務回傳失敗")
            except Exception as e:
                self.get_logger().error(f"抓取服務失敗: {str(e)}")

        future.add_done_callback(grab_callback)



def main(args=None):
    rclpy.init(args=args)
    node = MainControl()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
