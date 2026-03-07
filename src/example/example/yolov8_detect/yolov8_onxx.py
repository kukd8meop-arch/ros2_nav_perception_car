import cv2
import numpy as np
import onnxruntime
import time
import random


class Colors:
    # Ultralytics color palette https://ultralytics.com/
    def __init__(self):
        # hex = matplotlib.colors.TABLEAU_COLORS.values()
        hex = ('DC143C', '7FFF00', 'FF1493', '7CFC00', 'CFD231', '48F90A', '92CC17', '3DDB86', '1A9334', '00D4BB',
               '2C99A8', '00C2FF', '344593', '6473FF', '0018EC', '8438FF', '520085', 'CB38FF', 'FF95C8', 'FF37C7')
        self.palette = [self.hex2rgb('#' + c) for c in hex]
        self.n = len(self.palette)

    def __call__(self, i, bgr=False):
        c = self.palette[int(i) % self.n]
        return (c[2], c[1], c[0]) if bgr else c

    @staticmethod
    def hex2rgb(h):  # rgb order (PIL)
        return tuple(int(h[1 + i:1 + i + 2], 16) for i in (0, 2, 4))

colors = Colors()  # create instance for 'from utils.plots import colors'

def plot_one_box(x, img, color=None, label=None, line_thickness=None):
    """
    description: Plots one bounding box on image img,
                 this function comes from YoLo11 project.
    param:
        x:       a box likes [x1,y1,x2,y2]
        img:     a opencv image object
        color:   color to draw rectangle, such as (0,255,0)
        label:   str
        line_thickness: int
    return:
        no return

    """
    tl = (
            line_thickness or round(0.002 * (img.shape[0] + img.shape[1]) / 2) + 1
    )  # line/font thickness
    color = color or [random.randint(0, 255) for _ in range(3)]
    c1, c2 = (int(x[0]), int(x[1])), (int(x[2]), int(x[3]))
    cv2.rectangle(img, c1, c2, color, thickness=tl, lineType=cv2.LINE_AA)
    if label:
        tf = max(tl - 1, 1)  # font thickness
        t_size = cv2.getTextSize(label, 0, fontScale=tl / 3, thickness=tf)[0]
        c2 = c1[0] + t_size[0], c1[1] - t_size[1] - 3
        cv2.rectangle(img, c1, c2, color, -1, cv2.LINE_AA)  # filled
        cv2.putText(
            img,
            label,
            (c1[0], c1[1] - 2),
            0,
            tl / 3,
            [225, 255, 255],
            thickness=tf,
            lineType=cv2.LINE_AA,
        )

        
class YOLOv8ONNXInference:
    def __init__(self, onnx_model_path, class_names, input_shape=(640, 640), 
                 conf_threshold=0.25, iou_threshold=0.45):
        """
        初始化 YOLOv8 ONNX 推理器。

        Args:
            onnx_model_path (str): ONNX 模型文件的路径。
            class_names (tuple/list): 类别名称列表。
            input_shape (tuple): 模型预期的输入尺寸 (H, W)。
            conf_threshold (float): 置信度阈值。
            iou_threshold (float): NMS IoU 阈值。
        """
        self.onnx_model_path = onnx_model_path
        self.class_names = class_names
        self.input_shape = input_shape
        self.conf_threshold = conf_threshold
        self.iou_threshold = iou_threshold
        
        self.session = None
        self.input_name = None
        self.output_name = None
        self.class_color_map = {} # 用于存储每个类别的随机颜色

        self._load_model()

    def _load_model(self):
        """加载 ONNX Runtime 会话（纯 CPU）。"""
        print(f"Loading ONNX model from: {self.onnx_model_path}...")
        try:
            # 明确指定只使用 CPUExecutionProvider
            self.session = onnxruntime.InferenceSession(self.onnx_model_path, 
                                                        providers=['CPUExecutionProvider'])
            self.input_name = self.session.get_inputs()[0].name
            self.output_name = self.session.get_outputs()[0].name
            print("ONNX Runtime session created successfully using CPUExecutionProvider.")
            print(f"Model Input Name: {self.input_name}, Output Name: {self.output_name}")
        except Exception as e:
            print(f"Error loading ONNX model: {e}")
            print("Please ensure the ONNX model path is correct and ONNX Runtime (CPU version) is installed.")
            exit() # 模型加载失败，程序退出

    def _preprocess_image(self, image_bgr):
        """
        预处理图片，使其符合 YOLOv8 ONNX 模型的输入要求 (N, C, H, W)。
        """
        input_h, input_w = self.input_shape
        img_h, img_w = image_bgr.shape[:2]

        img_rgb = cv2.cvtColor(image_bgr, cv2.COLOR_BGR2RGB)
        scale = min(input_h / img_h, input_w / img_w)
        resized_w, resized_h = int(img_w * scale), int(img_h * scale)
        resized_img = cv2.resize(img_rgb, (resized_w, resized_h), interpolation=cv2.INTER_AREA)

        padded_img = np.full((input_h, input_w, 3), 128, dtype=np.uint8) # 填充为灰色
        padded_img[:resized_h, :resized_w] = resized_img

        input_tensor = padded_img.astype(np.float32) / 255.0
        input_tensor = np.transpose(input_tensor, (2, 0, 1)) # HWC -> CHW
        input_tensor = np.expand_dims(input_tensor, axis=0)  # CHW -> NCHW (batch size 1)

        return input_tensor, scale, img_w, img_h

    def _postprocess_output(self, onnx_output, input_scale, original_img_shape):
        """
        后处理 YOLOv8 ONNX 模型的输出，包括 NMS。
        返回 boxes, scores, class_ids 列表。
        """
        # 确保输出是 (num_boxes, 4 + num_classes) 的形式
        # 根据你的模型输出形状 (1, 10, num_anchors)，进行转置
        if onnx_output.shape[1] == 10 and len(onnx_output.shape) == 3: 
            output = onnx_output[0].T # (1, 10, num_anchors) -> (num_anchors, 10)
        elif len(onnx_output.shape) == 2: # 已经是 (num_anchors, 10)
            output = onnx_output
        else:
            raise ValueError(f"Unexpected ONNX output shape: {onnx_output.shape}. Expected (1, 10, num_anchors) or (num_anchors, 10).")

        boxes_raw = output[:, :4]  # (cx, cy, w, h)
        scores_raw = output[:, 4:] # 类别置信度 (num_anchors, num_classes)

        # 将中心点坐标转换为左上角和右下角坐标 (x1, y1, x2, y2)
        x1 = boxes_raw[:, 0] - boxes_raw[:, 2] / 2
        y1 = boxes_raw[:, 1] - boxes_raw[:, 3] / 2
        x2 = boxes_raw[:, 0] + boxes_raw[:, 2] / 2
        y2 = boxes_raw[:, 1] + boxes_raw[:, 3] / 2
        boxes_xyxy = np.stack([x1, y1, x2, y2], axis=1)

        # 获取每个框的最高置信度及其对应的类别ID
        max_scores = np.max(scores_raw, axis=1)
        class_ids = np.argmax(scores_raw, axis=1)

        # 过滤掉低置信度的框
        mask = max_scores > self.conf_threshold
        filtered_boxes = boxes_xyxy[mask]
        filtered_scores = max_scores[mask]
        filtered_class_ids = class_ids[mask]

        if filtered_boxes.shape[0] == 0:
            return [], [], [] # 没有检测结果

        # 映射回原始图像尺寸
        original_boxes = filtered_boxes / input_scale
        # 裁剪到原始图像边界，确保坐标在图像范围内
        original_boxes[:, 0] = np.clip(original_boxes[:, 0], 0, original_img_shape[1])
        original_boxes[:, 1] = np.clip(original_boxes[:, 1], 0, original_img_shape[0])
        original_boxes[:, 2] = np.clip(original_boxes[:, 2], 0, original_img_shape[1])
        original_boxes[:, 3] = np.clip(original_boxes[:, 3], 0, original_img_shape[0])

        # 准备 NMS 输入 (OpenCV NMSBoxes 需要 (x1, y1, width, height) 格式)
        nms_boxes = original_boxes.copy()
        nms_boxes[:, 2] = nms_boxes[:, 2] - nms_boxes[:, 0] # width
        nms_boxes[:, 3] = nms_boxes[:, 3] - nms_boxes[:, 1] # height

        indices = cv2.dnn.NMSBoxes(nms_boxes.tolist(), filtered_scores.tolist(), 
                                   self.conf_threshold, self.iou_threshold)

        final_boxes = []
        final_scores = []
        final_class_ids = []

        if len(indices) > 0:
            for i in indices.flatten():
                final_boxes.append(original_boxes[i].tolist())
                final_scores.append(filtered_scores[i].item())
                final_class_ids.append(int(filtered_class_ids[i].item()))
                
        return final_boxes, final_scores, final_class_ids

    def infer_frame(self, frame_bgr):
        """
        对单个视频帧进行推理。

        Args:
            frame_bgr (np.array): OpenCV 读取的 BGR 格式的视频帧。

        Returns:
            tuple: (boxes, scores, class_ids)
                boxes (list): 检测到的边界框列表 [[x1, y1, x2, y2], ...]
                scores (list): 对应的置信度列表 [score1, score2, ...]
                class_ids (list): 对应的类别ID列表 [id1, id2, ...]
        """
        input_tensor, scale, original_w, original_h = self._preprocess_image(frame_bgr)
        
        onnx_output = self.session.run([self.output_name], {self.input_name: input_tensor})[0]
        
        boxes, scores, class_ids = self._postprocess_output(onnx_output, scale, (original_h, original_w))
        
        return boxes, scores, class_ids

    def get_color(self, class_id):
        """
        为每个类别生成并存储一个随机 BGR 颜色。
        """
        if class_id not in self.class_color_map:
            # 生成随机 BGR 颜色
            self.class_color_map[class_id] = (random.randint(0, 255), random.randint(0, 255), random.randint(0, 255))
        return self.class_color_map[class_id]

# --- 主程序逻辑 ---
def main():
    # 配置参数
    ONNX_MODEL_PATH = 'best_traffic.onnx' # 你的 ONNX 模型路径
    # 你的模型训练的类别名称列表（非常重要，请根据你的模型实际情况修改！）
    # 你的模型输出是 10 个属性（4个bbox + 6个类别），所以有 6 个类别
    CLASS_NAMES = ('go', 'right', 'park', 'red', 'green', 'crosswalk')
    
    CONF_THRESHOLD = 0.25 # 置信度阈值
    IOU_THRESHOLD = 0.45  # NMS IoU 阈值
    MODEL_INPUT_SHAPE = (640, 640) # 模型导出时的输入尺寸 (H, W)

    # 初始化 YOLOv8 ONNX 推理器
    yolo_infer = YOLOv8ONNXInference(
        onnx_model_path=ONNX_MODEL_PATH,
        class_names=CLASS_NAMES,
        input_shape=MODEL_INPUT_SHAPE,
        conf_threshold=CONF_THRESHOLD,
        iou_threshold=IOU_THRESHOLD
    )

    # 1. 打开摄像头 (0 表示默认摄像头，如果你的摄像头不是0，请修改)
    # 你也可以在这里传入视频文件路径，例如: cap = cv2.VideoCapture('your_video.mp4')
    cap = cv2.VideoCapture(0) 

    if not cap.isOpened():
        print("Error: Could not open video stream. Please check camera index or video path.")
        return

    print("Starting video inference. Press 'q' to quit.")

    frame_count = 0
    start_time = time.time()
    fps = 0 # 初始化FPS变量

    while True:
        ret, frame = cap.read()
        if not ret:
            print("End of video stream or error reading frame.")
            break

        # 2. 进行推理
        # frame 是 OpenCV 读取的 BGR 图像
        boxes, scores, class_ids = yolo_infer.infer_frame(frame)

        # 3. 绘制检测结果
        for box, score, class_id in zip(boxes, scores, class_ids):
            x1, y1, x2, y2 = map(int, box)
            label = yolo_infer.class_names[class_id] if class_id < len(yolo_infer.class_names) else f"Class {class_id}"
            
            # 获取当前类别的颜色
            color = yolo_infer.get_color(class_id) 
            
            # 使用 plot_one_box 绘制
            plot_one_box(box, frame, color=color, label=f'{label} {score:.2f}')

        # 计算并显示 FPS
        frame_count += 1
        if frame_count % 30 == 0: # 每30帧计算一次FPS
            end_time_fps = time.time()
            fps = 30 / (end_time_fps - start_time)
            print(f"FPS: {fps:.2f}")
            start_time = time.time()
        
        cv2.putText(frame, f"FPS: {fps:.2f}", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)


        # 4. 显示实时视频流
        cv2.imshow("YOLOv8 ONNX Video Inference (CPU)", frame)

        # 按 'q' 键退出
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    # 释放资源
    cap.release()
    cv2.destroyAllWindows()
    print("\nVideo inference (CPU version) finished.")

if __name__ == "__main__":
    main()