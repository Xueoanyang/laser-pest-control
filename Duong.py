# -*- coding: utf-8 -*-
"""
YOLO RKNN 量化模型部署程序 (RV1126B)
--------------------------------------
适配 i8 量化模型，修正 9 输出后处理，整合性能优化。

关键变更相对原版:
  1. 输入改为 uint8 直传 (量化模型归一化已内嵌, 无需 astype(float32))
  2. 实现 9 输出特征图 -> 框/分数/类别的完整 YOLO 后处理 (解码 + NMS)
  3. 预分配推理输入缓冲区, 避免每帧 GC
  4. 显示降频 + 简化绘制, 大幅降低 CPU 占用
  5. 分段计时, 便于定位瓶颈
  6. 保留: 双目标跟踪 / IIR 滤波 / 跳变限幅 / 串口双坐标上报
  7. ★ 丢失期间发送最后有效坐标, 超过 MAX_LOST_FRAMES 才发 -1

注意: box 解码方式取决于你的 YOLO 训练版本。默认采用 YOLOv5 风格 (带 anchor)。
      若框位置明显错位, 请调整 DECODE_MODE 或 ANCHORS, 见下方注释。
"""

import cv2
import time
import queue
import threading
import numpy as np
import serial
from rknnlite.api import RKNNLite

# ============================================================
#  配置参数
# ============================================================
RKNN_MODEL_PATH = "./best_72.rknn"   # 量化后的 rknn 模型
CAMERA_DEVICE = "/dev/video52"
SERIAL_PORT = "/dev/ttyS5"
SERIAL_BAUDRATE = 115200

# 检测参数
CONF_THRESH = 0.373          # 主目标置信度阈值
DOT_CONF_THRESH = 0.001      # dot 目标置信度阈值 (独立)
IOU_THRESH = 0.45            # NMS IoU 阈值
INPUT_SIZE = 640             # 模型输入尺寸

# 显示尺寸 (小屏适配)
SCREEN_WIDTH = 480
SCREEN_HEIGHT = 320
DISPLAY_EVERY = 5            # 每 N 帧显示一次, 降低 GUI 开销 (1=每帧, 999=关闭)

# 平滑滤波参数 (一阶 IIR)
FILTER_ALPHA = 0.30
JUMP_THRESH = 300            # 单帧跳变限幅 (像素)

# 帧率控制
TARGET_FPS = 25
FRAME_INTERVAL = 1.0 / TARGET_FPS

# 目标丢失检测
MAX_LOST_FRAMES = 20
DOT_LOST_FRAMES = 20

# 类别定义
CLASS_NAMES = [ "fly", "dot"]
NUM_CLASSES = len(CLASS_NAMES)
DOT_CLASS_ID = 1

# ============================================================
#  YOLO 后处理配置 (★ 需根据你的模型版本确认 ★)
# ============================================================
# 输出顺序 (与 ONNX 结构图一致, 3 个尺度, 每尺度 box/cls/obj)
# scale0 (80x80, stride=8):  output(4ch box), output1(3ch cls), output2(1ch obj)
# scale1 (40x40, stride=16): output3, output4, output5
# scale2 (20x20, stride=32): output6, output7, output8
SCALES = [
    {"stride": 8,  "box": 0, "cls": 1, "obj": 2},
    {"stride": 16, "box": 3, "cls": 4, "obj": 5},
    {"stride": 32, "box": 6, "cls": 7, "obj": 8},
]

# 解码模式:
#   'v5' : YOLOv5 风格, xy=(sigmoid*2-0.5+grid)*stride, wh=(sigmoid*2)^2*anchor
#   'v8' : YOLOv8 风格 (anchor-free), xy=(sigmoid*2-0.5+grid)*stride, wh=(sigmoid*2)^2*stride
#   'dist' : dist2bbox, box=[l,t,r,b] DFL 距离, x1=(grid-l)*stride, x2=(grid+r)*stride
#   'direct' : 直接偏移, xy=(raw+grid)*stride, wh=raw*stride
# ★ 改用 dist (dist2bbox): box 输出是 DFL 距离 [l,t,r,b], 非 sigmoid.
#   v5 偏大(anchor 过大), v8 偏小(sigmoid 压缩了距离值), dist 直接用距离最准.
DECODE_MODE = 'dist'

# YOLOv5 anchor (仅 v5 模式使用, v8/direct 模式忽略)
ANCHORS = [
    [(10, 13)],            # P3/8
    [(30, 61)],            # P4/16
    [(116, 90)],           # P5/32
]

# ★ sigmoid 融合标志 (分 box 和 cls/obj 独立设置)
# DEBUG 显示: cls/obj 值域 0~1 (已融合), box 值域 0~15 (DFL 距离, 非 sigmoid)
SIGMOID_FUSED_CLS = True    # cls/obj 已融合 sigmoid (输出 0~1)
SIGMOID_FUSED_BOX = True    # box 是 DFL 距离值 (0~reg_max), 不需要 sigmoid

# ★ letterbox 预处理 (与 YOLO 训练默认一致, 保持宽高比 + 灰边填充)
# 摄像头 640x480 直接 resize 到 640x640 会拉伸 y 轴, 导致框不贴合.
# 设 True: 用 letterbox (上下补灰边); 设 False: 直接拉伸 (原行为).
USE_LETTERBOX = True

# 调试: 首帧打印各输出值域, 用于判断 sigmoid 是否融合 / 解码是否正确
DEBUG_RAW = True


# ============================================================
#  工具函数
# ============================================================
def sigmoid(x):
    return 1.0 / (1.0 + np.exp(-x))


def letterbox(img, canvas_buf, new_shape=640, color=(114, 114, 114)):
    """
    等比缩放 + 灰边填充到 new_shape×new_shape (写入预分配 canvas_buf, 避免每帧分配).
    返回: canvas_buf, ratio, dh, dw
    """
    h, w = img.shape[:2]
    r = min(new_shape / h, new_shape / w)
    nh, nw = int(round(h * r)), int(round(w * r))
    dh, dw = (new_shape - nh) // 2, (new_shape - nw) // 2
    resized = cv2.resize(img, (nw, nh), interpolation=cv2.INTER_LINEAR)
    canvas_buf[:] = color
    canvas_buf[dh:dh + nh, dw:dw + nw] = resized
    return canvas_buf, r, dh, dw


def nms(boxes, scores, iou_thresh):
    """numpy NMS, boxes 为 x1y1x2y2, 返回保留索引"""
    if len(boxes) == 0:
        return np.array([], dtype=int)
    x1 = boxes[:, 0]; y1 = boxes[:, 1]
    x2 = boxes[:, 2]; y2 = boxes[:, 3]
    areas = np.maximum(0.0, x2 - x1) * np.maximum(0.0, y2 - y1)
    order = scores.argsort()[::-1]

    keep = []
    while order.size > 0:
        i = order[0]
        keep.append(i)
        if order.size == 1:
            break
        xx1 = np.maximum(x1[i], x1[order[1:]])
        yy1 = np.maximum(y1[i], y1[order[1:]])
        xx2 = np.minimum(x2[i], x2[order[1:]])
        yy2 = np.minimum(y2[i], y2[order[1:]])
        w = np.maximum(0.0, xx2 - xx1)
        h = np.maximum(0.0, yy2 - yy1)
        inter = w * h
        ovr = inter / (areas[i] + areas[order[1:]] - inter + 1e-9)
        idx = np.where(ovr <= iou_thresh)[0]
        order = order[idx + 1]
    return np.array(keep, dtype=int)


def post_process(outputs, min_conf, iou_thresh):
    """
    9 输出特征图 -> boxes[N,4](x1y1x2y2, 640 坐标系), scores[N], class_ids[N]
    outputs: RKNNLite inference 返回的 list, 9 个 (1,C,H,W) 数组
    min_conf: 最低保留阈值 (取主/dot 阈值的较小者, 保证低分 dot 不被误删).
              类别各自的阈值在主程序中二次过滤.
    """
    all_boxes = []
    all_scores = []
    all_cls = []

    for si, sc in enumerate(SCALES):
        stride = sc["stride"]
        box_feat = outputs[sc["box"]][0]   # (4, H, W)
        cls_feat = outputs[sc["cls"]][0]   # (3, H, W)
        obj_feat = outputs[sc["obj"]][0]   # (1, H, W)
        _, H, W = box_feat.shape

        # 网格坐标
        gx, gy = np.meshgrid(np.arange(W, dtype=np.float32),
                             np.arange(H, dtype=np.float32))
        gx = gx.reshape(-1); gy = gy.reshape(-1)

        # 取出 4 个通道并展平: 顺序假设 [cx, cy, w, h]
        bx = box_feat[0].reshape(-1).astype(np.float32)
        by = box_feat[1].reshape(-1).astype(np.float32)
        bw = box_feat[2].reshape(-1).astype(np.float32)
        bh = box_feat[3].reshape(-1).astype(np.float32)

        obj = obj_feat[0].reshape(-1).astype(np.float32)        # (H*W,)
        cls = cls_feat.reshape(NUM_CLASSES, -1).T.astype(np.float32)  # (H*W, 3)

        # cls/obj: 若未融合则手动 sigmoid
        if not SIGMOID_FUSED_CLS:
            obj = sigmoid(obj)
            cls = sigmoid(cls)

        # box: 若未融合则手动 sigmoid (★ 关键修正: box 和 cls/obj 独立)
        if not SIGMOID_FUSED_BOX:
            bx = sigmoid(bx); by = sigmoid(by)
            bw = sigmoid(bw); bh = sigmoid(bh)

        if DECODE_MODE == 'v5':
            cx = (bx * 2.0 - 0.5 + gx) * stride
            cy = (by * 2.0 - 0.5 + gy) * stride
            aw, ah = ANCHORS[si][0]
            w = (bw * 2.0) ** 2 * aw
            h = (bh * 2.0) ** 2 * ah
            x1 = cx - w / 2.0; y1 = cy - h / 2.0
            x2 = cx + w / 2.0; y2 = cy + h / 2.0
        elif DECODE_MODE == 'v8':
            cx = (bx * 2.0 - 0.5 + gx) * stride
            cy = (by * 2.0 - 0.5 + gy) * stride
            w = (bw * 2.0) ** 2 * stride
            h = (bh * 2.0) ** 2 * stride
            x1 = cx - w / 2.0; y1 = cy - h / 2.0
            x2 = cx + w / 2.0; y2 = cy + h / 2.0
        elif DECODE_MODE == 'dist':
            # dist2bbox: box 4 通道 = [left, top, right, bottom] 距离 (DFL 输出)
            # 不需要 sigmoid, 直接用网格点 ± 距离 × stride
            x1 = (gx - bx) * stride
            y1 = (gy - by) * stride
            x2 = (gx + bw) * stride
            y2 = (gy + bh) * stride
        else:  # direct
            cx = (bx + gx) * stride
            cy = (by + gy) * stride
            w = bw * stride
            h = bh * stride
            x1 = cx - w / 2.0; y1 = cy - h / 2.0
            x2 = cx + w / 2.0; y2 = cy + h / 2.0

        boxes = np.stack([x1, y1, x2, y2], axis=1)

        # 分数 = obj * cls
        scores_all = obj[:, None] * cls                  # (N, 3)
        class_ids = np.argmax(scores_all, axis=1)
        scores = np.max(scores_all, axis=1)

        mask = scores > min_conf
        all_boxes.append(boxes[mask])
        all_scores.append(scores[mask])
        all_cls.append(class_ids[mask])

    if not all_boxes or sum(len(b) for b in all_boxes) == 0:
        return (np.zeros((0, 4), np.float32),
                np.zeros((0,), np.float32),
                np.zeros((0,), np.int32))

    boxes = np.concatenate(all_boxes, 0)
    scores = np.concatenate(all_scores, 0)
    class_ids = np.concatenate(all_cls, 0)

    # 按类别分别 NMS (类别间不互相抑制)
    keep_all = []
    for c in np.unique(class_ids):
        idx = np.where(class_ids == c)[0]
        k = nms(boxes[idx], scores[idx], iou_thresh)
        keep_all.append(idx[k])
    keep = np.concatenate(keep_all) if keep_all else np.array([], dtype=int)

    return boxes[keep], scores[keep], class_ids[keep]


# ============================================================
#  串口初始化
# ============================================================
def init_serial():
    try:
        ser = serial.Serial(SERIAL_PORT, SERIAL_BAUDRATE, timeout=0.1)
        print(f"[OK] Serial: {SERIAL_PORT} @ {SERIAL_BAUDRATE}")
        return ser
    except Exception as e:
        print(f"[ERR] Serial: {e}")
        return None


# ============================================================
#  目标跟踪器 (IIR + 跳变限幅 + 最后有效坐标缓存)
# ============================================================
class TargetTracker:
    def __init__(self):
        self.smooth_cx = None
        self.smooth_cy = None
        self.lost_count = 0
        self.prev_valid = False
        # ★ 修改1: 缓存最后有效坐标, 丢失期间发送该值而非立即发 -1
        self.last_main_cx = None
        self.last_main_cy = None

        self.dot_smooth_cx = None
        self.dot_smooth_cy = None
        self.dot_lost_count = 0
        self.dot_prev_valid = False
        # ★ 修改1: dot 同理
        self.last_dot_cx = None
        self.last_dot_cy = None

    def update(self, boxes, scores, class_ids):
        main_indices = [i for i, c in enumerate(class_ids) if c != DOT_CLASS_ID]
        dot_indices = [i for i, c in enumerate(class_ids) if c == DOT_CLASS_ID]

        main_boxes = boxes[main_indices] if main_indices else np.zeros((0, 4))
        main_scores = scores[main_indices] if main_indices else np.zeros((0,))
        main_cls = class_ids[main_indices] if main_indices else np.zeros((0,), np.int32)

        dot_boxes = boxes[dot_indices] if dot_indices else np.zeros((0, 4))
        dot_scores = scores[dot_indices] if dot_indices else np.zeros((0,))

        tracked, center, box, score, cls_id = self._update_main(main_boxes, main_scores, main_cls)
        dot_tracked, dot_center, dot_box, dot_score = self._update_dot(dot_boxes, dot_scores)
        return tracked, center, box, score, cls_id, dot_tracked, dot_center, dot_box, dot_score

    def _update_main(self, boxes, scores, class_ids):
        if len(boxes) == 0:
            self.lost_count += 1
            self.prev_valid = False
            return False, None, None, None, None
        best = np.argmax(scores)
        box = boxes[best].astype(int)
        score = scores[best]
        cls_id = int(class_ids[best])
        cx_raw = int((box[0] + box[2]) / 2)
        cy_raw = int((box[1] + box[3]) / 2)

        if self.prev_valid and self.smooth_cx is not None:
            dx = cx_raw - self.smooth_cx
            dy = cy_raw - self.smooth_cy
            if abs(dx) > JUMP_THRESH or abs(dy) > JUMP_THRESH:
                self.lost_count += 1
                return False, None, None, None, None

        if self.smooth_cx is None:
            self.smooth_cx, self.smooth_cy = cx_raw, cy_raw
        else:
            self.smooth_cx = int(self.smooth_cx * (1 - FILTER_ALPHA) + cx_raw * FILTER_ALPHA)
            self.smooth_cy = int(self.smooth_cy * (1 - FILTER_ALPHA) + cy_raw * FILTER_ALPHA)
        self.lost_count = 0
        self.prev_valid = True
        # ★ 修改2: 成功跟踪时更新最后有效坐标缓存
        self.last_main_cx = self.smooth_cx
        self.last_main_cy = self.smooth_cy
        return True, (self.smooth_cx, self.smooth_cy), box, score, cls_id

    def _update_dot(self, boxes, scores):
        if len(boxes) == 0:
            self.dot_lost_count += 1
            self.dot_prev_valid = False
            return False, None, None, None
        best = np.argmax(scores)
        box = boxes[best].astype(int)
        score = scores[best]
        cx_raw = int((box[0] + box[2]) / 2)
        cy_raw = int((box[1] + box[3]) / 2)

        if self.dot_prev_valid and self.dot_smooth_cx is not None:
            dx = cx_raw - self.dot_smooth_cx
            dy = cy_raw - self.dot_smooth_cy
            if abs(dx) > JUMP_THRESH or abs(dy) > JUMP_THRESH:
                self.dot_lost_count += 1
                return False, None, None, None

        if self.dot_smooth_cx is None:
            self.dot_smooth_cx, self.dot_smooth_cy = cx_raw, cy_raw
        else:
            self.dot_smooth_cx = int(self.dot_smooth_cx * (1 - FILTER_ALPHA) + cx_raw * FILTER_ALPHA)
            self.dot_smooth_cy = int(self.dot_smooth_cy * (1 - FILTER_ALPHA) + cy_raw * FILTER_ALPHA)
        self.dot_lost_count = 0
        self.dot_prev_valid = True
        # ★ 修改3: 成功跟踪时更新 dot 最后有效坐标缓存
        self.last_dot_cx = self.dot_smooth_cx
        self.last_dot_cy = self.dot_smooth_cy
        return True, (self.dot_smooth_cx, self.dot_smooth_cy), box, score

    def is_lost(self):
        return self.lost_count >= MAX_LOST_FRAMES

    def is_dot_lost(self):
        return self.dot_lost_count >= DOT_LOST_FRAMES


# ============================================================
#  串口发送
# ============================================================
def send_coords(ser, main_cx, main_cy, dot_cx, dot_cy):
    if ser is None:
        return
    try:
        if main_cx is None or main_cy is None:
            main_cx, main_cy = -1, -1
        if dot_cx is None or dot_cy is None:
            dot_cx, dot_cy = -1, -1
        msg = f"${main_cx},{main_cy},{dot_cx},{dot_cy}#"
        print(msg)
        ser.write(msg.encode())
        ser.flush()
    except Exception as e:
        print(f"[ERR] Serial send: {e}")


# ============================================================
#  多线程流水线: 采集预处理线程 / NPU推理线程 / 主线程(后处理+输出)
# ============================================================
def preprocess_one(frame, input_buf):
    """单帧预处理: letterbox + RGB, 写入 input_buf. (frame 已在采集线程翻转)"""
    h, w = frame.shape[:2]
    if USE_LETTERBOX:
        lb_r = min(INPUT_SIZE / h, INPUT_SIZE / w)
        nh, nw = int(round(h * lb_r)), int(round(w * lb_r))
        lb_dh = (INPUT_SIZE - nh) // 2
        lb_dw = (INPUT_SIZE - nw) // 2
        if lb_r == 1.0 and nh == h and nw == w:
            img_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        else:
            resized = cv2.resize(frame, (nw, nh), interpolation=cv2.INTER_LINEAR)
            img_rgb = cv2.cvtColor(resized, cv2.COLOR_BGR2RGB)
        input_buf[0, :] = 114
        input_buf[0, lb_dh:lb_dh + nh, lb_dw:lb_dw + nw] = img_rgb
    else:
        img = cv2.resize(frame, (INPUT_SIZE, INPUT_SIZE))
        img_rgb = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
        input_buf[0] = img_rgb
        lb_r, lb_dh, lb_dw = 1.0, 0, 0
    return lb_r, lb_dh, lb_dw


def capture_thread(cap, q_out, stop_event, input_bufs, t_cap_acc, t_pre_acc):
    """线程A: 采集 + 预处理 → q_out (双缓冲: 轮流用两个 input_buf)"""
    buf_idx = 0
    while not stop_event.is_set():
        t0 = time.time()
        ret, frame = cap.read()
        if not ret:
            break
        t_cap_acc[0] += time.time() - t0

        # 水平镜像 (在预处理前翻转, 保证 input_buf 和显示用 frame 一致)
        frame = cv2.flip(frame, 1)

        t0 = time.time()
        input_buf = input_bufs[buf_idx]
        lb_r, lb_dh, lb_dw = preprocess_one(frame, input_buf)
        t_pre_acc[0] += time.time() - t0

        try:
            q_out.put((input_buf, lb_r, lb_dh, lb_dw, frame), timeout=1.0)
            buf_idx = 1 - buf_idx   # 双缓冲轮换
        except queue.Full:
            pass
    stop_event.set()


def inference_thread(rknn, q_in, q_out, stop_event, t_npu_acc):
    """线程B: q_in → NPU推理 → q_out (NPU 工作线程, 释放 GIL 可与 CPU 并行)"""
    while not stop_event.is_set():
        try:
            item = q_in.get(timeout=1.0)
        except queue.Empty:
            continue
        if item is None:
            break
        input_buf, lb_r, lb_dh, lb_dw, frame = item
        t0 = time.time()
        try:
            outputs = rknn.inference(inputs=[input_buf])
        except Exception as e:
            print(f"[ERR] inference: {e}")
            stop_event.set()
            break
        t_npu_acc[0] += time.time() - t0
        try:
            q_out.put((outputs, lb_r, lb_dh, lb_dw, frame), timeout=1.0)
        except queue.Full:
            pass
    stop_event.set()


# ============================================================
#  主程序
# ============================================================
def main():
    global DEBUG_RAW
    print("=" * 50)
    print("YOLO RKNN 量化模型部署 (RV1126B) - 多线程流水线")
    print("=" * 50)
    print(f"[INFO] Model: {RKNN_MODEL_PATH}")
    print(f"[INFO] Classes: {CLASS_NAMES}")
    print(f"[INFO] Decode: {DECODE_MODE}, sigmoid_box={SIGMOID_FUSED_BOX}, sigmoid_cls={SIGMOID_FUSED_CLS}")
    print(f"[INFO] Main conf: {CONF_THRESH}, Dot conf: {DOT_CONF_THRESH}, IoU: {IOU_THRESH}")
    print("=" * 50)

    # 1. 串口
    ser = init_serial()

    # 2. RKNN
    rknn = RKNNLite()
    if rknn.load_rknn(RKNN_MODEL_PATH) != 0:
        print("[ERR] Load model failed"); return
    if rknn.init_runtime() != 0:
        print("[ERR] Init runtime failed"); return
    print("[OK] RKNN loaded")

    # 3. 摄像头
    cap = cv2.VideoCapture(CAMERA_DEVICE, cv2.CAP_V4L2)
    if not cap.isOpened():
        print("[ERR] Camera failed"); return
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
    print("[OK] Camera opened")

    # 4. 显示窗口
    cv2.namedWindow("Detection", cv2.WINDOW_NORMAL)
    cv2.setWindowProperty("Detection", cv2.WND_PROP_FULLSCREEN, cv2.WINDOW_FULLSCREEN)

    # 5. 跟踪器
    tracker = TargetTracker()

    # 6. 预分配双缓冲 input_buf (供采集线程轮换使用, 避免与 NPU 争用)
    input_bufs = [np.empty((1, INPUT_SIZE, INPUT_SIZE, 3), dtype=np.uint8) for _ in range(2)]

    # 7. 流水线队列 (深度 2, 双缓冲)
    q_pre2npu = queue.Queue(maxsize=2)   # 采集线程 → 推理线程
    q_npu2main = queue.Queue(maxsize=2)  # 推理线程 → 主线程
    stop_event = threading.Event()

    # 8. 各线程耗时统计 (用 list 传引用, 线程内累加)
    t_cap_acc = [0.0]; t_pre_acc = [0.0]; t_npu_acc = [0.0]

    # 9. 启动采集线程 + 推理线程
    th_cap = threading.Thread(target=capture_thread,
                              args=(cap, q_pre2npu, stop_event, input_bufs, t_cap_acc, t_pre_acc))
    th_npu = threading.Thread(target=inference_thread,
                              args=(rknn, q_pre2npu, q_npu2main, stop_event, t_npu_acc))
    th_cap.start(); th_npu.start()

    # 10. 主线程统计
    frame_count = 0
    fps = 0.0
    fps_start = time.time()
    t_post_acc = 0.0
    t_show_acc = 0.0

    print("\n[INFO] Detection started, press 'q' to exit\n")

    try:
        while not stop_event.is_set():
            # 从推理线程取结果 (outputs 已算完)
            try:
                item = q_npu2main.get(timeout=2.0)
            except queue.Empty:
                if stop_event.is_set():
                    break
                continue
            outputs, lb_r, lb_dh, lb_dw, frame = item

            # ---- 首帧调试 ----
            if DEBUG_RAW:
                DEBUG_RAW = False
                print("\n[DEBUG] 输出值域 (box 分通道 + cls/obj):")
                scale_names = ["80x80", "40x40", "20x20"]
                for si in range(3):
                    sn = scale_names[si]
                    bf = outputs[si * 3][0]
                    cf = outputs[si * 3 + 1][0]
                    of = outputs[si * 3 + 2][0]
                    print(f"  [{sn}] box ch0(cx): {bf[0].min():.3f}~{bf[0].max():.3f}")
                    print(f"  [{sn}] box ch1(cy): {bf[1].min():.3f}~{bf[1].max():.3f}")
                    print(f"  [{sn}] box ch2(w):  {bf[2].min():.3f}~{bf[2].max():.3f}")
                    print(f"  [{sn}] box ch3(h):  {bf[3].min():.3f}~{bf[3].max():.3f}")
                    print(f"  [{sn}] cls:  {cf.min():.4f}~{cf.max():.4f}")
                    print(f"  [{sn}] obj:  {of.min():.4f}~{of.max():.4f}")
                    max_score = (of[0] * cf.max(0)).max()
                    print(f"  [{sn}] max(obj*cls)={max_score:.6f}")
                print("  -> box 值域 0~15: DFL 距离 (SIGMOID_FUSED_BOX=True 正确)")
                print("  -> cls/obj 值域 0~1: 已融合 (SIGMOID_FUSED_CLS=True)")
                print(f"  -> 当前解码模式: {DECODE_MODE}\n")

            # ---- 后处理 (9 输出解码 + NMS) ----
            t0 = time.time()
            min_conf = min(CONF_THRESH, DOT_CONF_THRESH)
            boxes, scores, class_ids = post_process(outputs, min_conf, IOU_THRESH)

            if len(boxes) > 0:
                if USE_LETTERBOX:
                    boxes[:, [0, 2]] = (boxes[:, [0, 2]] - lb_dw) / lb_r
                    boxes[:, [1, 3]] = (boxes[:, [1, 3]] - lb_dh) / lb_r
                else:
                    boxes[:, [0, 2]] *= 640 / INPUT_SIZE
                    boxes[:, [1, 3]] *= 480 / INPUT_SIZE
                is_dot = class_ids == DOT_CLASS_ID
                main_keep = (~is_dot) & (scores > CONF_THRESH)
                dot_keep = is_dot & (scores > DOT_CONF_THRESH)
                keep = main_keep | dot_keep
                boxes = boxes[keep]
                scores = scores[keep]
                class_ids = class_ids[keep]
            t_post_acc += (time.time() - t0) * 1000

            # ---- 跟踪器 + 串口 ----
            tracked, center, box, score, cls_id, dot_tracked, dot_center, dot_box, dot_score = \
                tracker.update(boxes, scores, class_ids)

            # ★ 修改4: 丢失期间发最后有效坐标, 超过 MAX_LOST_FRAMES 才发 -1
            # 主目标: 检测到→发当前坐标; 丢失未超阈值→发缓存坐标; 超阈值→发-1
            if tracked:
                main_cx, main_cy = center
            elif not tracker.is_lost() and tracker.last_main_cx is not None:
                main_cx, main_cy = tracker.last_main_cx, tracker.last_main_cy
            else:
                main_cx, main_cy = None, None

            # dot 目标同理
            if dot_tracked:
                dot_cx, dot_cy = dot_center
            elif not tracker.is_dot_lost() and tracker.last_dot_cx is not None:
                dot_cx, dot_cy = tracker.last_dot_cx, tracker.last_dot_cy
            else:
                dot_cx, dot_cy = None, None

            send_coords(ser, main_cx, main_cy, dot_cx, dot_cy)

            # ---- 显示 (降频, 主线程负责, 用原图画框) ----
            t0 = time.time()
            if frame_count % DISPLAY_EVERY == 0:
                disp = frame.copy()
                # 主目标
                if tracked and box is not None:
                    x1, y1, x2, y2 = box
                    name = CLASS_NAMES[cls_id] if cls_id < NUM_CLASSES else f"cls_{cls_id}"
                    cv2.rectangle(disp, (x1, y1), (x2, y2), (0, 255, 0), 2)
                    cv2.putText(disp, f"{name}:{score:.2f}", (x1, y1 - 8),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)
                    cv2.circle(disp, center, 5, (0, 0, 255), -1)
                # dot 目标
                if dot_tracked and dot_box is not None:
                    x1, y1, x2, y2 = dot_box
                    cv2.rectangle(disp, (x1, y1), (x2, y2), (0, 255, 255), 2)
                    cv2.putText(disp, f"dot:{dot_score:.2f}", (x1, y1 - 8),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 1)
                    cv2.circle(disp, dot_center, 5, (255, 0, 255), -1)
                # 状态信息
                info = f"FPS:{fps:.1f} "
                info += f"M({center[0]},{center[1]}) " if tracked else "M(LOST) "
                info += f"D({dot_center[0]},{dot_center[1]})" if dot_tracked else "D(LOST)"
                cv2.putText(disp, info, (8, 22), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 255, 0), 1)
                disp = cv2.resize(disp, (SCREEN_WIDTH, SCREEN_HEIGHT))
                cv2.imshow("Detection", disp)
                if cv2.waitKey(1) & 0xFF == ord('q'):
                    stop_event.set()
                    break
            t_show_acc += (time.time() - t0) * 1000

            # ---- FPS 统计 ----
            frame_count += 1
            if frame_count % 30 == 0:
                fps = 30.0 / (time.time() - fps_start)
                fps_start = time.time()
                n = 30
                print(f"[FPS {fps:.1f}] cap={1000*t_cap_acc[0]/n:.0f} pre={1000*t_pre_acc[0]/n:.0f} "
                      f"npu={1000*t_npu_acc[0]/n:.0f} post={t_post_acc/n:.0f} show={t_show_acc/n:.0f} ms")
                t_cap_acc[0] = t_pre_acc[0] = t_npu_acc[0] = 0.0
                t_post_acc = t_show_acc = 0.0

    except KeyboardInterrupt:
        print("\n[INFO] Interrupted")
    finally:
        stop_event.set()
        # 通知推理线程退出
        try:
            q_pre2npu.put(None, timeout=1.0)
        except queue.Full:
            pass
        th_cap.join(timeout=2.0)
        th_npu.join(timeout=2.0)
        cap.release()
        cv2.destroyAllWindows()
        rknn.release()
        if ser:
            ser.close()
        print("\n[INFO] Done!")


if __name__ == "__main__":
    main()
