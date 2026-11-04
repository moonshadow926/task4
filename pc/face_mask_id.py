"""视觉任务四 PC 端：串口指令 -> 前置摄像头拍照 -> 人脸身份识别 + 口罩判断

检测：OpenCV YuNet；身份：OpenCV SFace 提取 128 维特征向量，与现场录入的特征做余弦比对（不训练网络）。
口罩：在对齐后的 112x112 人脸上，比较“眉心皮肤”和“口鼻下巴区域”的颜色，口鼻处皮肤占比低即判为戴口罩。

串口协议（STM32 -> PC，每行以 \r\n 结尾）：
    #CAP      拍一张并识别
    #CAP3     连拍 3 张（间隔 1 秒）用于对照
    #ENROLL   现场录入人脸（在控制台输入姓名）；录入过程中再收到一次 = 取消录入
    其他文本   原样打印（如“我爱视觉”）
PC -> STM32 回传：R:<name>,<MASK|NOMASK>,<相似度>\r\n

窗口快捷键（不接开发板也能调试）：c 拍照  e 录入  x 清空人脸库  q 退出
"""
import argparse
import csv
import json
import os
import queue
import threading
import time

import cv2
import numpy as np

try:
    import serial
except ImportError:
    serial = None

HERE = os.path.dirname(os.path.abspath(__file__))
MODEL_DIR = os.path.join(HERE, "models")
DET_MODEL = os.path.join(MODEL_DIR, "face_detection_yunet_2023mar.onnx")
REC_MODEL = os.path.join(MODEL_DIR, "face_recognition_sface_2021dec.onnx")
DB_PATH = os.path.join(HERE, "faces_db.json")
CAP_DIR = os.path.join(HERE, "captures")
LOG_PATH = os.path.join(CAP_DIR, "results.csv")

COSINE_THRESH = 0.363     # SFace 官方推荐阈值，>= 判为同一人
MASK_SKIN_RATIO = 0.35    # 口鼻区域“像皮肤”的像素占比低于此值 -> 戴口罩（现场可调）
ENROLL_SHOTS = 5


class FaceSystem:
    def __init__(self):
        self.detector = cv2.FaceDetectorYN.create(DET_MODEL, "", (320, 320), 0.6, 0.3, 5000)
        self.recognizer = cv2.FaceRecognizerSF.create(REC_MODEL, "")
        self.db = self.load_db()

    # ---------- 人脸库 ----------
    def load_db(self):
        if not os.path.exists(DB_PATH):
            return {}
        with open(DB_PATH, encoding="utf-8") as f:
            return {k: [np.array(v, dtype=np.float32) for v in feats] for k, feats in json.load(f).items()}

    def save_db(self):
        with open(DB_PATH, "w", encoding="utf-8") as f:
            json.dump({k: [v.tolist() for v in feats] for k, feats in self.db.items()}, f, ensure_ascii=False)

    # ---------- 基础能力 ----------
    def detect(self, frame):
        h, w = frame.shape[:2]
        self.detector.setInputSize((w, h))
        _, faces = self.detector.detect(frame)
        return [] if faces is None else list(faces)

    def feature(self, frame, face):
        aligned = self.recognizer.alignCrop(frame, face)
        return aligned, self.recognizer.feature(aligned).flatten().copy()

    @staticmethod
    def mask_check(aligned):
        """aligned 为 112x112 对齐人脸：双眼约在 y=51，鼻尖 y=72，嘴角 y=92。"""
        lab = cv2.cvtColor(aligned, cv2.COLOR_BGR2LAB).astype(np.float32)
        ref = lab[24:42, 42:70].reshape(-1, 3)      # 眉心/额头下部，作为本人肤色参考
        low = lab[76:108, 30:82].reshape(-1, 3)     # 鼻下-嘴-下巴
        ref_med = np.median(ref, axis=0)
        d_ab = np.linalg.norm(low[:, 1:] - ref_med[1:], axis=1)
        d_l = np.abs(low[:, 0] - ref_med[0])
        skin_ratio = float(np.mean((d_ab < 10) & (d_l < 45)))
        return skin_ratio < MASK_SKIN_RATIO, skin_ratio

    def identify(self, feat):
        best_name, best_sim = "Unknown", 0.0
        for name, feats in self.db.items():
            for ref in feats:
                sim = self.recognizer.match(feat.reshape(1, -1), ref.reshape(1, -1), cv2.FaceRecognizerSF_FR_COSINE)
                if sim > best_sim:
                    best_name, best_sim = name, sim
        if best_sim < COSINE_THRESH:
            best_name = "Unknown"
        return best_name, best_sim

    # ---------- 业务 ----------
    def analyze(self, frame):
        results = []
        for face in self.detect(frame):
            aligned, feat = self.feature(frame, face)
            masked, ratio = self.mask_check(aligned)
            if masked:
                name, sim = "-", 0.0          # 学长要求：戴口罩时不要求身份识别
            else:
                name, sim = self.identify(feat)
            results.append(dict(box=face[:4].astype(int), name=name, sim=sim, masked=masked, ratio=ratio))
        return results

    def enroll_feature(self, frame):
        """从一帧中取录入用特征：返回 (feat, None)；失败返回 (None, 原因)"""
        faces = self.detect(frame)
        if not faces:
            return None, "noface"
        face = max(faces, key=lambda f: f[2] * f[3])       # 只取画面中最大的脸
        aligned, feat = self.feature(frame, face)
        if self.mask_check(aligned)[0]:
            return None, "mask"
        return feat, None

    def add_person(self, name, feats):
        self.db.setdefault(name, []).extend(feats)
        self.save_db()


class Enroller:
    """非阻塞录入状态机：IDLE -> NAME(等待输入姓名) -> CAPTURE(采集 N 帧) -> IDLE
    录入过程中主循环照常运行，再次收到 #ENROLL（再按一次 KEY2）即取消，防止误触。"""
    IDLE, NAME, CAPTURE = range(3)

    def __init__(self, fs):
        self.fs = fs
        self.state = self.IDLE
        self.lines = queue.Queue()     # 后台线程读控制台输入，避免 input() 卡死主循环
        threading.Thread(target=self._read_stdin, daemon=True).start()

    def _read_stdin(self):
        while True:
            try:
                self.lines.put(input())
            except EOFError:
                return

    @property
    def busy(self):
        return self.state != self.IDLE

    def toggle(self):
        """收到 #ENROLL：空闲则开始录入，录入中则取消"""
        if self.busy:
            self.cancel()
            return
        while not self.lines.empty():                        # 丢弃之前残留的输入
            self.lines.get_nowait()
        self.state = self.NAME
        print("[录入] 请在控制台输入录入者姓名并回车（建议拼音，如 ZhangSan）；再按一次 KEY2 / e 取消")

    def cancel(self):
        print("[录入] 已取消，人脸库未改动")
        self.state = self.IDLE

    def step(self, frame):
        """主循环每帧调用一次"""
        if self.state == self.NAME:
            try:
                line = self.lines.get_nowait()
            except queue.Empty:
                return
            self.name = line.strip() or "Senior"
            self.feats, self.next_t, self.last_warn = [], 0.0, ""
            self.state = self.CAPTURE
            print(f"[录入] {self.name}：请正对摄像头、不戴口罩，采集 {ENROLL_SHOTS} 帧（再按一次 KEY2 / e 取消）...")
        elif self.state == self.CAPTURE and frame is not None and time.time() >= self.next_t:
            feat, why = self.fs.enroll_feature(frame)
            if feat is None:
                if why == "mask" and self.last_warn != "mask":
                    print("[录入] 检测到口罩，请摘下口罩")
                self.last_warn = why
                return
            self.last_warn = ""
            self.feats.append(feat)
            self.next_t = time.time() + 0.2                  # 两帧之间稍微间隔，取到不同姿态
            print(f"[录入] 已采集 {len(self.feats)}/{ENROLL_SHOTS}")
            if len(self.feats) >= ENROLL_SHOTS:
                self.fs.add_person(self.name, self.feats)
                self.state = self.IDLE
                print(f"[录入] 完成，当前人脸库：{list(self.fs.db.keys())}")


def draw(frame, results):
    for r in results:
        x, y, w, h = r["box"]
        color = (0, 165, 255) if r["masked"] else ((0, 200, 0) if r["name"] != "Unknown" else (0, 0, 255))
        cv2.rectangle(frame, (x, y), (x + w, y + h), color, 2)
        text = "MASK" if r["masked"] else f"{r['name']} {r['sim']:.2f} NO-MASK"
        cv2.putText(frame, text, (x, max(20, y - 8)), cv2.FONT_HERSHEY_SIMPLEX, 0.6, color, 2)
    return frame


def capture_once(fs, cam, ser):
    ok, frame = cam.read()
    if not ok:
        print("[拍照] 摄像头读取失败")
        return
    results = fs.analyze(frame)
    os.makedirs(CAP_DIR, exist_ok=True)
    stamp = time.strftime("%Y%m%d_%H%M%S")
    path = os.path.join(CAP_DIR, f"{stamp}.jpg")
    cv2.imwrite(path, draw(frame.copy(), results))
    cv2.imshow("result", draw(frame, results))

    if not results:
        print("[结果] 画面中未检测到人脸")
        reply = "R:NOFACE"
    else:
        for i, r in enumerate(results, 1):
            if r["masked"]:
                print(f"[结果] 人脸{i}：佩戴口罩（口鼻肤色占比 {r['ratio']:.2f}），身份不识别")
            else:
                who = "陌生人" if r["name"] == "Unknown" else r["name"]
                print(f"[结果] 人脸{i}：身份={who}，相似度={r['sim']:.3f}，未佩戴口罩（口鼻肤色占比 {r['ratio']:.2f}）")
        r = results[0]
        reply = f"R:{r['name']},{'MASK' if r['masked'] else 'NOMASK'},{r['sim']:.2f}"

    new_log = not os.path.exists(LOG_PATH)
    with open(LOG_PATH, "a", newline="", encoding="utf-8-sig") as f:
        wr = csv.writer(f)
        if new_log:
            wr.writerow(["time", "photo", "faces", "name", "masked", "similarity", "skin_ratio"])
        for r in results or [dict(name="", masked="", sim="", ratio="")]:
            wr.writerow([stamp, path, len(results), r["name"], r["masked"], r["sim"], r["ratio"]])

    print(f"[保存] {path}")
    if ser:
        ser.write((reply + "\r\n").encode("ascii", "replace"))


def decode_line(raw):
    for enc in ("utf-8", "gbk"):        # Keil 源文件可能是 UTF-8 也可能是 GB2312
        try:
            return raw.decode(enc).strip()
        except UnicodeDecodeError:
            pass
    return raw.decode("latin-1").strip()


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", help="串口号，如 COM5；不填则只用键盘调试")
    ap.add_argument("--baud", type=int, default=115200)
    ap.add_argument("--cam", type=int, default=0)
    args = ap.parse_args()

    fs = FaceSystem()
    cam = cv2.VideoCapture(args.cam, cv2.CAP_DSHOW)
    cam.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
    cam.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
    ser = None
    if args.port:
        if serial is None:
            raise SystemExit("请先 pip install pyserial")
        # 正点原子板 USB_232 有一键下载电路：DTR/RTS 控制 RESET 和 BOOT0。
        # pyserial 打开串口时默认拉起 DTR/RTS，会把单片机卡在复位/Bootloader，按键全部无效，
        # 所以必须在 open() 之前把两根线关掉（串口助手里也要取消勾选 DTR/RTS）。
        ser = serial.Serial()
        ser.port, ser.baudrate, ser.timeout = args.port, args.baud, 0
        ser.dtr = False
        ser.rts = False
        ser.open()
        print(f"[串口] 已打开 {args.port} @ {args.baud}")
    print(f"[人脸库] {list(fs.db.keys()) or '空，请先录入'}")
    enroller = Enroller(fs)

    buf = b""
    while True:
        ok, frame = cam.read()
        if ok:
            cv2.imshow("preview (c=capture e=enroll x=clear q=quit)", frame)
        enroller.step(frame if ok else None)
        cmds = []
        if ser and ser.in_waiting:
            buf += ser.read(ser.in_waiting)
            while b"\n" in buf:
                line, buf = buf.split(b"\n", 1)
                text = decode_line(line)
                if text.startswith("#"):
                    cmds.append(text)
                elif text:
                    print(f"[单片机] {text}")
        key = cv2.waitKey(1) & 0xFF
        cmds += {ord("c"): ["#CAP"], ord("e"): ["#ENROLL"], ord("x"): ["#CLEAR"]}.get(key, [])
        if key == ord("q"):
            break

        for cmd in cmds:
            print(f"[指令] {cmd}")
            if enroller.busy and cmd != "#ENROLL":
                print("[录入] 正在录入中，忽略该指令（再按一次 KEY2 / e 可取消录入）")
                continue
            if cmd == "#CAP":
                capture_once(fs, cam, ser)
            elif cmd == "#CAP3":
                for _ in range(3):
                    capture_once(fs, cam, ser)
                    cv2.waitKey(1000)
            elif cmd == "#ENROLL":
                enroller.toggle()
            elif cmd == "#CLEAR":
                fs.db = {}
                fs.save_db()
                print("[人脸库] 已清空")

    cam.release()
    if ser:
        ser.close()
    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
