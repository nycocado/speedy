"""Inferência YOLOv8 sobre NCNN (sem torch — só numpy + cv2).

Encapsula o backend para que trocar de modelo seja só trocar os ficheiros .param/.bin
e os parâmetros. Trocar de NCNN para outro runtime = substituir esta classe mantendo
o método infer().
"""
import cv2
import ncnn
import numpy as np


class Detection:
    __slots__ = ('x1', 'y1', 'x2', 'y2', 'score', 'cls')

    def __init__(self, x1, y1, x2, y2, score, cls):
        self.x1, self.y1, self.x2, self.y2 = x1, y1, x2, y2
        self.score, self.cls = score, cls


class YoloNcnn:
    def __init__(self, param_path, bin_path, input_size=320, num_threads=2,
                 input_name='in0', output_name='out0'):
        self.input_size = int(input_size)
        self.input_name = input_name
        self.output_name = output_name
        self.net = ncnn.Net()
        self.net.opt.use_vulkan_compute = False   # Pi4 não tem GPU Vulkan útil
        self.net.opt.num_threads = int(num_threads)
        if self.net.load_param(param_path) != 0:
            raise RuntimeError(f'Falha ao carregar param: {param_path}')
        if self.net.load_model(bin_path) != 0:
            raise RuntimeError(f'Falha ao carregar model: {bin_path}')

    def _letterbox(self, bgr):
        h, w = bgr.shape[:2]
        s = self.input_size
        r = min(s / h, s / w)
        nh, nw = int(round(h * r)), int(round(w * r))
        resized = cv2.resize(bgr, (nw, nh), interpolation=cv2.INTER_LINEAR)
        canvas = np.full((s, s, 3), 114, dtype=np.uint8)
        dh, dw = (s - nh) // 2, (s - nw) // 2
        canvas[dh:dh + nh, dw:dw + nw] = resized
        return canvas, r, dw, dh

    def infer(self, bgr, conf_thr=0.35, nms_thr=0.45):
        canvas, r, dw, dh = self._letterbox(bgr)
        rgb = cv2.cvtColor(canvas, cv2.COLOR_BGR2RGB).astype(np.float32) / 255.0
        chw = np.ascontiguousarray(rgb.transpose(2, 0, 1))

        ex = self.net.create_extractor()
        ex.input(self.input_name, ncnn.Mat(chw).clone())
        ret, out = ex.extract(self.output_name)
        if ret != 0:
            return []

        pred = np.squeeze(np.array(out))
        if pred.ndim != 2:
            return []
        if pred.shape[0] < pred.shape[1]:     # quero (anchors, 4+nc); 4+nc é a menor dim
            pred = pred.T

        scores_all = pred[:, 4:]
        cls = scores_all.argmax(axis=1)
        conf = scores_all[np.arange(scores_all.shape[0]), cls]
        keep = conf >= conf_thr
        if not keep.any():
            return []

        xywh = pred[keep, :4]
        conf = conf[keep]
        cls = cls[keep]
        # NMSBoxes usa caixas no espaço da rede (320); o unmap p/ a imagem vem depois.
        boxes = np.empty_like(xywh)
        boxes[:, 0] = xywh[:, 0] - xywh[:, 2] * 0.5
        boxes[:, 1] = xywh[:, 1] - xywh[:, 3] * 0.5
        boxes[:, 2] = xywh[:, 2]
        boxes[:, 3] = xywh[:, 3]
        idxs = cv2.dnn.NMSBoxes(boxes.tolist(), conf.tolist(), conf_thr, nms_thr)
        if len(idxs) == 0:
            return []
        idxs = np.array(idxs).flatten()

        dets = []
        for i in idxs:
            cx, cy, bw, bh = xywh[i]
            x1 = (cx - bw * 0.5 - dw) / r
            y1 = (cy - bh * 0.5 - dh) / r
            x2 = (cx + bw * 0.5 - dw) / r
            y2 = (cy + bh * 0.5 - dh) / r
            dets.append(Detection(float(x1), float(y1), float(x2), float(y2),
                                  float(conf[i]), int(cls[i])))
        return dets
