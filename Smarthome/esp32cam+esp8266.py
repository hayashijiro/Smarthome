#!/usr/bin/env python3
import os
import json
import time
import threading
import logging

import cv2
import numpy as np
import requests
from flask import Flask, Response, jsonify, request

ESP32CAM_IP = os.environ.get("ESP32CAM_IP", "http://10.87.168.152")
ESP32CAM_CAPTURE = f"{ESP32CAM_IP}/capture"
ESP32CAM_STREAM = f"{ESP32CAM_IP}/stream"
ESP32CAM_FLASH = f"{ESP32CAM_IP}/flash"

DEVICE_MAP_FILE = "device_map.json"
DEVICE_MAP_STATIC = {}

DATASET_PATH = os.environ.get("DATASET_PATH", "dataset/owner")
CONFIDENCE_THRESHOLD = float(os.environ.get("CONFIDENCE_THRESHOLD", 70.0))
DARKNESS_THRESHOLD = int(os.environ.get("DARKNESS_THRESHOLD", 50))

CAM_FETCH_TIMEOUT = 5
CAM_FETCH_RETRIES = 2
CACHE_TTL_SECONDS = 5

FLASK_HOST = "0.0.0.0"
FLASK_PORT = int(os.environ.get("FLASK_PORT", 5000))

logging.basicConfig(level=logging.INFO, format="%(asctime)s %(levelname)s %(message)s")
logger = logging.getLogger("smartserver")

app = Flask(__name__)

if os.path.exists(DEVICE_MAP_FILE):
    try:
        with open(DEVICE_MAP_FILE, "r") as f:
            DEVICE_MAP = json.load(f)
            logger.info("Loaded DEVICE_MAP from %s", DEVICE_MAP_FILE)
    except Exception as e:
        logger.warning("Failed to load %s: %s. Using empty map.", DEVICE_MAP_FILE, e)
        DEVICE_MAP = DEVICE_MAP_STATIC.copy()
else:
    DEVICE_MAP = DEVICE_MAP_STATIC.copy()

train_images = []
train_labels = []
recognizer = None
recognizer_lock = threading.Lock()


def preprocess_face(img):
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    gray = cv2.equalizeHist(gray)
    gray = cv2.GaussianBlur(gray, (3,3), 0)
    face_resized = cv2.resize(gray, (200,200))
    return face_resized


def load_dataset(path=DATASET_PATH):
    images, labels = [], []
    label_owner = 1
    face_cascade_path = cv2.data.haarcascades + "haarcascade_frontalface_default.xml"
    face_cascade = cv2.CascadeClassifier(face_cascade_path)
    if face_cascade.empty():
        logger.error("Could not load Haar cascade")
        return images, labels
    if not os.path.exists(path):
        logger.info("No dataset at %s", path)
        return images, labels
    for fname in os.listdir(path):
        if not fname.lower().endswith((".png",".jpg",".jpeg")):
            continue
        img = cv2.imread(os.path.join(path,fname))
        if img is None:
            continue
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        faces = face_cascade.detectMultiScale(gray, 1.3, 5, minSize=(30,30))
        for (x,y,w,h) in faces:
            roi = img[y:y+h, x:x+w]
            images.append(preprocess_face(roi))
            labels.append(label_owner)
    logger.info("Loaded %d training images", len(images))
    return images, labels


def init_recognizer():
    global recognizer, train_images, train_labels
    train_images, train_labels = load_dataset()
    if len(train_images) > 0:
        try:
            recognizer = cv2.face.LBPHFaceRecognizer_create()
            recognizer.train(train_images, np.array(train_labels))
            logger.info("Recognizer trained")
        except Exception as e:
            recognizer = None
            logger.exception("Failed to init recognizer: %s", e)
    else:
        recognizer = None
        logger.info("No training images; recognizer disabled")


init_recognizer()
face_cascade = cv2.CascadeClassifier(cv2.data.haarcascades + "haarcascade_frontalface_default.xml")
if face_cascade.empty():
    logger.error("Face cascade not loaded")

_result_cache = {}
_cache_lock = threading.Lock()


def cached_result(device_id):
    with _cache_lock:
        entry = _result_cache.get(device_id)
        if not entry:
            return None
        result, ts = entry
        if time.time() - ts <= CACHE_TTL_SECONDS:
            return result
        del _result_cache[device_id]
        return None


def set_cached_result(device_id, result):
    with _cache_lock:
        _result_cache[device_id] = (result, time.time())


def fetch_image_from_cam(url, timeout=CAM_FETCH_TIMEOUT, retries=CAM_FETCH_RETRIES):
    for attempt in range(1, retries+1):
        try:
            r = requests.get(url, timeout=timeout)
            r.raise_for_status()
            img_arr = np.frombuffer(r.content, np.uint8)
            frame = cv2.imdecode(img_arr, cv2.IMREAD_COLOR)
            if frame is None:
                logger.warning("Decoded frame is None (attempt %d)", attempt)
                continue
            return frame
        except Exception as e:
            logger.warning("fetch attempt %d failed: %s", attempt, e)
            time.sleep(0.1)
    logger.error("All fetch attempts failed")
    return None


def enhance_image(img):
    try:
        enhanced = cv2.convertScaleAbs(img, alpha=1.3, beta=30)
        kernel = np.array([[0,-1,0],[-1,5,-1],[0,-1,0]])
        sharpened = cv2.filter2D(enhanced, -1, kernel)
        denoised = cv2.fastNlMeansDenoisingColored(sharpened, None, 10,10,7,21)
        return denoised
    except Exception:
        return img


def analyze_frame_for_owner(frame):
    if frame is None:
        return "error"
    try:
        frame_proc = enhance_image(frame)
        gray = cv2.cvtColor(frame_proc, cv2.COLOR_BGR2GRAY)
        faces = face_cascade.detectMultiScale(gray, 1.3, 5, minSize=(30,30))
        if len(faces) == 0:
            return "intruder"
        if recognizer is None:
            return "intruder"
        with recognizer_lock:
            for (x,y,w,h) in faces:
                try:
                    roi = frame_proc[y:y+h, x:x+w]
                    processed = preprocess_face(roi)
                    label, confidence = recognizer.predict(processed)
                    if int(label) == 1 and confidence < CONFIDENCE_THRESHOLD:
                        return "owner"
                    return "intruder"
                except Exception as e:
                    logger.exception("Predict error: %s", e)
                    return "error"
    except Exception as e:
        logger.exception("analyze error: %s", e)
        return "error"


@app.route('/health')
def health():
    return jsonify(status='ok', trained=(recognizer is not None), train_images=len(train_images))


@app.route('/analyze')
def analyze():
    frame = fetch_image_from_cam(ESP32CAM_CAPTURE)
    if frame is None:
        return Response('error', mimetype='text/plain'), 503
    flash_on = False
    try:
        if np.mean(cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)) < DARKNESS_THRESHOLD:
            try:
                requests.get(f"{ESP32CAM_FLASH}?state=on", timeout=3)
                flash_on = True
                f2 = fetch_image_from_cam(ESP32CAM_CAPTURE)
                if f2 is not None:
                    frame = f2
            except Exception as e:
                logger.warning("Flash on failed: %s", e)
        result = analyze_frame_for_owner(frame)
        return Response(result, mimetype='text/plain')
    finally:
        if flash_on:
            try:
                requests.get(f"{ESP32CAM_FLASH}?state=off", timeout=3)
            except Exception:
                pass


@app.route('/check_owner')
def check_owner():
    device_id = request.args.get('device_id')
    if not device_id:
        return Response('error', mimetype='text/plain'), 400
    cached = cached_result(device_id)
    if cached is not None:
        return Response(cached, mimetype='text/plain')
    frame = fetch_image_from_cam(ESP32CAM_CAPTURE)
    if frame is None:
        return Response('error', mimetype='text/plain'), 503
    flash_on = False
    try:
        if np.mean(cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)) < DARKNESS_THRESHOLD:
            try:
                requests.get(f"{ESP32CAM_FLASH}?state=on", timeout=3)
                flash_on = True
                f2 = fetch_image_from_cam(ESP32CAM_CAPTURE)
                if f2 is not None:
                    frame = f2
            except Exception as e:
                logger.warning("Flash on failed: %s", e)
        result = analyze_frame_for_owner(frame)
        set_cached_result(device_id, result)
        return Response(result, mimetype='text/plain')
    finally:
        if flash_on:
            try:
                requests.get(f"{ESP32CAM_FLASH}?state=off", timeout=3)
            except Exception:
                pass


@app.route('/device/command', methods=['POST'])
def device_command():
    data = request.get_json(force=True, silent=True)
    if not data:
        return jsonify(status='error', msg='invalid json'), 400
    device_id = data.get('device_id')
    cmd = data.get('cmd')
    params = data.get('params', {})
    if not device_id or not cmd:
        return jsonify(status='error', msg='device_id and cmd required'), 400
    if device_id not in DEVICE_MAP:
        return jsonify(status='error', msg='unknown device_id'), 404
    esp_base = DEVICE_MAP[device_id].rstrip('/')
    try:
        url = f"{esp_base}/cmd"
        params_get = {'name': cmd}
        for k, v in params.items():
            params_get[k] = v
        r = requests.get(url, params=params_get, timeout=5)
        r.raise_for_status()
        return jsonify(status='ok', device=device_id, resp=r.text)
    except Exception as e:
        logger.exception("device_command failed: %s", e)
        return jsonify(status='error', msg=str(e)), 500


@app.route('/flash')
def flash_control():
    state = request.args.get('state', '')
    if state not in ['on','off']:
        return jsonify(status='error', msg='state must be on/off'), 400
    try:
        resp = requests.get(f"{ESP32CAM_FLASH}?state={state}", timeout=5)
        return jsonify(status='ok', msg=resp.text)
    except Exception as e:
        logger.exception("flash proxy failed: %s", e)
        return jsonify(status='error', msg=str(e)), 500


@app.route('/stream')
def stream_proxy():
    def generate():
        try:
            with requests.get(ESP32CAM_STREAM, stream=True, timeout=10) as r:
                for chunk in r.iter_content(chunk_size=1024):
                    if chunk:
                        yield chunk
        except Exception as e:
            logger.exception("stream error: %s", e)
    return Response(generate(), mimetype='multipart/x-mixed-replace; boundary=frame')


if __name__ == '__main__':
    logger.info('Starting simple server')
    app.run(host=FLASK_HOST, port=FLASK_PORT, threaded=True)
