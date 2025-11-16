# import cac thu vien can thiet de xu li anh
import cv2  # thu vien co cac tac vu xu li anh va nhan dien khuon mat
import numpy as np  # dung de xu li mang (vi anh la cac pixel)
import requests  # dung de giao tiep qua HTTP (goi API cua espcam32)
import os  # dung de doc ten file trong thu muc dataset
from flask import Flask, Response, jsonify, request  # <-- Response đã được import

ESP32CAM_IP = "http://10.87.168.152"  # IP ESP32-CAM
ESP32CAM_CAPTURE = f"{ESP32CAM_IP}/capture"
ESP32CAM_STREAM = f"{ESP32CAM_IP}/stream"
ESP32CAM_FLASH = f"{ESP32CAM_IP}/flash"

DATASET_PATH = "dataset/owner"

app = Flask(__name__)  # tạo đối tượng web server


# ====== Hàm tăng độ nét ảnh ======
def enhance_image(img):  # hàm hỗ tro tăng chất lượng ảnh
    # tăng cường độ sáng beta và độ tương phản alpha
    enhanced = cv2.convertScaleAbs(img, alpha=1.3, beta=30)
    # định nghĩa ma trận kernel cho bộ lọc làm sắc nét
    sharpen_kernel = np.array([[0, -1, 0], [-1, 5, -1], [0, -1, 0]])
    # áp dụng bộ lọc kernel để làm cho các cạnh và chi tiết rõ hơn
    sharpened = cv2.filter2D(enhanced, -1, sharpen_kernel)
    # sử dụng thuật toán khử nhiễu cho ảnh màu, giảm các hạt mo
    denoised = cv2.fastNlMeansDenoisingColored(sharpened, None, 10, 10, 7, 21)
    return denoised  # trả về ảnh đã được tăng cường chất lượng


# ====== Tiền xử lý khuôn mặt ======
def preprocess_face(img):
    # chuyển ảnh sang màu xám, mô hình LBPH hoạt động tốt nhất trên ảnh xám
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

    # cân bằng histogram: tăng cường độ tương phản cục bộ, vì ảnh đưa lên rất mờ
    # làm nổi bật các đặc điểm khuôn mặt bất kể điểu kiện ánh sáng
    gray = cv2.equalizeHist(gray)

    # làm mờ nhẹ, giảm nhiễu hạt giảm độ nhiễu pixel xuống trước khi đưa vào nhận diện
    gray = cv2.GaussianBlur(gray, (3, 3), 0)

    # thay đổi kích thước khuôn mặt về 200 x 200 pixel (chuẩn của mô hình)
    face_resized = cv2.resize(gray, (200, 200))

    # trả về khuôn mặt
    return face_resized


# ====== Load dataset chủ nhà ======
# định nghĩa hàm dữ liệu huấn luyện (để nhận dạng chủ nhà)
def load_dataset(path=DATASET_PATH):
    images, labels = [], []  # 1 mảng trống chưa dùng

    # gán nhãn số 1 cho tất cả ảnh của chủ nhà
    label_owner = 1

    # chỉ định thuật toán haar cascade để dùng xác định tọa độ trên khuôn mặt
    # ĐÃ SỬA LỖI CHÍNH TẢ Ở ĐÂY: haarcascade_frontalface_default.xml
    face_cascade = cv2.CascadeClassifier(cv2.data.haarcascades + "haarcascade_frontalface_default.xml")

    # Kiểm tra xem tệp Haar Cascade có được tải thành công không
    if face_cascade.empty():
        print("ERROR: Không thể tải haarcascade_frontalface_default.xml. Hãy kiểm tra đường dẫn OpenCV data.")
        return images, np.array(labels)  # Trả về mảng rỗng để tránh lỗi tiếp theo

    # lặp qua từng khung ảnh trong dataset
    print("Loading dataset...")
    # Kiểm tra thư mục dataset có tồn tại không
    if not os.path.exists(path):
        print(f"ERROR: Thư mục dataset không tồn tại: {path}")
        return images, np.array(labels)

    for fname in os.listdir(path):
        # Đảm bảo chỉ xử lý các file ảnh
        if not fname.lower().endswith(('.png', '.jpg', '.jpeg')):
            continue

        # đọc file và đưa vào biến img
        img = cv2.imread(os.path.join(path, fname))

        # nếu trong img không đọc được file bỏ qua file hình này và chuyển sang file tiếp theo
        if img is None:
            print(f"Warning: Không đọc được file: {fname}")
            continue
        # chuyển img sang xám
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        # áp dụng mô hình haar đã được nạp vào trong face_cascade để tìm tọa độ ảnh đã được chuyển qua xám
        # trả về faces là danh sách các tọa độ
        faces = face_cascade.detectMultiScale(gray, 1.3, 5, minSize=(30, 30))
        # lặp trong luoon cái khuôn mặt
        for (x, y, w, h) in faces:
            # sử dụng tọa độ để cắt vùng khuôn mặt từ ảnh màu gốc
            roi = img[y:y + h, x:x + w]
            # gọi hàm processed để cân bằng sáng, làm mờ và resize khuôn mặt về 200x200
            processed = preprocess_face(roi)
            # thêm ảnh đã xử lí vào danh sách images
            images.append(processed)
            # thêm nhãn số 1 vào danh sách labels
            labels.append(label_owner)
            # sau khi lặp xong tất cả file, hàm trả về danh sách ảnh (images)
            # và danh sách nhãn đã chuyển thành mảng NumPy (images[], labels[])
    print(f"Loaded {len(images)} images.")
    return images, np.array(labels)


# ====== Train LBPH ======
train_images, train_labels = load_dataset()

# Kiểm tra nếu dataset có ảnh thì mới train
if len(train_images) > 0:
    # khởi tạo thật toán LBPH cho recognizer dựa trên thuật toán LBPH
    recognizer = cv2.face.LBPHFaceRecognizer_create()
    # sử dụng ảnh và nhãn đã tải để dạy cho mô hình LBPH, mô hình sẽ học cách phân biệt chủ nhà
    print("Training LBPH recognizer...")
    recognizer.train(train_images, train_labels)
    print("Training complete.")
else:
    print("ERROR: No images found or Haar Cascade failed to load. Please check your 'dataset/owner' folder.")
    # Không gọi exit() để ứng dụng Flask vẫn chạy, nhưng việc nhận diện sẽ không hoạt động đúng

# SỬA LỖI CHÍNH TẢ Ở ĐÂY
face_cascade = cv2.CascadeClassifier(cv2.data.haarcascades + "haarcascade_frontalface_default.xml")
if face_cascade.empty():
    print("FATAL ERROR: Face Cascade could not be loaded for recognition.")


# ====== Hàm kiểm tra độ sáng ảnh ======
def is_dark(frame, threshold=50):
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    # tính độ sáng trung bình của tất cả pixel trong ảnh
    mean_brightness = np.mean(gray)
    return mean_brightness < threshold


# ====== API cho ESP8266 gọi để nhận diện ======
@app.route("/analyze", methods=["GET"])
def analyze():
    # Kiểm tra xem mô hình đã được train chưa
    if len(train_images) == 0 or face_cascade.empty():
        return Response("error", mimetype='text/plain')

    try:
        # Lấy ảnh từ ESP32-CAM
        resp = requests.get(ESP32CAM_CAPTURE, timeout=5)

        # tạo ra 1 mảng NumPy từ buffer , lấy dữ liệu byte thô và chuyển thành 1 mảng
        img_arr = np.frombuffer(resp.content, np.uint8)
        # giải nén mảng thành đối tượng ảnh hoàn chỉnh, frame là mảng 3 chiều đại diện cho bức ảnh
        frame = cv2.imdecode(img_arr, cv2.IMREAD_COLOR)

        if frame is None:
            print("ERROR: Không thể giải mã ảnh từ ESP32-CAM.")
            return Response("error", mimetype='text/plain')

        is_currently_dark = is_dark(frame)

        # Nếu ảnh quá tối thì bật flash
        if is_currently_dark:
            print("⚡ Ảnh tối, bật flash ESP32-CAM")
            try:
                requests.get(f"{ESP32CAM_FLASH}?state=on", timeout=3)
                # chụp lại sau khi bật flash
                resp = requests.get(ESP32CAM_CAPTURE, timeout=5)

                img_arr = np.frombuffer(resp.content, np.uint8)
                frame = cv2.imdecode(img_arr, cv2.IMREAD_COLOR)
            except Exception as e:
                print("Không bật được flash:", e)

        # tăng chất lượng ảnh
        frame = enhance_image(frame)
        # chuyển ảnh sang xám
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        # dùng haar trả về tọa độ
        faces = face_cascade.detectMultiScale(gray, 1.3, 5, minSize=(30, 30))

        result = "intruder"  # Mặc định là người lạ

        if len(faces) == 0:
            print("Không tìm thấy khuôn mặt.")
            result = "intruder"  # Mặc định là người lạ

        for (x, y, w, h) in faces:
            # thực hiện mô hình LBPH
            roi = frame[y:y + h, x:x + w]
            processed = preprocess_face(roi)

            # bắt đầu chạy LBPH, đưa processed mới vào và tạo biểu đồ và lưu vào trong đó
            # thực hiện so sánh với biểu đồ đã được lưu thông qua training
            label, confidence = recognizer.predict(processed)

            # ----- SỬA LOGIC NHẬN DIỆN -----
            # Label == 1 (chủ nhà) VÀ Độ tin cậy (confidence) CÀNG THẤP CÀNG TỐT
            if label == 1 and confidence < 70:
                result = "owner"
                print(f"✅ Chủ nhà (confidence = {confidence})")
            else:
                result = "intruder"
                print(f"🚨 Người lạ (label={label}, confidence = {confidence})")
            # -------------------------------

        # Sau khi xử lý, tắt flash nếu đã bật
        if is_currently_dark:
            try:
                requests.get(f"{ESP32CAM_FLASH}?state=off", timeout=3)
                print("Tắt flash.")
            except:
                pass

        # === THAY ĐỔI QUAN TRỌNG: Trả về văn bản thuần túy ===
        return Response(result, mimetype='text/plain')

    except Exception as e:
        print(f"Lỗi nghiêm trọng trong /analyze: {e}")
        # === THAY ĐỔI QUAN TRỌNG: Trả về văn bản thuần túy ===
        return Response("error", mimetype='text/plain')


# ====== API điều khiển flash LED thủ công ======
@app.route("/flash", methods=["GET"])
def flash_control():
    state = request.args.get("state", "")
    if state not in ["on", "off"]:
        return jsonify(status="error", msg="state must be on/off")
    try:
        resp = requests.get(f"{ESP32CAM_FLASH}?state={state}", timeout=5)
        return jsonify(status="ok", msg=resp.text)
    except Exception as e:
        return jsonify(status="error", msg=str(e))


# ====== Proxy stream từ ESP32-CAM ======
@app.route("/stream")
def stream_proxy():
    def generate():
        try:
            with requests.get(ESP32CAM_STREAM, stream=True, timeout=10) as r:
                for chunk in r.iter_content(chunk_size=1024):
                    if chunk:
                        yield chunk
        except Exception as e:
            print(f"Lỗi stream proxy: {e}")

    return Response(generate(), mimetype="multipart/x-mixed-replace; boundary=frame")


# ====== Chạy server Flask ======
if __name__ == "__main__":
    app.run(host="0.0.0.0", port=5000)