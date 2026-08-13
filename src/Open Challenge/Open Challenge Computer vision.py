import time
import cv2
import numpy as np
import serial

# ==========================================
# 1. الاتصال بـ STM32
# ==========================================
ser = None

def connect_to_stm32():
    """محاولة الاتصال المستمرة بـ STM32 وتجاهل الأخطاء حتى ينجح الاتصال"""
    global ser
    while True:
        try:
            # المحاولة الأولى عبر الاسم الثابت (udev)
            ser = serial.Serial('/dev/stm32_f411', 115200, timeout=1)
            print("✅ تم الاتصال بـ STM32 بنجاح (/dev/stm32_f411)")
            return
        except Exception:
            try:
                # المحاولة الثانية عبر المنفذ الافتراضي
                ser = serial.Serial('/dev/ttyACM0', 115200, timeout=1)
                print("✅ تم الاتصال بـ STM32 بنجاح (/dev/ttyACM0)")
                return
            except Exception:
                print("⚠️ جاري محاولة إعادة الاتصال بـ STM32...")
                time.sleep(1)

# بدء الاتصال الأولي
connect_to_stm32()

# ==========================================
# 2. إعدادات التوجيه وحالات الروبوت
# ==========================================
SERVO_CENTER = 2048
SERVO_OFFSET = 350
SERVO_MIN_LIMIT = SERVO_CENTER - SERVO_OFFSET
SERVO_MAX_LIMIT = SERVO_CENTER + SERVO_OFFSET
DRIVE_SPEED = 140

# حالة الروبوت وتجاه الحلبة
ROBOT_STATE = "STRAIGHT"
TRACK_DIRECTION = "UNKNOWN"

def send_control_to_stm32(speed, servo_pos, track_dir):
    """
    إرسال أمر السيرفو والسرعة والاتجاه إلى STM32
    """
    global ser
    data_packet = f"S:{speed},P:{servo_pos},D:{ROBOT_STATE}\n"
    
    while True:
        if ser and ser.is_open:
            try:
                ser.write(data_packet.encode('utf-8'))
                break
            except (OSError, serial.SerialException) as e:
                print(f"خطأ في إرسال البيانات (انقطع الاتصال): {e}")
                
                try:
                    ser.close()
                except:
                    pass
                
                connect_to_stm32() 
        else:
            connect_to_stm32()

# ==========================================
# 3. إعدادات الألوان (HSV)
# ==========================================
LOWER_BLUE = np.array([90, 70, 30])
UPPER_BLUE = np.array([135, 255, 255])

LOWER_ORANGE = np.array([4, 150, 130])
UPPER_ORANGE = np.array([20, 255, 255])

COLOR_THRESHOLD = 250  

# ==========================================
# 4. معالجة الصور والتوجيه
# ==========================================
def process_frame(frame):
    global ROBOT_STATE, TRACK_DIRECTION

    display_frame = frame.copy()
    h, w = frame.shape[:2]

    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    roi_bottom = hsv[int(h * 0.75):h, :]

    mask_blue = cv2.inRange(roi_bottom, LOWER_BLUE, UPPER_BLUE)
    mask_orange = cv2.inRange(roi_bottom, LOWER_ORANGE, UPPER_ORANGE)

    blue_pixels = cv2.countNonZero(mask_blue)
    orange_pixels = cv2.countNonZero(mask_orange)

    # State Machine
    if ROBOT_STATE == "STRAIGHT":
        if TRACK_DIRECTION == "UNKNOWN":
            if blue_pixels > COLOR_THRESHOLD:
                TRACK_DIRECTION = "CCW"
                ROBOT_STATE = "TURN_LEFT"
                print("🔵 مسار: عكس عقارب الساعة (CCW) -> التفاف يسار")
            elif orange_pixels > COLOR_THRESHOLD:
                TRACK_DIRECTION = "CW"
                ROBOT_STATE = "TURN_RIGHT"
                print("🟠 مسار: مع عقارب الساعة (CW) -> التفاف يمين")

        elif TRACK_DIRECTION == "CCW":
            if blue_pixels > COLOR_THRESHOLD:
                ROBOT_STATE = "TURN_LEFT"

        elif TRACK_DIRECTION == "CW":
            if orange_pixels > COLOR_THRESHOLD:
                ROBOT_STATE = "TURN_RIGHT"

    elif ROBOT_STATE == "TURN_LEFT":
        if blue_pixels < COLOR_THRESHOLD:
            ROBOT_STATE = "STRAIGHT"
            print("✅ الانتهاء من المنعطف اليساري -> قيادة مستقيمة")

    elif ROBOT_STATE == "TURN_RIGHT":
        if orange_pixels < COLOR_THRESHOLD:
            ROBOT_STATE = "STRAIGHT"
            print("✅ الانتهاء من المنعطف اليميني -> قيادة مستقيمة")

    # تحديد موقع السيرفو
    if ROBOT_STATE == "TURN_LEFT":
        servo_position = SERVO_MIN_LIMIT
    elif ROBOT_STATE == "TURN_RIGHT":
        servo_position = SERVO_MAX_LIMIT
    else:
        servo_position = SERVO_CENTER

    send_control_to_stm32(DRIVE_SPEED, servo_position, TRACK_DIRECTION)

    # العرض المرئي
    cv2.putText(display_frame, f"STATE: {ROBOT_STATE}", (10, 30),
                cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
    cv2.putText(display_frame, f"DIR: {TRACK_DIRECTION}", (10, 60),
                cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 0), 2)

    cv2.imshow("Main Camera", display_frame)
    cv2.imshow("Blue Mask", mask_blue)
    cv2.imshow("Orange Mask", mask_orange)

# ==========================================
# 5. الحلقة الرئيسية
# ==========================================
cap = cv2.VideoCapture(0)
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 320)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 240)

print("--- النظام جاهز ---")

try:
    while True:
        ret, frame = cap.read()
        if not ret or frame is None:
            continue

        process_frame(frame)

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
except KeyboardInterrupt:
    print("\nتم إيقاف التشغيل بواسطة المستخدم.")
finally:
    if 'cap' in locals() and cap.isOpened():
        cap.release()
    if ser and ser.is_open:
        ser.close()
    cv2.destroyAllWindows()

