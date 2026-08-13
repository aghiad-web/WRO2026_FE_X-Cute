"""
كود رؤية حاسوبية لكشف المكعبات (أحمر/أخضر) - مسابقة WRO 2026 Future Engineers
------------------------------------------------------------------------
المنطق:
- مكعب أخضر  -> استدارة يسار (LEFT)
- مكعب أحمر  -> استدارة يمين (RIGHT)
- ما في مكعب -> استمرار للأمام (FORWARD)

يعتمد على تحويل الصورة لفضاء الألوان HSV لأنه أدق وأكثر ثباتاً
من RGB بالنسبة لتغيرات الإضاءة (مهم جداً بإضاءة الحلبة/الترابيزة).

يمكنك تعديل نطاقات الألوان (HSV ranges) حسب إضاءة مكان التدريب عندك.
"""

import cv2
import numpy as np
import serial
import time

# ==========================================================
# 0) إعدادات السيريال (UART) للاتصال مع STM32
# ==========================================================
# غيّر SERIAL_PORT حسب جهازك:
#   ويندوز:  "COM3", "COM4" ... (شوفه من Device Manager)
#   لينكس:   "/dev/ttyUSB0" أو "/dev/ttyACM0"
#   ماك:     "/dev/tty.usbserial-XXXX"
SERIAL_PORT = "COM4"
BAUD_RATE = 115200          # لازم يكون نفس الـ baud rate المضبوط بكود STM32 (USART)

# الأوامر اللي رح تنبعت لـ STM32 - حرف واحد لكل قرار (أخف وأسرع إرسال)
CMD_LEFT = b'L'
CMD_RIGHT = b'R'
CMD_FORWARD = b'F'

ser = None  # متغير الاتصال السيريال، رح يتفتح بالـ main()


def init_serial():
    """يحاول يفتح بورت السيريال، وإذا فشل يكمل الكود بدون STM32 (وضع تجربة فقط)"""
    global ser
    try:
        ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1)
        time.sleep(2)  # وقت مطلوب عادةً حتى يجهز البورت بعد الفتح
        print(f"تم فتح السيريال على {SERIAL_PORT} بسرعة {BAUD_RATE}")
    except serial.SerialException as e:
        print(f"تحذير: ما قدرت أفتح السيريال ({e}). رح يشتغل الكود بدون إرسال لـ STM32.")
        ser = None


def send_serial_command(cmd_bytes):
    """يبعت أمر واحد عبر السيريال لو البورت مفتوح"""
    if ser is not None and ser.is_open:
        try:
            ser.write(cmd_bytes)
        except serial.SerialException as e:
            print(f"خطأ بالإرسال عبر السيريال: {e}")


# ==========================================================
# 1) إعدادات نطاقات الألوان (HSV) - عدّلها حسب الإضاءة عندك
# ==========================================================
# اللون الأحمر بياخد نطاقين لأنه يلف حوالين 0/180 بمقياس Hue في OpenCV
LOWER_RED_1 = np.array([0, 100, 80])
UPPER_RED_1 = np.array([10, 255, 255])
LOWER_RED_2 = np.array([170, 100, 80])
UPPER_RED_2 = np.array([180, 255, 255])

LOWER_GREEN = np.array([40, 70, 60])
UPPER_GREEN = np.array([85, 255, 255])

# أقل مساحة (بالبكسل) عشان نعتبر الكشف "مكعب حقيقي" مش ضوضاء
MIN_CONTOUR_AREA = 800

# مسافة تقريبية: كل ما زادت مساحة الكونتور كل ما كان المكعب أقرب للروبوت
CLOSE_AREA_THRESHOLD = 6000  # عدّلها حسب تجربتك العملية

# دقة الكاميرا - قللها إذا بدك أداء أسرع (أقل تأخير/لاق)
# دقة أوطى = معالجة أسرع بس تفاصيل أقل، دقة أعلى = أدق بس أبطأ
FRAME_WIDTH = 640
FRAME_HEIGHT = 480


def get_color_mask(hsv_frame):
    
    mask_green = cv2.inRange(hsv_frame, LOWER_GREEN, UPPER_GREEN)

    mask_red_1 = cv2.inRange(hsv_frame, LOWER_RED_1, UPPER_RED_1)
    mask_red_2 = cv2.inRange(hsv_frame, LOWER_RED_2, UPPER_RED_2)
    mask_red = cv2.bitwise_or(mask_red_1, mask_red_2)

    # تنظيف الضوضاء (فتح ثم إغلاق)
    kernel = np.ones((5, 5), np.uint8)
    mask_green = cv2.morphologyEx(mask_green, cv2.MORPH_OPEN, kernel)
    mask_green = cv2.morphologyEx(mask_green, cv2.MORPH_CLOSE, kernel)

    mask_red = cv2.morphologyEx(mask_red, cv2.MORPH_OPEN, kernel)
    mask_red = cv2.morphologyEx(mask_red, cv2.MORPH_CLOSE, kernel)

    return mask_green, mask_red


def find_largest_cube(mask):
    """يلاقي أكبر كونتور بالقناع ويرجع (مركزه، مساحته، صندوقه) أو None"""
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    if not contours:
        return None

    largest = max(contours, key=cv2.contourArea)
    area = cv2.contourArea(largest)

    if area < MIN_CONTOUR_AREA:
        return None

    x, y, w, h = cv2.boundingRect(largest)
    center_x = x + w // 2
    center_y = y + h // 2

    return {
        "center": (center_x, center_y),
        "area": area,
        "box": (x, y, w, h),
    }


def decide_direction(frame):
    """
    يحلل فريم واحد ويرجع القرار:
    'LEFT'  -> مكعب أخضر
    'RIGHT' -> مكعب أحمر
    'FORWARD' -> ما في مكعب واضح
    مع معلومات إضافية للتصحيح (debug)
    """
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    mask_green, mask_red = get_color_mask(hsv)

    green_cube = find_largest_cube(mask_green)
    red_cube = find_largest_cube(mask_red)

    decision = "FORWARD"
    chosen = None
    color_name = None

    # إذا في المكعبين سوا، ناخد الأقرب (الأكبر مساحة) لأنه هو اللي لازم نتعامل معه أول
    if green_cube and red_cube:
        if green_cube["area"] >= red_cube["area"]:
            chosen, decision, color_name = green_cube, "LEFT", "GREEN"
        else:
            chosen, decision, color_name = red_cube, "RIGHT", "RED"
    elif green_cube:
        chosen, decision, color_name = green_cube, "LEFT", "GREEN"
    elif red_cube:
        chosen, decision, color_name = red_cube, "RIGHT", "RED"

    return decision, chosen, color_name, mask_green, mask_red


def draw_debug(frame, chosen, decision, color_name):
    """يرسم صندوق حول المكعب المكتشف والقرار فوق الصورة (للتجربة على الشاشة)"""
    if chosen:
        x, y, w, h = chosen["box"]
        color_bgr = (0, 255, 0) if color_name == "GREEN" else (0, 0, 255)
        cv2.rectangle(frame, (x, y), (x + w, y + h), color_bgr, 2)
        cv2.putText(frame, f"{color_name} -> {decision}", (x, y - 10),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, color_bgr, 2)
    else:
        cv2.putText(frame, "FORWARD", (20, 40),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.9, (255, 255, 255), 2)
    return frame


# ==========================================================
# 2) دالة اتخاذ القرار الفعلي بتربطها بموتورات الروبوت
#    عدّل محتوى الدالة حسب مكتبة التحكم عندك (GPIO / Serial / PWM ...)
# ==========================================================
def send_command_to_robot(decision):
    if decision == "LEFT":
        print(">> أمر: استدارة يسار (مكعب أخضر) -> إرسال 'L'")
        send_serial_command(CMD_LEFT)
    elif decision == "RIGHT":
        print(">> أمر: استدارة يمين (مكعب أحمر) -> إرسال 'R'")
        send_serial_command(CMD_RIGHT)
    else:
        print(">> أمر: استمرار للأمام -> إرسال 'F'")
        send_serial_command(CMD_FORWARD)


# ==========================================================
# 3) الحلقة الرئيسية (Main loop)
# ==========================================================
def main():
    init_serial()  # فتح الاتصال مع STM32 (لو ما نجح، الكود بيكمل بدون إرسال)

    # index الكاميرا ممكن يختلف (0, 1, 2...) حسب جهازك
    cap = cv2.VideoCapture(0)

    if not cap.isOpened():
        print("خطأ: ما قدرت أفتح الكاميرا. جرب تغيير رقم index أو تأكد من التوصيل.")
        return

    # تطبيق الدقة المطلوبة (إذا الكاميرا ما بتدعمها، بترجع لأقرب دقة مدعومة تلقائياً)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, FRAME_WIDTH)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, FRAME_HEIGHT)

    actual_w = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
    actual_h = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
    print(f"دقة الكاميرا الفعلية: {actual_w}x{actual_h}")

    print("بدء الكشف... اضغط 'q' للخروج")

    last_sent = None  # لتجنب إرسال نفس الأمر مئات المرات بالثانية

    while True:
        ret, frame = cap.read()
        if not ret:
            print("خطأ بقراءة الفريم من الكاميرا")
            break

        decision, chosen, color_name, mask_green, mask_red = decide_direction(frame)

        # نرسل الأمر للروبوت فقط لما نكون قريبين كفاية من المكعب (اختياري)
        should_act = chosen is None or chosen["area"] >= CLOSE_AREA_THRESHOLD

        # نبعت بس لما القرار يتغير عن آخر مرة - بيخفف الضغط على STM32
        if should_act and decision != last_sent:
            send_command_to_robot(decision)
            last_sent = decision

        # عرض للتصحيح - احذف هاد الجزء لما ترفع الكود عالروبوت الفعلي بدون شاشة
        debug_frame = draw_debug(frame.copy(), chosen, decision, color_name)
        cv2.imshow("Camera Feed", debug_frame)
        cv2.imshow("Green Mask", mask_green)
        cv2.imshow("Red Mask", mask_red)

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    cap.release()
    cv2.destroyAllWindows()
    if ser is not None and ser.is_open:
        ser.close()
        print("تم إغلاق السيريال.")


if __name__ == "__main__":
    main()
