import cv2
import numpy as np

def process_coins():
    
    img = cv2.imread('/home/gary/HW_robotics_ws/src/my_turtle_controller/resource/coins2.jpg')
    if img is None:
        print("Can't find image!!!")
        return
    
    scale_percent = 20
    width = int(img.shape[1] * scale_percent / 100)
    height = int(img.shape[0] * scale_percent / 100)
    dim = (width, height)
    img = cv2.resize(img, dim, interpolation=cv2.INTER_AREA)

    # 轉成灰階
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

    # 高斯模糊
    blurred = cv2.GaussianBlur(gray, (7, 7), 0)

    # 自動閾值處理
    thresh = cv2.adaptiveThreshold(blurred, 255, cv2.ADAPTIVE_THRESH_GAUSSIAN_C, cv2.THRESH_BINARY_INV, 11, 2)
    
    # 邊緣檢測
    low_threshold = 10
    high_threshold = 30
    canny = cv2.Canny(blurred, low_threshold, high_threshold)

    # 形態學運算 (Morphology: Closing)
    kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (7, 7))
    closing = cv2.morphologyEx(thresh, cv2.MORPH_CLOSE, kernel)
    opening = cv2.morphologyEx(closing, cv2.MORPH_OPEN, kernel)
    
    # # 圓形檢測
    # circles = cv2.HoughCircles(blurred, cv2.HOUGH_GRADIENT, 1.2, minDist=70,
    #                            param1=50, param2=30, minRadius=10, maxRadius=65)
    # if circles is not None:
    #     circles = np.uint16(np.around(circles))
    #     for i in circles[0, :]:
    #         # 畫出圓周
    #         cv2.circle(img, (i[0], i[1]), i[2], (0, 255, 0), 2)
    #         # 畫出圓心
    #         cv2.circle(img, (i[0], i[1]), 2, (0, 0, 255), 3)
    # cv2.imshow('Detected Coins', img)

    # 尋找輪廓
    contours, _ = cv2.findContours(closing, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    count_contour = 0
    # 畫出輪廓在原圖上
    for cnt in contours:
        area = cv2.contourArea(cnt)
        # 過濾雜訊：只處理面積大於一定數值的輪廓
        if area > 1000:
            # 畫出綠色輪廓
            cv2.drawContours(img, [cnt], -1, (0, 255, 0), 2)
            
            # 計算質心
            M = cv2.moments(cnt)
            if M["m00"] != 0:
                cx = int(M["m10"] / M["m00"])
                cy = int(M["m01"] / M["m00"])
                cv2.circle(img, (cx, cy), 5, (0, 0, 255), -1)
                cv2.putText(img, f"Area:{int(area)}", (cx-20, cy-20), 
                           cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 0), 1)
            # 數輪廓
            count_contour = count_contour + 1
    print(f"偵測到硬幣數量: {count_contour}")

    # 顯示結果
    cv2.imshow("contours", img)
    cv2.imshow("gray", gray)
    cv2.imshow("blurred", blurred)
    cv2.imshow("thresh", thresh)
    cv2.imshow("canny", canny)
    cv2.imshow("opening", opening)
    cv2.imshow("closing", closing)
    
    cv2.waitKey(0)
    cv2.destroyAllWindows()

if __name__ == "__main__":
    process_coins()