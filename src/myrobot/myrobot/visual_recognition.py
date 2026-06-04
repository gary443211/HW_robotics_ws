import cv2
import numpy as np
import cv2.aruco as aruco

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class ArucoPositionNode(Node):
    def __init__(self):
        super().__init__('aruco_position_node')
        self.publisher_ = self.create_publisher(String, '/aruco_positions', 10)

def process_image(args=None):
    rclpy.init(args=args)
    node = ArucoPositionNode()

    cap = cv2.VideoCapture(2)
    aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_4X4_50)
    parameters = aruco.DetectorParameters()
    detector = aruco.ArucoDetector(aruco_dict, parameters)
    # 定義 ID 與大小的對應關係
    id_to_size = {1: "Large", 2: "Medium", 3: "Small"}

    while cap.isOpened() and rclpy.ok():
        ret, img = cap.read()
    
        # 轉成灰階
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

        # 高斯模糊
        blurred = cv2.GaussianBlur(gray, (1, 1), 0)

        # 自動閾值處理
        ret, thresh = cv2.threshold(blurred, 110, 255, cv2.THRESH_BINARY)
        # thresh = cv2.adaptiveThreshold(blurred, 255, cv2.ADAPTIVE_THRESH_GAUSSIAN_C, cv2.THRESH_BINARY_INV, 11, 2)
        
        # # 邊緣檢測
        # low_threshold = 10
        # high_threshold = 30
        # canny = cv2.Canny(blurred, low_threshold, high_threshold)

        # # 形態學運算 (Morphology: Closing)
        # kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (7, 7))
        # closing = cv2.morphologyEx(thresh, cv2.MORPH_CLOSE, kernel)
        # opening = cv2.morphologyEx(closing, cv2.MORPH_OPEN, kernel)
        
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

        # # 尋找輪廓
        # contours, _ = cv2.findContours(closing, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        # filtered_contours = [cnt for cnt in contours if cv2.contourArea(cnt) > 2000]
        # sorted_contours = sorted(filtered_contours, key=cv2.contourArea, reverse=False)
        # count_contour = 0
        # # 畫出輪廓在原圖上
        # for cnt in sorted_contours:
        #     area = cv2.contourArea(cnt)
        #     # 過濾雜訊：只處理面積大於一定數值的輪廓
        #     if area > 2000:
        #         # 畫出綠色輪廓
        #         cv2.drawContours(img, [cnt], -1, (0, 255, 0), 2)
                
        #         # 計算質心
        #         M = cv2.moments(cnt)
        #         if M["m00"] != 0:
        #             cx = int(M["m10"] / M["m00"])
        #             cy = int(M["m01"] / M["m00"])
        #             cv2.circle(img, (cx, cy), 5, (0, 0, 255), -1)
        #             cv2.rectangle(img, (cx-100, cy-100), (cx+100, cy+100), (0, 0, 255), 2)
        #             cv2.putText(img, f"Area:{int(area)}", (cx-20, cy-20), 
        #                     cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 0), 1)
        #             if count_contour == 0:
        #                 cv2.putText(img, "Small", (cx-20, cy-110), 
        #                     cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 0), 1)
        #             elif count_contour == 1:
        #                 cv2.putText(img, "Midiumn", (cx-20, cy-110), 
        #                     cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 0), 1)
        #             elif count_contour == 2:
        #                 cv2.putText(img, "Large", (cx-20, cy-110), 
        #                     cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 0), 1)
        #         # 數輪廓
        #         count_contour = count_contour + 1
        # print(f"偵測到硬幣數量: {count_contour}")

        # 偵測 Marker
        corners, ids, rejected = detector.detectMarkers(thresh)

        if ids is not None:
            # 畫出偵測到的框
            aruco.drawDetectedMarkers(img, corners, ids)
            
            detected_markers = []

            for i in range(len(ids)):
                marker_id = ids[i][0]
                size_label = id_to_size.get(marker_id, "Unknown")
                
                pts = corners[i][0]
                top_left = pts[0]
                # 計算中心 x 座標以判斷左右
                cx = int(np.mean(pts[:, 0]))
                detected_markers.append((cx, marker_id, size_label))
                
                # 在畫面上標註
                cv2.putText(img, f"{size_label}", 
                            (int(top_left[0]), int(top_left[1])-30), 
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 0, 0), 2)
                cv2.putText(img, f"({int(top_left[0])}, {int(top_left[1])})", 
                            (int(top_left[0]), int(top_left[1])-10), 
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 0, 0), 2)

            draw_triple_rect(img, corners)
            
            if len(detected_markers) == 3:
                # 依 x 座標排序，小到大即為左到右
                detected_markers.sort(key=lambda m: m[0])
                left, mid, right = detected_markers[0], detected_markers[1], detected_markers[2]
                
                msg = String()
                msg.data = f"Left: {left[2]} (ID:{left[1]}), Mid: {mid[2]} (ID:{mid[1]}), Right: {right[2]} (ID:{right[1]})"
                node.publisher_.publish(msg)

        # 顯示結果
        cv2.imshow("contours", img)
        cv2.imshow("gray", gray)
        cv2.imshow("blurred", blurred)
        cv2.imshow("thresh", thresh)
        # cv2.imshow("canny", canny)
        # cv2.imshow("opening", opening)
        # cv2.imshow("closing", closing)
    
        key = cv2.waitKey(1)
        if key == 27:
            break
            
        rclpy.spin_once(node, timeout_sec=0.01)

    cap.release()
    cv2.destroyAllWindows()
    node.destroy_node()
    rclpy.shutdown()

def draw_triple_rect(frame, corners):
    for corner in corners:
        # 1. 取得四個角點並計算中心
        pts = corner.reshape((4, 2))
        (topLeft, topRight, bottomRight, bottomLeft) = pts
        
        cx = int((topLeft[0] + bottomRight[0]) / 2)
        cy = int((topLeft[1] + bottomRight[1]) / 2)

        # 2. 計算標記的當前像素寬度 (L)
        # 使用歐幾里得距離公式計算 topLeft 到 topRight 的距離
        L = np.sqrt((topLeft[0] - topRight[0])**2 + (topLeft[1] - topRight[1])**2)

        # 3. 計算 3 倍大小矩形的邊界
        # 半長度為 1.5 * L
        half_side = int(2.5 * L)
        
        rect_topLeft = (cx - half_side, cy - half_side)
        rect_bottomRight = (cx + half_side, cy + half_side)

        # 4. 畫出 3 倍大的藍色矩形
        cv2.rectangle(frame, rect_topLeft, rect_bottomRight, (255, 0, 0), 2)
        
        # 畫個紅點標示中心，方便你驗證
        cv2.circle(frame, (cx, cy), 5, (0, 0, 255), -1)

    return frame

if __name__ == "__main__":
    process_image()