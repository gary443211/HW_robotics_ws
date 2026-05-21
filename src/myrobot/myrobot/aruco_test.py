import cv2
import cv2.aruco as aruco

def detect_aruco():
    cap = cv2.VideoCapture(1) # 開啟你的內建或外接鏡頭 
    aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_4X4_50)
    parameters = aruco.DetectorParameters()
    detector = aruco.ArucoDetector(aruco_dict, parameters)

    # 定義 ID 與大小的對應關係
    id_to_size = {1: "Large", 2: "Medium", 3: "Small"}

    while True:
        ret, frame = cap.read()
        if not ret: break

        # 偵測 Marker
        corners, ids, rejected = detector.detectMarkers(frame)

        if ids is not None:
            # 畫出偵測到的框
            aruco.drawDetectedMarkers(frame, corners, ids)
            
            for i in range(len(ids)):
                marker_id = ids[i][0]
                size_label = id_to_size.get(marker_id, "Unknown")
                
                # 在畫面上標註
                corner = corners[i][0][0] # 取得左上角座標
                cv2.putText(frame, f"{size_label} (ID:{marker_id})", 
                            (int(corner[0]), int(corner[1])-10), 
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)

        cv2.imshow('ArUco Detection', frame)
        if cv2.waitKey(1) & 0xFF == ord('q'): break

    cap.release()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    detect_aruco()