import cv2
import cv2.aruco as aruco
import numpy as np

# 카메라 매트릭스와 왜곡 계수 (캘리브레이션 후 얻은 값)
cameraMatrix = np.array([[1.38992675e+03, 0.00000000e+00, 1.09573766e+03],
                        [0.00000000e+00, 1.39126023e+03, 7.02526462e+02],
                        [0.00000000e+00, 0.00000000e+00, 1.00000000e+00]])
distCoeffs = np.array([0.13879985, -0.35371541, -0.00368127, -0.00311165, 0.22808276])

# 아루코 마커 감지
image = cv2.imread('/home/rokey/check0.jpg')
gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
aruco_dict = aruco.Dictionary(aruco.DICT_5X5_100, 5)
parameters = aruco.DetectorParameters()
corners, ids, rejectedImgPoints = aruco.detectMarkers(gray, aruco_dict, parameters=parameters)

# 마커의 위치와 회전 추정
markerLength = 0.1  # 마커의 실제 크기 (미터 단위)
rvec, tvec, _ = aruco.estimatePoseSingleMarkers(corners, markerLength, cameraMatrix, distCoeffs)

# 추정된 위치와 회전 출력
print("Rotation Vector:", rvec)
print("Translation Vector:", tvec)

# 마커에 위치와 회전 정보 그리기
for i in range(len(ids)):
    aruco.drawAxis(image, cameraMatrix, distCoeffs, rvec[i], tvec[i], markerLength)
    
cv2.imshow('Frame', image)
cv2.waitKey(0)
cv2.destroyAllWindows()
