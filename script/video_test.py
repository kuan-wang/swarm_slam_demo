import cv2

cap = cv2.VideoCapture(0)

while (1):
    ret,frame = cap.read()
    if(ret):
        cv2.imshow("frame",frame)
        if cv2.waitKey(20) & 0xFF < 80:
            break
