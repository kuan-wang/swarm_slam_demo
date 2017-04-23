import cv2
import numpy as np


cap0 = cv2.VideoCapture(0)
# cap0.set(6,cv2.VideoWriter_fourcc('M', 'J', 'P', 'G'))
# cap0.set(3,320)
# cap0.set(4,240)

cap1 = cv2.VideoCapture(1)
# cap1.set(6,cv2.VideoWriter_fourcc('M', 'J', 'P', 'G'))
cap1.set(3,1280)
cap1.set(4,480)



while(1):
    ret0, frame0 = cap0.read()
    if ret0 == True:
        cv2.imshow("capture0", frame0)
    if cv2.waitKey(10) & 0xFF == ord('q'):
        break

    ret1, frame1 = cap1.read()
    print cap1.get(3),"x",cap0.get(4)
    print cap1.get(6)
    if ret1 == True:
        cv1.imshow("capture1", frame1)

    if cv1.waitKey(10) & 0xFF == ord('q'):
        break

cap0.release()
cap1.release()
cv2.destroyAllWindows()
