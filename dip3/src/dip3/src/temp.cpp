import cv2
import numpy as np
 
img = cv2.imread('test19.jpg')
img1 = img.copy()
img2 = img.copy()
img = cv2.GaussianBlur(img, (3, 3), 0)
gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
edges = cv2.Canny(gray, 50, 150, apertureSize=3)
lines = cv2.HoughLines(edges, 1, np.pi/180, 110)
 
for line in lines:
    rho = line[0][0]
    theta = line[0][1]
    a = np.cos(theta)
    b = np.sin(theta)
    x0 = a*rho
    y0 = b*rho
    x1 = int(x0 + 1000*(-b))
    y1 = int(y0 + 1000*(a))
    x2 = int(x0 - 1000*(-b))
    y2 = int(y0 - 1000*(a))
 
    cv2.line(img1, (x1, y1), (x2, y2), (0, 0, 255), 2)
 
lines = cv2.HoughLinesP(edges, 1, np.pi/180, 30, 300, 5)
 
for line in lines:
    x1 = line[0][0]
    y1 = line[0][1]
    x2 = line[0][2]
    y2 = line[0][3]
    cv2.line(img2, (x1, y1), (x2, y2), (0, 255, 0), 2)
 
cv2.imshow('houghlines3', img1)
cv2.imshow('edges', img2)
cv2.waitKey(0)
print(lines)