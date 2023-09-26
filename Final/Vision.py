import gym
import vision_arena
import time
import pybullet as p
import pybullet_data
import cv2
import numpy as np
import os

def morp(mask):
    kernel=np.ones((3,3),np.uint8)
    mask=cv2.erode(mask,kernel,iterations=8)
    mask=cv2.dilate(mask,kernel,iterations=11)

# ar=np.zeroes(9,9)
parent_path = os.path.dirname(os.getcwd())
os.chdir(parent_path)
env = gym.make("vision_arena-v0")
time.sleep(3)
img = env.camera_feed(is_flat=True)
hsv=cv2.cvtColor(img,cv2.COLOR_BGR2HSV)
mask=cv2.inRange(hsv,np.array(([25,97,106])),np.array([32,255,255]))
mask=cv2.erode(mask,np.ones((5,5),np.uint8),iterations = 1)
morp(mask)
res=cv2.bitwise_and(img,img,mask=mask)
cont,_=cv2.findContours(mask,cv2.RETR_TREE,cv2.CHAIN_APPROX_NONE)
# cv2.imshow('mask',mask)
cv2.imshow('res',res)
for cnt in cont:
    area = cv2.contourArea(cnt)
    approx = cv2.approxPolyDP(cnt, 0.03*cv2.arcLength(cnt, True), True)
    # if a==0:           
    if len(approx) == 3:
        cv2.drawContours(res, [cnt], 0, (0, 255, 0), 3)          
    if len(approx) == 4:
        cv2.drawContours(res, [cnt], 0, (0, 255, 0), 3)
    else:
        cv2.drawContours(res, [cnt], 0 , 0, cv2.FILLED)           
with_cont=cv2.resize(res,(400,320))
cv2.imshow('cont',with_cont)

# for cnt in ct_y:
#     area = cv2.contourArea(cnt)
#     cen = cv2.moments(cnt)
#     cenx = int(cen["m10"] / cen["m00"])
#     ceny = int(cen["m01"] / cen["m00"])



cv2.waitKey(5)
while True:
    p.stepSimulation()
    env.move_husky(5,5,5,5)
    if cv2.waitKey(5)==ord('q'):
        cv2.destroyAllWindows()
        break
p.disconnect()