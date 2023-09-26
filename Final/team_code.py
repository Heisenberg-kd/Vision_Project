import gym
import numpy as np
import vision_arena
import time
import pybullet as p
import pybullet_data
import cv2
import cv2.aruco as aruco
import numpy.linalg as la


nodes = np.full((9,9),0,np.uint8)
k=0
for i in range(9):
    for j in range(9):
        nodes[i][j]=k

        k=k+1
adj = np.full((81,81),0,np.uint8)
for i in range(8):
        adj[nodes[0][i]][nodes[0][i+1]]=1
        adj[nodes[i][8]][nodes[i+1][8]] = 1
        adj[nodes[8-i][0]][nodes[7-i][0]] = 1
        adj[nodes[8][8-i]][nodes[8][7-i]] = 1
        if(i!=4) and i!=3 and i!=2 and  i!=5:
            adj[nodes[i][4]][nodes[i+1][4]]=1
            adj[nodes[i + 1][4]][nodes[i][4]] = 1
            adj[nodes[4][i]][nodes[4][i+1]] = 1
            adj[nodes[4][i+1]][nodes[4][i]] = 1
        if i>1 and i<6:
            adj[nodes[2][i]][nodes[2][i + 1]] = 1
            adj[nodes[i][6]][nodes[i + 1][6]] = 1
            adj[nodes[8 - i][2]][nodes[7 - i][2]] = 1
            adj[nodes[6][8 - i]][nodes[6][7 - i]] = 1



pos = np.full((9,9),0,np.uint8)
def unit_vector(vector):
    return vector / np.linalg.norm(vector)

def angle(vector1, vector2):
    v1_u = unit_vector(vector1)
    v2_u = unit_vector(vector2)
    minor = np.linalg.det(
        np.stack((v1_u[-2:], v2_u[-2:]))
    )

    return np.sign(minor) * np.arccos(np.clip(np.dot(v1_u, v2_u), -1.0, 1.0))
def py_agl(v1, v2):
    cosang = np.dot(v1, v2)
    sinang = la.norm(np.cross(v1, v2))
    return np.arctan2(sinang, cosang)
def final_angle(v1,v2):
    angle1=np.degrees(angle(v1,v2))
    angle2=np.degrees(py_agl(v1,v2))
    if angle1==0:
        return angle2
    else:
        return angle1
def align(v1,v2):
    
    agl=final_angle(v1,v2)
    if agl<=-160:
        p.stepSimulation()
        env.move_husky(27, -27, 27, -27)
        p.stepSimulation()
        env.move_husky(15, -15, 15, -15)
    if agl<=180 and agl>=160:
        p.stepSimulation()
        env.move_husky(-27, 27, -27, 27)
        p.stepSimulation()
        env.move_husky(-15, 15, -15, 15)
        # p.stepSimulation()
        # env.move_husky(-0.5, -0.5, 0, 0)
        # print('R')
    elif agl<0:
        p.stepSimulation()
        env.move_husky(7, -7, 7, -7)
        p.stepSimulation()
        env.move_husky(5, -5, 5, -5)
        # p.stepSimulation()
        # env.move_husky(-0.5, -0.5, 0, 0)
        # print('R')
    elif agl>0:
        p.stepSimulation()
        env.move_husky(-7, 7, -7, 7)
        p.stepSimulation()
        env.move_husky(-5, 5, -5, 5)
        # p.stepSimulation()
        # env.move_husky(-0.5,-0.5, 0, 0)
        # print('L')
    
        

def BFS(s,k):
    par = np.full((9,9),0,np.uint8)
    dist = np.full((9,9),0,np.uint8)
    
    visited = [False] * (81)

    
    queue = []

    
    queue.append(s)
    visited[s] = True
    dist[int(s / 9)][s % 9]=0

    while queue:


        s = queue.pop(0)
        #print(s, end=" ")
        if pos[int(s/9)][s%9]==k and dist[int(s / 9)][s % 9]>0:
            # print(s)
            return s,par
        

        for i in range(81):
            if adj[s][i]==1:
                if visited[i] == False:
                    dist[int(i/9)][i%9]=dist[int(s/9)][s%9]+1
                    queue.append(i)
                    par[int(i/9)][i%9]=s
                    visited[i] = True
    return -1,-1

def bot_pos(img):
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    corners, ids, rejectedImgPoints = aruco.detectMarkers(gray, ARUCO_DICT, parameters=ARUCO_PARAMETERS)
    corners=np.asarray(corners)
    
        # for cnrs in corners:
        #     cen1 = cv2.moments(cnrs)  # we find the centre of that shape we can approx ravel.
        #     cenx1 = int(cen1["m10"] / cen1["m00"])
        #     ceny1 = int(cen1["m01"] / cen1["m00"])
        #     n=nodes[int(ceny1/x)][int(cenx1/y)]
        #     print(n)
            # x1=int(int(n/9)*x)
            # y1=int(int(n%9)*y)
            # x1=abs(cenx1-x1)
            # y1=abs(ceny1-y1)
            # print(x1,y1)
            # if (x1+y1)<25:
            #    print(n) 
            # print(cenx1,",",ceny1)
        #print(corners.shape())
        # Print corners and ids to the console
        # for i, corner in zip(ids, corners):
            # print('ID: {}; Corners: {}'.format(i, corner))
    img = aruco.drawDetectedMarkers(img, corners, borderColor=(0, 0, 255))
    #cv2.imshow('img',img)
    if ids is not None:
        x1=(corners[0][0][2][1]+corners[0][0][3][1])/2
        y1=(corners[0][0][2][0]+corners[0][0][3][0])/2
        x2=(corners[0][0][0][1]+corners[0][0][1][1])/2
        y2=(corners[0][0][0][0]+corners[0][0][1][0])/2
        X=(x1+x2)/2
        Y=(y1+y2)/2
        l=int((x1)/x)
        h=int(y1/y)
        L=int(X/x)
        H=int(Y/y)
        # if l>8:
        #     l=8
        # if h>8:
        #     h=8
        # if L>8:
        #     L=8
        # if H>8:
        #     H=8
        
        n=nodes[l][h]
        c=nodes[L][H]
        vect=[(y2-y1),(x1-x2)]
        return c,n,vect,x1,y1
    return -1,-1,-1,-1,-1
if __name__=="__main__":
    env = gym.make("vision_arena-v0")
    ARUCO_PARAMETERS = aruco.DetectorParameters_create()
    ARUCO_DICT = aruco.Dictionary_get(aruco.DICT_ARUCO_ORIGINAL)
    board = aruco.GridBoard_create(
        markersX=2,
        markersY=2,
        markerLength=0.09,
        markerSeparation=0.01,
        dictionary=ARUCO_DICT)
    imA = env.camera_feed()
    r = cv2.selectROI(imA)
    imgA = imA[int(r[1]):int(r[1]+r[3]), int(r[0]):int(r[0]+r[2])]
    x=imgA.shape[0]/9
    y=imgA.shape[1]/9


    hsv=cv2.cvtColor(imgA,cv2.COLOR_BGR2HSV)
    #---------FOR_RED--------------
    lv_red=np.array([0,31,50])
    uv_red=np.array([16,255,255])
    lv2_red=np.array([170,31,50])
    uv2_red=np.array([180,255,255])
    mask_red=cv2.inRange(hsv,lv_red,uv_red)
    mask2_red=cv2.inRange(hsv,lv2_red,uv2_red)
    res_red=cv2.addWeighted(mask_red,1,mask2_red,1,0)
    res1_red = cv2.bitwise_and(imgA, imgA, mask=res_red)
    res1_red[np.where((res1_red==[0,0,0]).all(axis=2))]=[255,255,255]
    ct_r, _ = cv2.findContours(res_red, cv2.RETR_TREE, cv2.CHAIN_APPROX_NONE)
    for cnt in ct_r:
        area = cv2.contourArea(cnt)
        cen = cv2.moments(cnt)
        cenx = int(cen["m10"] / cen["m00"])
        ceny = int(cen["m01"] / cen["m00"])
        if area > 400:
            approx = cv2.approxPolyDP(cnt, 0.02 * cv2.arcLength(cnt, True), True)
            if(len(approx)==3):
                #cv2.fillPoly(img, pts=cnt, color=(0,0,0))
                cv2.drawContours(imgA, cnt, -1, (0, 255, 0), 3)
                cv2.putText(imgA, "1", (cenx, ceny), cv2.FONT_HERSHEY_COMPLEX, 0.5, (0, 0, 0))
                pos[int(ceny/y)][int(cenx/x)] = 1
            elif(len(approx)==4):
                #cv2.fillPoly(img, pts=cnt, color=(0,0,0))
                cv2.drawContours(imgA, cnt, -1, (0, 255, 0), 3)
                cv2.putText(imgA, "2", (cenx, ceny), cv2.FONT_HERSHEY_COMPLEX, 0.5, (0, 0, 0))
                pos[int(ceny/y)][int(cenx/x)] = 2
            elif(len(approx)>5):
                #cv2.fillPoly(img, pts=cnt, color=(0,0,0))
                cv2.drawContours(imgA, cnt, -1, (0, 255, 0), 3)
                cv2.putText(imgA, "3", (cenx, ceny), cv2.FONT_HERSHEY_COMPLEX, 0.5, (0, 0, 0))
                pos[int(ceny/y)][int(cenx/x)] = 3
    #----------FOR__Yellow------------
    lv_yellow=np.array([29,70,46])
    uv_yellow=np.array([52,255,255])
    mask_yellow=cv2.inRange(hsv,lv_yellow,uv_yellow)
    res_yellow = cv2.bitwise_and(imgA, imgA, mask=mask_yellow)
    res_yellow[np.where((res_yellow==[0,0,0]).all(axis=2))]=[255,255,255]
    ct_y, _ = cv2.findContours(mask_yellow, cv2.RETR_TREE, cv2.CHAIN_APPROX_NONE)
    for cnt in ct_y:
        area = cv2.contourArea(cnt)
        cen = cv2.moments(cnt)
        cenx = int(cen["m10"] / cen["m00"])
        ceny = int(cen["m01"] / cen["m00"])
        if area > 400:
            approx = cv2.approxPolyDP(cnt, 0.02 * cv2.arcLength(cnt, True), True)
            if(len(approx)==3):
                #cv2.fillPoly(img, pts=cnt, color=(0,0,0))
                cv2.drawContours(imgA, cnt, -1, (0, 255, 0), 3)
                cv2.putText(imgA, "4", (cenx, ceny), cv2.FONT_HERSHEY_COMPLEX, 0.5, (0, 0, 0))
                pos[int(ceny/y)][int(cenx/x)] = 4
            elif(len(approx)==4):
                #cv2.fillPoly(img, pts=cnt, color=(0,0,0))
                cv2.drawContours(imgA, cnt, -1, (0, 255, 0), 3)
                cv2.putText(imgA, "5", (cenx, ceny), cv2.FONT_HERSHEY_COMPLEX, 0.5, (0, 0, 0))
                pos[int(ceny/y)][int(cenx/x)] = 5
            elif(len(approx)>5):
                #cv2.fillPoly(imgA, pts=cnt, color=(0,0,0))
                cv2.drawContours(imgA, cnt, -1, (0, 255, 0), 3)
                cv2.putText(imgA, "6", (cenx, ceny), cv2.FONT_HERSHEY_COMPLEX, 0.5, (0, 0, 0))
                pos[int(ceny/y)][int(cenx/x)] = 6
    # cv2.imshow('image_array',imgA)   
    start=1
    startnode=-1


    while True:
        p.stepSimulation()
        #env.move_husky(10, 10, 10,10)
        im = env.camera_feed()
        img = im[int(r[1]):int(r[1]+r[3]), int(r[0]):int(r[0]+r[2])]
        c,n,v1,x1,y1=bot_pos(img)
        while v1==-1:
            p.stepSimulation()
            env.move_husky(5,-5,5,-5)
            img1 = env.camera_feed()
            c,n,v1,x1,y1=bot_pos(img1)

        #print('current node---',c)
        if start==1:
            startnode=c
            start=0
        if start==0:
            if startnode==4:
                if c!=4 and c!=13 and c!=22:
                    adj[22][31]=1
                    #adj[22][13]=0 ##########-------TRICK------###########
                    adj[4][5]=0
                    adj[22][23]=0
                    if(pos[1][4]==pos[3][4]):
                        adj[22][13]=0
                    start=-1
                    
            elif startnode==44:
                if c!=44 and c!=43 and c!=42:
                    adj[42][41]=1
                    #adj[42][43]=0
                    adj[44][53]=0
                    adj[42][51]=0
                    if(pos[4][5]==pos[4][7]):
                        adj[42][43]=0
                    start=-1
            elif startnode==76:
                if c!=76 and c!=67 and c!=58:
                    adj[58][49]=1
                    #adj[58][67]=0
                    adj[76][75]=0
                    adj[58][57]=0
                    if(pos[7][4]==pos[5][4]):
                        adj[58][67]=0
                    start=-1
            elif startnode==36:
                if c!=36 and c!=37 and c!=38:
                    adj[38][39]=1
                    #adj[38][37]=0
                    adj[36][27]=0
                    adj[38][29]=0
                    if(pos[4][1]==pos[4][3]):
                        adj[38][37]=0
                    start=-1
        if startnode==4:
            if c==31:
                print('HOME')
                x2=(n%9)*y+26
                y2=(int(n/9))*x+26
                while n!=40:
                    while ag2>1:
                        # print('inner')
                        # print(fa)
                        p.stepSimulation()
                        align(v1,v2)
                        img1=env.camera_feed()
                        c,n,v1,x1,y1=bot_pos(img1)
                        if(abs(x1-y2)<17):
                            v2=[(x2-y1),0]
                        elif(abs(x2-y1)<17):
                            v2=[0,(x1-y2)]
                        else:
                            v2=[(x2-y1),(x1-y2)]
                        ag2=np.degrees(py_agl(v1,v2))
                    if ag2<1:
                        p.stepSimulation()
                        env.move_husky(12,12,12,12)
                    img1=env.camera_feed()
                    c,n,v1,x1,y1=bot_pos(img1)

                print('task complete')
                time.sleep(2)
                break
        if startnode==36:
            if c==39:
                print('HOME')
                x2=(n%9)*y+26
                y2=(int(n/9))*x+26
                while n!=40:
                    while ag2>1:
                        # print('inner')
                        # print(fa)
                        p.stepSimulation()
                        align(v1,v2)
                        img1=env.camera_feed()
                        c,n,v1,x1,y1=bot_pos(img1)
                        if(abs(x1-y2)<17):
                            v2=[(x2-y1),0]
                        elif(abs(x2-y1)<17):
                            v2=[0,(x1-y2)]
                        else:
                            v2=[(x2-y1),(x1-y2)]
                        ag2=np.degrees(py_agl(v1,v2))
                    if ag2<1:
                        p.stepSimulation()
                        env.move_husky(12,12,12,12)
                    img1=env.camera_feed()
                    c,n,v1,x1,y1=bot_pos(img1)
                for i in range(20):
                    p.stepSimulation()
                    env.move_husky(10,10,10,10)
                print('task complete')
                time.sleep(5)
                break
        if startnode==44:
            if c==41:
                print('HOME')
                x2=(n%9)*y+26
                y2=(int(n/9))*x+26
                while n!=40:
                    while ag2>1:
                        # print('inner')
                        # print(fa)
                        p.stepSimulation()
                        align(v1,v2)
                        img1=env.camera_feed()
                        c,n,v1,x1,y1=bot_pos(img1)
                        if(abs(x1-y2)<17):
                            v2=[(x2-y1),0]
                        elif(abs(x2-y1)<17):
                            v2=[0,(x1-y2)]
                        else:
                            v2=[(x2-y1),(x1-y2)]
                        ag2=np.degrees(py_agl(v1,v2))
                    if ag2<1:
                        p.stepSimulation()
                        env.move_husky(12,12,12,12)
                    img1=env.camera_feed()
                    c,n,v1,x1,y1=bot_pos(img1)
                print('task complete')
                time.sleep(5)
                break
        if startnode==76:
            if c==49:
                print('HOME')
                x2=(n%9)*y+26
                y2=(int(n/9))*x+26
                while n!=40:
                    while ag2>1:
                        # print('inner')
                        # print(fa)
                        p.stepSimulation()
                        align(v1,v2)
                        img1=env.camera_feed()
                        c,n,v1,x1,y1=bot_pos(img1)
                        if(abs(x1-y2)<17):
                            v2=[(x2-y1),0]
                        elif(abs(x2-y1)<17):
                            v2=[0,(x1-y2)]
                        else:
                            v2=[(x2-y1),(x1-y2)]
                        ag2=np.degrees(py_agl(v1,v2))
                    if ag2<1:
                        p.stepSimulation()
                        env.move_husky(12,12,12,12)
                    img1=env.camera_feed()
                    c,n,v1,x1,y1=bot_pos(img1)
                print('task complete')
                time.sleep(2)
                break
        dnode=-1
        while dnode==-1:
            des=env.roll_dice()
            if des=="TR":
                d=1
            elif des=="SR":
                d=2
            elif des=="CR":
                d=3
            elif des=="TY":
                d=4
            elif des=="SY":
                d=5
            elif des=="CY":
                d=6
            dnode,prt=BFS(c,d)
            if dnode!=-1:
                print('DESTINATION IS--'+des)
        print(des+ ' is at node--',dnode)
        #print(prt)
        q=[]
        q.append(dnode)
        while dnode!=c:
            dnode=prt[dnode//9][dnode%9]
            if dnode!=c:
                q.append(dnode)
               
        print('********')
        
        print(q)
        print('********')
        while q:
            print('next node')
            dt=q.pop()
            print(dt)
            img1=env.camera_feed()
            
            c,n,v1,x1,y1=bot_pos(img1)
            while v1==-1:
                p.stepSimulation()
                env.move_husky(20,-20,20,-20)
                img1=env.camera_feed()
                
                c,n,v1,x1,y1=bot_pos(img1)
            x2=(dt%9)*y+26
            y2=(int(dt/9))*x+26
            if(abs(x1-y2)<17):
                v2=[(x2-y1),0]
            elif(abs(x2-y1)<17):
                v2=[0,(x1-y2)]
            else:
                v2=[(x2-y1),(x1-y2)]

            ag2=np.degrees(py_agl(v1,v2))
            
            # print(v1)
            # print(x2)
            # print(y2)
            # print(x1)
            # print(y1)
            # print(v2)
            #fa=final_angle(v1,v2)
            # print('in')
            # print(fa)
            
            while(n!=dt):
                
                while ag2>1:
                    # print('inner')
                    # print(fa)
                    p.stepSimulation()
                    align(v1,v2)
                    img1=env.camera_feed()
                    
                    c,n,v1,x1,y1=bot_pos(img1)
                    while v1==-1:
                        p.stepSimulation()
                        env.move_husky(20,-20,20,-20)
                        img1=env.camera_feed()
                        
                        c,n,v1,x1,y1=bot_pos(img1)
                    if(abs(x1-y2)<17):
                        v2=[(x2-y1),0]
                    elif(abs(x2-y1)<17):
                        v2=[0,(x1-y2)]
                    else:
                        v2=[(x2-y1),(x1-y2)]
                    ag2=np.degrees(py_agl(v1,v2))
                    #fa=final_angle(v1,v2)
                if ag2<1:
                    dist=abs(abs(v2[0])+abs(v2[1])-14)
                    
                    #print(dist)
                    if(dist>42):
                        p.stepSimulation()
                        env.move_husky(150,150,150,150)
                    else:
                        p.stepSimulation()
                        env.move_husky(6,6,6,6)
                img1=env.camera_feed()
                
                
                c,n,v1,x1,y1=bot_pos(img1)
                while v1==-1:
                    p.stepSimulation()
                    env.move_husky(20,-20,20,-20)
                    img1=env.camera_feed()
                    
                    c,n,v1,x1,y1=bot_pos(img1)
                if(abs(x1-y2)<19):
                    v2=[(x2-y1),0]
                elif(abs(x2-y1)<19):
                    v2=[0,(x1-y2)]
                else:
                    v2=[(x2-y1),(x1-y2)]
                ag2=np.degrees(py_agl(v1,v2))

            # for i in range(20):
            #     p.stepSimulation()
            #     env.move_husky(-12,-12,-12,-12)
            if n!=72 or (n!=76):
                for i in range(20):
                    p.stepSimulation()
                    env.move_husky(-4,-4,-4,-4)
            # if n==72:
            #     for i in range(20):
            #         p.stepSimulation()
            #         env.move_husky(-6,-6,-6,-6)
                #fa=final_angle(v1,v2)
        print('waiting for destination')
        #time.sleep(1)
        env.move_husky(-10,-10,-10,-10)
    #print(pos)
    cv2.destroyAllWindows()    
    p.disconnect()