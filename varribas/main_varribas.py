# Copyright (c) 2016
# Author: Victor Arribas <v.arribas.urjc@gmail.com>
# License: GPLv3 <http://www.gnu.org/licenses/gpl-3.0.html>

from modules.qtimshow import imshow
from .print_once import print_once
import safety_distance

from pid import PID

import cv2
import numpy as np
import math


__author__ = 'varribas'


debug = [0,1,2,3]
debug = [0,1,21,22,3,-1]
debug = [0,-1,-11]
#debug = [-1, -11]


target_depth = 1.75
# target_depth_up = 2.5
depth_estimation_offset = +0.25  # over estimation (drone width)
depth_under_reverse_left = 2.00


use_exponential_remap = True

class EntryPoint:
    def __init__(self, sensor):
        self.sensor = sensor

        ''' A lot of PID due:
            1. Try to use a data space and PID params that saturates near 2.0 (external limit)
                Otherways, PID control becomes broken due difference between expected and real output
            2. Compensate dynamics offset. High velocity implies bigger offset
        '''
        if not use_exponential_remap:
            self.pid_forward = PID(0,  1.15, 1.00, 0.10,   6)  # 3D space (perfect)          [OK] #(0,  1.50, 0.50, 0.05,  12)
            self.pid_up      = PID(0,  1.75, 2.50, 0.40,   8)  # fov space percentage (good) []   #(0,  1.75, 3.00, 0.20,  12)
            self.pid_left    = PID(0,  1.00, 0.60, 0.10,  12)  # fov space percentage (good) [OK]
            self.pid_rot     = PID(0,  1.25, 0.75, 0.15,  12)  # fov space arbitrary (ok)    [OK] #(0,  1.15, 0.30, 0.10,  12)
        else:
            # # 2*(1-1/(1+abs(dd)))**2 [bad]
            # self.pid_forward = PID(0,  1.75, 2.50, 0.30,   6)  # ^2 3D space (bad)                 [] #(0,  2.00, 2.50, 0.30,   6)
            # self.pid_fwd_int = PID(0,  0.01, 0.30, 0.04,   6)  # compensate limited I due fast reaction (removed inertia) #(0,  0.01, 0.20, 0.03,   6)
            # 2*(1-1/(1+abs(dd)))  [better]
            self.pid_forward = PID(0,  1.15, 1.25, 0.15,   4)  # ^2 3D space (ok)                  []
            self.pid_fwd_int = PID(0,  0.00, 0.00, 0.00,   6)  # compensate limited I due fast reaction (removed inertia) #(0, 0.01, 0.10, 0.01) #(0,  0.02, 0.05, 1e-3,   6)
            self.pid_fwd_stb   = PID(0,  0.20, 0.20, 0.10,  24) # zero when others I erros non-zero
            self.pid_up      = PID(0,  1.50, 2.00, 0.20,   6)  # ^2 fov space percentage (perfect) [] #(0,  1.75, 2.50, 0.40,   8)
            self.pid_up_int  = PID(0,  0.01, 0.25, 0.02,   6)
            self.pid_left    = PID(0,  1.00, 0.60, 0.10,  12)  # ^2 fov space percentage (perfect) []  #(0,  1.00, 0.60, 0.10,  12)
            self.pid_rot     = PID(0,  1.40, 0.75, 0.15,  12)  # ^2 fov space arbitrary (good)     []  #(0,  1.25, 0.75, 0.15,  12)

        self.median_filter = MedianFilter(5)


        self.search_mode_it = 0
        self.search_mode_lastXY = None
        self.search_mode_up_dir  = -1
        self.search_mode_rot_dir = +1
        print 'search_mode_up_dir',self.search_mode_up_dir

    def execute(self):
        print_once('Executing varribas')

        ### ensure active connection
        pose = self.sensor.getPose3D()
        if pose is None:
            return

        ### bootstrap
        if pose.z < 0.2:
            print_once('BOOTSTRAP')
            self.sensor.toggleCam()
            self.sensor.takeoff()
            self.sensor.sendCMDVel(0,0,0,0,0,0)


        ## thresholding
        img = self.sensor.getImage()
        if 0 in debug:
            imshow('first-person view', img)


        hsv = cv2.cvtColor(img, cv2.COLOR_RGB2HSV)
        mask = cv2.inRange(hsv, (50,65,20), (60,255,250))
        mask_patch = cv2.inRange(hsv, (0,0,0), (1,1,1))
        mask = cv2.bitwise_or(mask, mask_patch)
        mask = cv2.morphologyEx(mask, op=cv2.MORPH_CLOSE, kernel=cv2.getStructuringElement(cv2.MORPH_CROSS, (3,3)))

        if 1 in debug:
            imshow('d1:mask', mask)#np.dstack((mask,mask,mask)))
            # img_mask = cv2.bitwise_and(img, img, mask=mask)
            # imshow('d1:mask2', img_mask)


        # detect horizon
        horizon = cv2.inRange(hsv, (0,0,160), (10,10,180))
        if 100 in debug:
            imshow('d100:horizon', horizon)
        if cv2.countNonZero(horizon) < 200:
            self.sensor.toggleCam()
            print 'INFO: toggling camera!'
            return


        ## Segmentation
        contours, hierarchy = cv2.findContours(mask, mode=cv2.RETR_EXTERNAL, method=cv2.CHAIN_APPROX_SIMPLE)
        contour, index = best_contours(contours)

        if 2 in debug:
            im_contours = img.copy()
            cv2.drawContours(im_contours, contours, index, (255,0,0), 1)
            imshow('d2:contour', im_contours)

        # not found
        if index == -1:
            print_once('[ON]  SEARCH MODE\n')
            if self.search_mode_it == 0:
                self.sensor.sendCMDVel(0,0,0,0,0,0)
            self.searchMode()
            return
        elif self.search_mode_it > 0:
            print_once('[OFF] SEARCH MODE\n')
            self.search_mode_it = 0
            self.search_mode_lastXY = None
            self.sensor.sendCMDVel(0,0,0,0,0,0)


        # join nearest contours
        n = len(contours)
        if 21 in debug:
            img_ = img.copy()
            color = [0,0,255]
            for i in range(n):
                if i % 4 == 3: color = [255-v for v in color] # reverse palette (x6)
                color = color[1:3] + [color[0]] # r,g,b,r,g,b... (x3)
                cv2.drawContours(img_, contours, i, color, 2)
            imshow('d2.1:contours', img_)

        # simple way #1: fill all contours and redo find step
        # * sensitive to missdetections!
        # * requires dilate and value tuning
        # mask_new = np.zeros(mask.shape, dtype=np.uint8)
        # cv2.drawContours(mask_new, contours, -1, (255), -1)
        # cv2.drawContours(mask_new, contours, -1, (255), 5)
        # contours, hierarchy = cv2.findContours(mask_new, mode=cv2.RETR_EXTERNAL, method=cv2.CHAIN_APPROX_NONE)
        # contour, index = best_contours(contours)
        # if len(contours) > 1:
        #     print '  warning: joined contours > 1'
        # if 22 in debug:
        #     imshow('d2.2:joined', mask_new)

        # simple way #2: assume outliers are little
        # * possible wrong assumption
        # * easy to detect outliers but requires tuning
        join_contour = []
        for i in range(n):
            area = cv2.contourArea(contours[i], False)
            if area > 10:
                join_contour.extend(contours[i])
        if len(join_contour) > 0:
            contour = np.asarray(join_contour)
        if 22 in debug:
            img_ = img.copy()
            cv2.drawContours(img_, [contour], 0, (255,0,0), 2)
            imshow('d2.2:contours', img_)


        ## width estimation
        rect = cv2.boundingRect(contour)
        rot_rect = cv2.minAreaRect(contour)  # ((x,y), (w,h), angle)
        w,h = rot_rect[1]
        width = max(w,h)

        (_,__,w,h) = rect
        width = max(w,h)


        if 3 in debug:
            x = int(rot_rect[0][0])
            y = int(rot_rect[0][1])
            a = rot_rect[2]
            h = width/2.0
            img_width = img.copy()
            hx = int(math.cos(a)*h)
            hy = int(math.sin(a)*h)
            cv2.line(img_width, (x-hx, y-hy), (x+hx, y+hy), (255,0,0), 2 )
            x,y,w,h = rect
            cv2.rectangle(img_width, (x,y), (x+w,y+h), (255,0,255), 1)
            imshow('d3:width', img_width)


        ## depth estimation
        real_width = 0.5  # m (drone width)
        horizontal_fov = 1.413717 # rad (camera params)
        image_width = mask.shape[1]
        focal_distance = (image_width/2) / math.tan(horizontal_fov/2)

        # x/f = X/Z
        depth = real_width * focal_distance / float(width)

        # depth filtering
        depth += depth_estimation_offset
        depth = self.median_filter.feed(depth)


        ## attitude deviation


        tx,ty = rot_rect[0]
        (ry,rx) = np.array(mask.shape)/2.0

        dx = (rx-tx)/rx
        dy = (ry-ty)/ry
        dd = (depth-target_depth)
        dr = math.atan(dx) #/(horizontal_fov/2)

        self.search_mode_lastXY = (dx,dy)

        # enforce be a little bit high
        dy += 0.05 # %

        # ## 3D measures
        # dy = - (ty-ry) * depth/focal_distance
        # dx = - (tx-rx) * depth/focal_distance

        # exponential remap (only for image measures as percentages)
        ddx = np.sign(dx) * 2*dx**2
        ddy = np.sign(dy) * 2*dy**2
        ddr = np.sign(dr) * 2*dr**2
        ddd = np.sign(dd) * 2*(1-1/(1+abs(dd)))
        #for i in range(0,100): dd=(i-50)/10.0; print dd,'-->',2*(1-1/(1+abs(dd)))**2 * np.sign(dd)

        ## velocity accommodation
        v_forward=v_up=v_left=v_rot=0

        # linear vs. exponential
        if not use_exponential_remap:
            v_forward = self.pid_forward.feedback(dd)
            v_left    = self.pid_left.feedback(dx)
            v_up      = self.pid_up.feedback(dy)
            v_rot     = self.pid_rot.feedback(dr)
        else:
            # if np.sign(dd) * np.sign(self.pid_fwd_int._errI) > 0:
            #     self.pid_fwd_int.I_len = min(48, self.pid_fwd_int.I_len+1)
            # else:
            #     self.pid_fwd_int.I_len = max(6, self.pid_fwd_int.I_len-2)

            v_forward = self.pid_forward.feedback(ddd) + self.pid_fwd_int.feedback(self.pid_forward._errI)
            v_left    = self.pid_left.feedback(ddx)
            v_up      = self.pid_up.feedback(ddy) + self.pid_up_int.feedback(self.pid_up._errI)
            v_rot     = self.pid_left.feedback(ddr)

            stability = max(0, 1 - (abs(self.pid_up._errI) + abs(self.pid_left._errI) + abs(self.pid_rot._errI)))
            _ddd = 2*(1-1/(1+abs(dd)))**2
            self.pid_fwd_stb.feedback( _ddd*stability )

            v_forward += self.pid_fwd_stb.feedback()

        # dead zone
        if   abs(dd) < 0.001: v_forward = 0
        if   abs(dr) < 0.025: v_rot = 0
        if   abs(dy) < 0.025: v_up = 0
        if   abs(dx) < 0.025: v_left = 0

        # safety constrains
        if depth <= safety_distance.safety_static_distance_XZ:
            print 'SAFETY CONSTRAINT! (static)'
            v_forward = min(v_forward, -2.0)
        elif depth <= safety_distance.safety_dynamic_distance_XZ:
            print 'SAFETY CONSTRAINT! (dynamic)'
            v_forward = min(v_forward, -0.5)
        else:
            pass #v_forward *= 1-ddy

        if depth < depth_under_reverse_left:
            print 'SAFETY CONSTRAINT! (reverse left)'
            v_left *= -1.00
            v_rot  *= +2.00


        if -1 in debug:
            print '''velocities (unclipped)
  forward: %.2f
  left: %.2f
  up: %.2f
  rot: %.2f''' %(v_forward, v_left, v_up, v_rot)


        # official clipping
        v_left = np.clip(v_left,       -self.sensor.MAX_LINX_SIM, self.sensor.MAX_LINX_SIM)
        v_forward = np.clip(v_forward, -self.sensor.MAX_LINY_SIM, self.sensor.MAX_LINY_SIM)
        v_up = np.clip(v_up,           -self.sensor.MAX_LINZ_SIM, self.sensor.MAX_LINZ_SIM)
        v_rot = np.clip(v_rot,         -self.sensor.MAX_ANGZ_SIM, self.sensor.MAX_ANGZ_SIM)


        ## send velocities
        self.sensor.sendCMDVel(v_left, v_forward, v_up, v_rot, 0, 0)  # left,forward,up

        if -1 in debug:
            print '''velocities (depth: %.2f)
  forward: %.2f
  left: %.2f
  up: %.2f
  rot: %.2f''' %(depth,  v_forward, v_left, v_up, v_rot)
            print 'dZ:%+.2f  dx:%+.2f dy:%+.2f  dR:%+.2f' %(dd,dx,dy,dr)
            if use_exponential_remap:
                 print 'ddZ:%+.2f  ddx:%+.2f ddy:%+.2f  ddR:%+.2f' %(ddd,ddx,ddy,ddr)

        if -11 in debug:
            print 'PID[fwd] %+.2f values: P=%+.2f D=%+.2f  I=%+.2f' %(self.pid_forward._feedback, self.pid_forward._errP, self.pid_forward._errD, self.pid_forward._errI)
            print 'PID[up ] %+.2f values: P=%+.2f D=%+.2f  I=%+.2f' %(self.pid_up._feedback, self.pid_up._errP, self.pid_up._errD, self.pid_up._errI)
            print 'PID[lft] %+.2f values: P=%+.2f D=%+.2f  I=%+.2f' %(self.pid_left._feedback, self.pid_left._errP, self.pid_left._errD, self.pid_left._errI)
            print 'PID[rot] %+.2f values: P=%+.2f D=%+.2f  I=%+.2f' %(self.pid_rot._feedback, self.pid_rot._errP, self.pid_rot._errD, self.pid_rot._errI)

            if use_exponential_remap:
                print 'PID[Ifw] %+.2f values: P=%+.2f D=%+.2f  I=%+.2f (h=%d)' %(self.pid_fwd_int._feedback, self.pid_fwd_int._errP, self.pid_fwd_int._errD, self.pid_fwd_int._errI, self.pid_fwd_int.I_len)
                print 'PID[Iup] %+.2f values: P=%+.2f D=%+.2f  I=%+.2f' %(self.pid_up_int._feedback, self.pid_up_int._errP, self.pid_up_int._errD, self.pid_up_int._errI)

                print 'PID[stb] %+.2f values: P=%+.2f D=%+.2f  I=%+.2f' %(self.pid_fwd_stb._feedback, self.pid_fwd_stb._errP, self.pid_fwd_stb._errD, self.pid_fwd_stb._errI)



    def searchMode(self):
        self.search_mode_it += 1

        helicoidal_search_up = 0.75
        helicoidal_search_rot = 1.5

        if self.search_mode_lastXY is not None:
            tx,ty = self.search_mode_lastXY
            self.search_mode_up_dir = np.sign(ty)
            self.search_mode_rot_dir = np.sign(tx)

        pose = self.sensor.getPose3D()
        if pose.z < 0.4:
            self.search_mode_up_dir = +1
            self.search_mode_lastXY = None
            print_once('Search Mode: going UP...')
        elif pose.z > 7:
            self.search_mode_up_dir = -1
            self.search_mode_lastXY = None
            print_once('Search Mode: going DOWN...')

        v_up = helicoidal_search_up * self.search_mode_up_dir
        v_rot = helicoidal_search_rot * self.search_mode_rot_dir
        
        self.sensor.sendCMDVel(0,0, v_up, v_rot, 0,0)  # left,forward,up


# aux functions
def best_contours(contours):
    best_area = 0
    best_contour = None
    best_index = -1

    index = -1
    for contour in contours:
        index +=1
        area = cv2.contourArea(contour, False)
        if area > best_area:
            best_area = area
            best_contour = contour
            best_index = index

    return best_contour, best_index


def clip_lt(value, cond, multiplier, outvalue):
    if abs(value) < cond:
        return outvalue*multiplier
    else:
        return outvalue


def clip_gt(value, cond, multiplier, outvalue):
    if abs(value) > cond:
        return outvalue*multiplier
    else:
        return outvalue


#other functions
def dump(obj):
    for attr in dir(obj):
        print "obj.%s = %s" % (attr, getattr(obj, attr))


def exit():
    import sys
    sys.exit(0)


class MedianFilter():
    def __init__(self, H=5):
        self.H=H
        self.history = []
        self.median = None

    def feed(self, value):
        self.history += [value]
        h = len(self.history)
        if h > self.H:
            self.history = self.history[h-self.H:h]

        self.median = np.median(self.history)
        return self.median

    def value(self):
        return self.median


class MeanSmoother:
    def __init__(self, alpha=0.1):
        self.alpha = alpha
        self.value = 0

    def feed(self, value):
        self.value = self.alpha*value + (1-self.alpha)*self.value

        return self.value

    def value(self):
        return self.value
