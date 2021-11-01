#!/usr/bin/env python3
############## Task1.1 - ArUco Detection ##############
import cv2
import cv2.aruco as aruco
import sys
import math
import time


def detect_ArUco(img):
    # function to detect ArUco markers in the image using ArUco library
    
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    aruco_dict = aruco.Dictionary_get(aruco.DICT_5X5_250)
    parameters = aruco.DetectorParameters_create()
    corners, ids, _ = aruco.detectMarkers(gray, aruco_dict, parameters=parameters)

    # argument: img is the test image
    # return: dictionary named Detected_ArUco_markers of the format {ArUco_id_no : corners}, where ArUco_id_no indicates ArUco id and corners indicates the four corner position of the aruco(numpy array)
    # for instance, if there is an ArUco(0) in some orientation then, ArUco_list can be like
    # {0: array([[315, 163],
    #							[319, 263],
    #							[219, 267],
    #							[215,167]], dtype=float32)}
    Detected_ArUco_markers = {}
    Detected_ArUco_markers = dict(zip(ids[:, 0], corners))
## enter your code here ##
    return Detected_ArUco_markers


def Calculate_orientation_in_degree(Detected_ArUco_markers):
    # function to calculate orientation of ArUco with respective to the scale mentioned in problem statement
    # argument: Detected_ArUco_markers  is the dictionary returned by the function detect_ArUco(img)
    # return : Dictionary named ArUco_marker_angles in which keys are ArUco ids and the values are angles (angles have to be calculated as mentioned in the problem statement)
    # for instance, if there are two ArUco markers with id 1 and 2 with angles 120 and 164 respectively, the
    # function should return: {1: 120 , 2: 164}

    ArUco_marker_angles = {}
    ## enter your code here ##
    for _id in Detected_ArUco_markers.keys():
        np_arr = Detected_ArUco_markers[_id]
        tl = np_arr[0, 0]
        tr = np_arr[0, 1]
        br = np_arr[0, 2]
        bl = np_arr[0, 3]
        y = [(tl[0]+br[0])/2, (tl[1]+br[1])/2]
        diffs = [(tl[0]-bl[0]), (tl[1]-bl[1])]
        #print(diffs)
        degree = math.atan2(diffs[0], diffs[1])*180/math.pi
        final_degree = degree+float(270)
        if (final_degree > 360.0):
            final_degree -= 360.0
        ArUco_marker_angles[_id] = final_degree

    # returning the angles of the ArUco markers in degrees as a dictionary
    return ArUco_marker_angles


def mark_ArUco(img, Detected_ArUco_markers, ArUco_marker_angles):
    # function to mark ArUco in the test image as per the instructions given in problem statement
    # arguments: img is the test image
    # Detected_ArUco_markers is the dictionary returned by function detect_ArUco(img)
    # ArUco_marker_angles is the return value of Calculate_orientation_in_degree(Detected_ArUco_markers)
    # return: image namely img after marking the aruco as per the instruction given in problem statement

    ## enter your code here ##
    
   
    for _id in Detected_ArUco_markers.keys():
        np_arr = Detected_ArUco_markers[_id]
        tl = np_arr[0, 0]
        tr = np_arr[0, 1]
        br = np_arr[0, 2]
        bl = np_arr[0, 3]

        ctr = [(tl[0]+br[0])/2, (tl[1]+br[1])/2]
        mid = [(tr[0]+tl[0])/2, (tr[1]+tl[1])/2]
        # dots
        cv2.circle(img, (int(tl[0]), int(tl[1])), 5, (125, 125, 125), -1)
        cv2.circle(img, (int(tr[0]), int(tr[1])), 5, (0, 255, 0), -1)
        cv2.circle(img, (int(br[0]), int(br[1])), 5, (180, 105, 255), -1)
        cv2.circle(img, (int(bl[0]), int(bl[1])), 5, (255, 255, 255), -1)
        cv2.circle(img, (int(ctr[0]), int(ctr[1])), 5, (0, 0, 255), -1)
        cv2.line(img, (int(ctr[0]), int(ctr[1])),
                 (int(mid[0]), int(mid[1])), (255, 0, 0), 2)
        cv2.putText(img,str(_id) , ((int(ctr[0])+8), int(ctr[1])), fontFace=cv2.FONT_HERSHEY_COMPLEX,thickness=2 ,fontScale=0.75, color=(0,0,255))
        samp = str(round(ArUco_marker_angles[_id]))
        cv2.putText(img,samp , ((int(ctr[0])-90), int(ctr[1])), fontFace=cv2.FONT_HERSHEY_COMPLEX,thickness=2, fontScale=0.75, color=(0, 255,0)) 
        cv2.putText(img, "TEAM ID ss_1403", (0, 15),fontFace=cv2.FONT_HERSHEY_COMPLEX,thickness=2, fontScale=0.75, color=(105, 53,0))

    #cv2.imshow('frame', img)
    #cv2.waitKey(0)
    return img


