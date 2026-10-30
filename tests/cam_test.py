import sys
import numpy as np
from pypylon import pylon
import time
import cv2 as cv

'''serial No's when standing in front of the rig:
    42023496 - left
    42023491 - right
    42023506 - top
    Basler Docs recommend adressing with these to avoid problems with order or number of connected components
    pypylon doc: https://docs.baslerweb.com/pypylon-introduction
'''   
if __name__ == "__main__":
    print("Camera Test Script")

    srl_no_left = 42023496
    srl_no_right = 42023491
    srl_no_top = 42023506
    time_out = 2000
    r_connected, l_connected, top_connected = False, False, False

    #first test from tutorial: find any cam and verify that a picture was taken
    with pylon.InstantCamera(pylon.FirstFound) as camera:
        camera.StartGrabbingMax(1)

        with camera.RetrieveResult(time_out) as grab_result:
            if grab_result.GrabSucceeded():
                image = grab_result.Array
                print(f"test img exists, shape:{image.shape}")
            else:
                print("grabbing test img of any device failed: " + grab_result.ErrorDescription)

    #list connected devices
    factory = pylon.TlFactory.GetInstance()
    device_info_list = factory.EnumerateDevices()

    for device_info in device_info_list:
        print(device_info.GetFriendlyName())
        print(device_info.GetSerialNumber())

    #recommended method to select a camera (here: map all 3 cams to variables)
    for device_info in device_info_list:
        if int(device_info.GetSerialNumber()) == srl_no_top:
            top_cam = pylon.InstantCamera(factory.CreateDevice(device_info))
            top_cam.Open()
            top_connected = True
            print("top cam initialized")
        if int(device_info.GetSerialNumber()) == srl_no_right:
            right_cam = pylon.InstantCamera(factory.CreateDevice(device_info))
            right_cam.Open()
            r_connected = True
            print("right cam initialized")
        if int(device_info.GetSerialNumber()) == srl_no_left:
            left_cam = pylon.InstantCamera(factory.CreateDevice(device_info))
            left_cam.Open()
            l_connected = True
            print("left cam initialized")

    #we need at least right and top now:
    if not (top_connected and r_connected):
        print("connect at least top and right cam")
        sys.exit()


    #demo: timed img aquisition with top cam:
    #enable trigger mode
    top_cam.TriggerSelector.Value = "FrameStart"
    top_cam.TriggerMode.Value = "On"
    top_cam.TriggerSource.Value = "Software"

    top_cam.StartGrabbing(pylon.GrabStrategy_OneByOne)
    #three times
    cnt = 0
    while cnt < 3:
        top_cam.ExecuteSoftwareTrigger()

        with top_cam.RetrieveResult(time_out) as grab_result:
            if grab_result.GrabSucceeded():
                img = grab_result.Array
                print(f"image taken, shape: {img.shape}")
            else:
                print("grab failed: ", grab_result.ErrorDescription)
        time.sleep(1.0)
        cnt += 1

    #we can use captured imgs directly with opencv!
    #demo: live img show
    right_cam.StartGrabbing()
    while right_cam.IsGrabbing():
        with right_cam.RetrieveResult(time_out) as grab_result:
            
            if grab_result.GrabSucceeded():
                img = grab_result.Array
                cv.imshow('Live', img)
                if cv.waitKey(1) == 27: #ESC to exit
                    break

    cv.destroyAllWindows()

    
    

    


