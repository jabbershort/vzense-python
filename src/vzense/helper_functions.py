from vzense.api.Vzense_api_710 import VzenseTofCam, PsDeviceInfo
from vzense.properties import *

import open3d as o3d

def test():
    print('hello')

def list_available_cameras():
    camera = VzenseTofCam()
    camera_count = camera.Ps2_GetDeviceCount()
    cameras = {}
    device_info=PsDeviceInfo()
    ret,device_infolist=camera.Ps2_GetDeviceListInfo(camera_count)
    if ret==0:
        device_info = device_infolist[0]
        for info in device_infolist: 
            cameras[info.alias.decode()] = {
                'type':info.devicetype,
                'uri':info.uri.decode(),
                'firmware':info.fw.decode(),
                'alias':info.alias.decode(),
                'status':info.status,
                'ip':info.ip.decode()
            }    
    return cameras

def view_frame(frame: CameraFrame):
    o3d.visualization.draw_geometries([frame.point_cloud])