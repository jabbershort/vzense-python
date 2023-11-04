import time
import logging

import vzense.helper_functions
from vzense.api.Vzense_api_710 import VzenseTofCam
from vzense.api.Vzense_enums_710 import *

from vzense.properties import *

class VzenseCamera:
    def __init__(self,
                 uri=None,
                 resolution:CameraResolution=CameraResolution.Res_1920_1080):
                 
        if uri is None:
            available_devices = vzense.helper_functions.list_available_cameras()
            uri = available_devices[next(iter(available_devices))]['uri'].encode('utf-8')
            self.name = available_devices[next(iter(available_devices))]['alias']
           
        else:
            if type(uri) != bytes:
                uri = uri.encode('utf-8')
            available_devices = vzense.helper_functions.list_available_cameras()
            available_devices = [available_devices[dev]['uri'].encode('utf-8') for dev in available_devices]
            assert uri in available_devices, Exception(f'Device {uri.decode("utf-8")} not found')

        self.resolution = resolution

        self.__camera = VzenseTofCam()
        
        ret = self.__camera.Ps2_OpenDevice(uri)
        if ret != 0:
            logging.error(f'Failed to open device {self.name} with error: {ret}')
            exit()

    def connect(self):
        ret = self.__camera.Ps2_StartStream()
        if ret != 0:
            logging.error(f'Failed to start stream with error: {ret}')
            exit()

        ret = self.__camera.Ps2_SetRGBResolution(self.resolution)
        if ret != 0:
            logging.error(f'Failed to set resolution with error {ret}')

        ret = self.__camera.Ps2_SetDataMode(PsDataMode.PsDepthAndRGB_30)
        if ret != 0:
            logging.error(f'Failed to set data mode with error: {ret}')
            exit()

        ret = self.__camera.Ps2_SetMapperEnabledRGBToDepth(c_bool(True))
        if ret != 0:
            logging.error(f'Failed to set mapping mode with error: {ret}')
            exit()
        ret, depthrange = self.__camera.Ps2_GetDepthRange()
        ret, self.__depth_max,self.__value_min, self.__value_max = self.__camera.Ps2_GetMeasuringRange(PsDepthRange(depthrange.value))
        if ret != 0:
            logging.error(f'Failed to get depth parameter aith error code {ret}.')
            exit()

        ret, params = self.__camera.Ps2_GetCameraParameters(PsSensorType.PsRgbSensor)
        if  ret != 0:
           logging.error(f'Failed to get matrix info, with error {ret}')
        ret, frameready = self.__camera.Ps2_ReadNextFrame()
        ret, frame = self.__camera.Ps2_GetFrame(PsFrameType.PsRGBFrame)
        self.intrinsic = CamIntrinsics.from_vzense(frame.height,frame.width,params)
        self.connected = True
        logging.info(f'Succesfully connected to the camera {self.name}')

    def update(self):
        color_frame = None
        depth_frame = None
        if not self.connected: 
            logging.error('Camera not connected')
        ret, frameready = self.__camera.Ps2_ReadNextFrame()
        if  ret !=0:
            logging.error("Ps2_ReadNextFrame failed:",ret)      
        if  frameready.rgb:      
            ret,frame = self.__camera.Ps2_GetFrame(PsFrameType.PsRGBFrame)
            color_frame = numpy.ctypeslib.as_array(frame.pFrameData, (1, frame.width * frame.height * 3))
            color_frame.dtype = numpy.uint8
            color_frame.shape = (frame.height, frame.width,3)
        if  frameready.mappedDepth:      
            ret,frame = self.__camera.Ps2_GetFrame(PsFrameType.PsMappedDepthFrame)
            depth_frame = numpy.ctypeslib.as_array(frame.pFrameData, (1, frame.width * frame.height * 2))
            depth_frame.dtype = numpy.uint16
            depth_frame.shape = (frame.height, frame.width)
        
        logging.debug(f'Received a new frame of shape {color_frame.shape} color and {depth_frame.shape} depth.')
        self.__latest_frame = CameraFrame(time.time(),color_frame,depth_frame,self.intrinsic)
        return self.__latest_frame

    def __del__(self):
        ret = self.__camera.Ps2_StopStream()       
        if ret == 0:
            print(f"Stopped {self.name} successful")
        else:
            print('Ps2_StopStream failed: ' + str(ret))  


if __name__ == "__main__":
    logging.basicConfig(level=logging.DEBUG)
    cam = VzenseCamera()
    cam.connect()
    latest_frame = cam.update()
    vzense.helper_functions.view_frame(latest_frame)