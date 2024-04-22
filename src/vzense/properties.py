from enum import IntEnum
from dataclasses import dataclass

import cv2
import numpy as np
import open3d as o3d

class CameraResolution(IntEnum):
    Res_1920_1080 = 0
    Res_1280_720 = 1
    Res_640_480 = 2
    Res_640_320 = 3

@dataclass
class CamIntrinsics:
    h: int
    w: int
    fx: float
    fy: float
    cx: float
    cy: float

    @staticmethod
    def from_vzense(height,width,vzense_params):
        return CamIntrinsics(height,width,vzense_params.fx,vzense_params.fy,vzense_params.cx,vzense_params.cy)

    @property
    def np_matrix(self):
        return np.array([[self.fx,0,self.cx],[0,self.fy,self.cy],[0,0,1]])

    def as_open3d(self):
        return o3d.camera.PinholeCameraIntrinsic(self.w,self.h,self.np_matrix)
    
@dataclass
class CameraFrame:
    timestamp: float
    color_frame: np.ndarray
    depth_frame: np.ndarray
    intrinsic: CamIntrinsics

    @property
    def rgbd(self):
        rgbd = o3d.geometry.RGBDImage.create_from_color_and_depth(
            o3d.geometry.Image(cv2.cvtColor(self.color_frame,cv2.COLOR_BGR2RGB)), 
            o3d.geometry.Image(self.depth_frame), 
            depth_scale=1000.0, 
            depth_trunc=3.0, 
            convert_rgb_to_intensity=False)
        return rgbd

    @property
    def point_cloud(self) -> o3d.geometry.PointCloud:
        pcd = o3d.geometry.PointCloud.create_from_rgbd_image(
            self.rgbd, 
            self.intrinsic.as_open3d()
            )
        return pcd
    
    def mesh(self,size=0.002):
        tsdf = o3d.pipelines.integration.ScalableTSDFVolume(
            voxel_length = size,
            sdf_trunc=0.04,
            color_type=o3d.pipelines.integration.TSDFVolumeColorType.RGB8)
        tsdf.integrate(self.rgbd,self.intrinsic.as_open3d(),np.identity(4))
        mesh = tsdf.extract_triangle_mesh()
        mesh.compute_vertex_normals()
        return mesh