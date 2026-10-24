#!/usr/bin/env python3
"""Experimental depth-to-color registration for passive semantic observations.

This does not modify the camera driver or the depth image used by navigation.
CameraInfo K/D and the depth-optical -> color-optical transform are required.
No hole filling is performed: unknown color pixels remain zero depth.
"""

import numpy as np


def quaternion_rotation(q):
    x, y, z, w = (float(q.x), float(q.y), float(q.z), float(q.w))
    norm = x*x + y*y + z*z + w*w
    if not np.isfinite(norm) or norm < 1e-12:
        raise ValueError("Invalid depth-to-color rotation")
    x, y, z, w = np.asarray([x, y, z, w]) / np.sqrt(norm)
    return np.array([
        [1-2*(y*y+z*z), 2*(x*y-z*w), 2*(x*z+y*w)],
        [2*(x*y+z*w), 1-2*(x*x+z*z), 2*(y*z-x*w)],
        [2*(x*z-y*w), 2*(y*z+x*w), 1-2*(x*x+y*y)],
    ], dtype=np.float64)


def register_depth(depth_mm, depth_info, color_info, depth_to_color):
    """Return sparse uint16 depth in raw color pixels, or raise on bad geometry."""
    if depth_mm.dtype != np.uint16 or depth_mm.shape != (depth_info.height, depth_info.width):
        raise ValueError("Depth image and CameraInfo mismatch")
    if depth_info.distortion_model != "plumb_bob" or color_info.distortion_model != "plumb_bob":
        raise ValueError("Unsupported camera distortion model")
    if any(abs(v) > 1e-9 for v in depth_info.d):
        raise ValueError("Distorted depth rays require a separate undistortion step")
    if color_info.width < 1 or color_info.height < 1:
        raise ValueError("Invalid color image size")
    fy_d, fx_d = float(depth_info.k[4]), float(depth_info.k[0])
    if min(fx_d, fy_d, color_info.k[0], color_info.k[4]) <= 0:
        raise ValueError("Invalid camera focal length")
    yy, xx = np.nonzero((depth_mm >= 300) & (depth_mm <= 4000))
    output = np.zeros((color_info.height, color_info.width), dtype=np.uint16)
    if not len(xx):
        return output
    z = depth_mm[yy, xx].astype(np.float64) / 1000.0
    xyz = np.stack(((xx - depth_info.k[2]) * z / fx_d,
                    (yy - depth_info.k[5]) * z / fy_d, z), axis=0)
    t = depth_to_color.translation
    color_xyz = quaternion_rotation(depth_to_color.rotation) @ xyz
    color_xyz += np.array([t.x, t.y, t.z], dtype=np.float64)[:, None]
    positive = color_xyz[2] > 0.05
    x = color_xyz[0, positive] / color_xyz[2, positive]
    y = color_xyz[1, positive] / color_xyz[2, positive]
    d = np.pad(np.asarray(color_info.d, dtype=np.float64), (0, max(0, 8-len(color_info.d))))
    r2 = x*x + y*y
    radial = (1+d[0]*r2+d[1]*r2*r2+d[4]*r2*r2*r2) / (1+d[5]*r2+d[6]*r2*r2+d[7]*r2*r2*r2)
    xd = x*radial + 2*d[2]*x*y + d[3]*(r2+2*x*x)
    yd = y*radial + d[2]*(r2+2*y*y) + 2*d[3]*x*y
    u = np.rint(color_info.k[0]*xd + color_info.k[2]).astype(np.int32)
    v = np.rint(color_info.k[4]*yd + color_info.k[5]).astype(np.int32)
    valid = ((u >= 0) & (u < color_info.width) & (v >= 0) &
             (v < color_info.height) & np.isfinite(xd) & np.isfinite(yd))
    flat = np.full(output.size, np.iinfo(np.uint16).max, dtype=np.uint16)
    indices = v[valid] * color_info.width + u[valid]
    measured_mm = np.rint(color_xyz[2, positive][valid] * 1000).astype(np.uint16)
    np.minimum.at(flat, indices, measured_mm)
    flat[flat == np.iinfo(np.uint16).max] = 0
    return flat.reshape(output.shape)
