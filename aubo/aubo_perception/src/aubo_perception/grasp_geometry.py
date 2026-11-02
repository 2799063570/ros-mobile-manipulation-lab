"""检测器共用的几何计算；长度为米，角度为基座 XY 平面内的弧度。"""
import math
import numpy as np


def validate_modes(task_mode, height_mode, camera_mount, selected_class):
    if task_mode not in ("sorting", "servo", "both"):
        raise ValueError("task_mode must be sorting, servo or both")
    if height_mode not in ("table", "depth"):
        raise ValueError("height_mode must be table or depth")
    if camera_mount not in ("eye_in_hand", "eye_to_hand"):
        raise ValueError("camera_mount must be eye_in_hand or eye_to_hand")
    if task_mode in ("servo", "both") and not selected_class:
        raise ValueError("servo requires selected_class")


def center_height(surface_z, table_z, object_height, tolerance):
    """深度测到顶面，先校验顶面高度，再减去半个物高得到物体中心。"""
    values = (surface_z, table_z, object_height, tolerance)
    if not all(math.isfinite(v) for v in values) or object_height <= 0 or tolerance < 0:
        raise ValueError("invalid height parameters")
    if abs(surface_z - table_z - object_height) > tolerance:
        raise ValueError("depth surface disagrees with table + object height: "
                         "surface_z={:.5f}, expected_z={:.5f}, table_z={:.5f}, "
                         "object_height={:.5f}, error={:+.5f}, tolerance={:.5f} m".format(
                             surface_z, table_z + object_height, table_z, object_height,
                             surface_z - table_z - object_height, tolerance))
    return surface_z - object_height / 2


def object_height_from_depth(surface_z, table_z, minimum_height, maximum_height):
    """Measure visible surface height above the table, independently of class size."""
    if (not all(math.isfinite(v) for v in (surface_z, table_z, minimum_height, maximum_height))
            or not 0 < minimum_height <= maximum_height):
        raise ValueError("invalid measured object height/bounds")
    height = surface_z - table_z
    if height < minimum_height - 1e-9 or height > maximum_height + 1e-9:
        raise ValueError("measured object height outside bounds: surface_z={:.5f}, table_z={:.5f}, "
                         "height={:.5f}, allowed=[{:.5f}, {:.5f}] m".format(
                             surface_z, table_z, height, minimum_height, maximum_height))
    return min(max(height, minimum_height), maximum_height)


def ray_plane(u, v, K, rotation, translation, plane_z):
    if K[0] <= 0 or K[4] <= 0:
        raise ValueError("invalid camera intrinsics")
    ray = rotation.dot([(u-K[2])/K[0], (v-K[5])/K[4], 1.0])
    origin = np.asarray(translation, dtype=float)
    if abs(ray[2]) < 1e-8:
        raise ValueError("camera ray parallel to table")
    scale = (plane_z-origin[2])/ray[2]
    if scale <= 0:
        raise ValueError("table behind camera")
    point = origin + scale*ray
    if not np.isfinite(point).all():
        raise ValueError("invalid projection")
    return point


def rectangle_grasp(cx, cy, width, height, angle, K, rotation, translation, surface_z):
    """旋转框两条中线投影到水平顶面，取米制短边作为夹爪闭合方向。

    不能直接把图像角度当作机械臂 yaw；斜视相机需要先通过 TF 投影。
    """
    if not all(math.isfinite(v) for v in (cx, cy, width, height, angle)) or min(width, height) <= 0:
        raise ValueError("invalid rectangle")
    edges = []
    for length, theta in ((width, angle), (height, angle + math.pi/2)):
        du, dv = length*math.cos(theta)/2, length*math.sin(theta)/2
        a = ray_plane(cx-du, cy-dv, K, rotation, translation, surface_z)
        b = ray_plane(cx+du, cy+dv, K, rotation, translation, surface_z)
        delta = b-a
        edges.append((float(np.linalg.norm(delta[:2])), math.atan2(delta[1], delta[0])))
    narrow, yaw = min(edges, key=lambda e: e[0])
    return narrow, (yaw + math.pi/2) % math.pi - math.pi/2


def rectangle_footprint(cx, cy, width, height, angle, K, rotation, translation, surface_z):
    """Project all four OBB corners into the same world frame as the grasp pose."""
    if not all(math.isfinite(v) for v in (cx, cy, width, height, angle)) or min(width, height) <= 0:
        raise ValueError('invalid footprint rectangle')
    c, s = math.cos(angle), math.sin(angle)
    return [ray_plane(cx+c*dx-s*dy, cy+s*dx+c*dy, K, rotation, translation, surface_z)
            for dx, dy in ((-width/2, -height/2), (width/2, -height/2),
                           (width/2, height/2), (-width/2, height/2))]
