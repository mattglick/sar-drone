import numpy as np

def pixel_to_latlon(
    bbox_cx, bbox_cy,
    img_w, img_h,
    drone_lat, drone_lon,
    drone_alt,
    roll, pitch, yaw,
    fov_h_deg=82.0
):
    fx = (img_w / 2) / np.tan(np.radians(fov_h_deg / 2))
    fy = fx
    cx, cy = img_w / 2, img_h / 2

    ray_cam = np.array([
        (bbox_cx-cx)/ fx,
        (bbox_cy-cy) / fy,
        -1.0
    ])

    ray_cam /= np.linalg.norm(ray_cam)

    R = rotation_matrix(roll, pitch, yaw)
    ray_world = R @ ray_cam

    print(f"DEBUG ray_world: {ray_world}")  # let's see what direction the ray points

    if ray_world[2] >= 0:
        return None, None
    t = -drone_alt / ray_world[2]
    north_offset = ray_world[0] * t
    east_offset  = ray_world[1] * t

    d_lat = north_offset / 111320.0
    d_lon = east_offset  / (111320.0 * np.cos(np.radians(drone_lat)))
    return drone_lat + d_lat, drone_lon + d_lon


def rotation_matrix(roll, pitch, yaw):
    cr, sr = np.cos(roll),  np.sin(roll)
    cp, sp = np.cos(pitch), np.sin(pitch)
    cy, sy = np.cos(yaw),   np.sin(yaw)
    Rz = np.array([[cy,-sy,0],[sy,cy,0],[0,0,1]])
    Ry = np.array([[cp,0,sp],[0,1,0],[-sp,0,cp]])
    Rx = np.array([[1,0,0],[0,cr,-sr],[0,sr,cr]])
    return Rz @ Ry @ Rx


if __name__ == "__main__":
    result = pixel_to_latlon(
        bbox_cx=640, bbox_cy=360,
        img_w=1280, img_h=720,
        drone_lat=47.6062, drone_lon=-122.3321,
        drone_alt=50.0,
        roll=0, pitch=0.5, yaw=0
    )
    lat, lon = result
    if lat is not None:
        print(f"Estimated ground position: {lat:.6f}, {lon:.6f}")
    else:
        print("No ground intersection found")