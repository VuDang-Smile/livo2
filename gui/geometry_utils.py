
import numpy as np
import cv2

def cartesian_to_spherical(points: np.ndarray) -> np.ndarray:
    """
    Converts the given 3D points from cartesian coordinates to spherical coordinates

    Args:
        points (np.ndarray): 3D points in cartesian coordinates

    Returns:
        sp_coords (np.ndarray): 3D points in spherical coordinates (rho, theta, phi)
    """

    assert points.shape[-1] == 3, "Input should have 3 (X, Y, Z) components"

    x, y, z = points[..., 0], points[..., 1], points[..., 2]

    # Distance of points from the origin
    rho = np.linalg.norm(points, axis=-1)

    # Normalize the points on the sphere of the above radius
    # to get the points on the unit sphere
    x /= rho
    y /= rho
    z /= rho

    # Elevation angle (aka latitude)
    phi = np.arcsin(y)

    # Azimuthal angle (aka longitude)
    theta = np.arctan2(x, z)

    # return np.stack([rho, theta, phi], axis=-1)
    return np.stack([rho, theta, phi]).T


def spherical_to_cartesian(sp_coords: np.ndarray) -> np.ndarray:
    """
    Converts the given 3D points from spherical coordinates to cartesian coordinates

    Args:
        sp_coords (np.ndarray): 3D points in spherical coordinates (rho, theta, phi)

    Returns:
        points (np.ndarray): 3D points in cartesian coordinates
    """

    assert sp_coords.shape[-1] == 3, "Input should have 3 (rho, phi, theta) components"

    rho = sp_coords[..., 0]
    theta = sp_coords[..., 1]
    phi = sp_coords[..., 2]

    x = rho * np.cos(phi) * np.sin(theta)
    y = rho * np.sin(phi)
    z = rho * np.cos(phi) * np.cos(theta)

    return np.stack([x, y, z], axis=-1)


def get_camera_matrix(FOV: float, width: int, height: int) -> np.ndarray:
    """

    Computes the intrinsic camera matrix from the given camera
    field of view (FOV) and image/window dimensions.

    Args:
        FOV (float): Field of view in radians
        width (int): Image/window width
        height (int): Image/window height

    Returns:
        K (np.ndarray): 3x3 matrix representing the intrinsic camera matrix
    """

    f = 0.5 * width / np.tan(0.5 * FOV)
    cx = (width) / 2.0
    cy = (height) / 2.0

    K = np.array([
            [f, 0, cx],
            [0, f, cy],
            [0, 0, 1]]).astype(np.float32)

    return K

def camera_to_world(points: np.ndarray, K: np.ndarray, R:np.ndarray) -> np.ndarray:
    """

    Transforms the given 3D points from camera coordinates to world coordinates

    Args:
        points (np.ndarray): 3D points in homogeneous camera coordinates
        K (np.ndarray): 3x3 matrix representing the intrinsic camera matrix
        R (np.ndarray): 3x3 matrix representing the extrinsic camera matrix (rotation)

    Returns:
        world_points (np.ndarray): 3D points in world coordinates

    """

    K_inv = np.linalg.inv(K)

    world_points = (points @ K_inv.T) @ R.T
    return world_points

def get_extrinsic_matrix(THETA:float, PHI:float):

    # Default
    elevation_vector = np.array([0.0, THETA, 0.0], np.float32)
    azimuth_vector = np.array([PHI, 0.0, 0.0], np.float32)

    # Use Rodrigues' formula to convert the
    # angle vector (simulatenous) to rotation matrix
    R1, _ = cv2.Rodrigues(elevation_vector)
    R2, _ = cv2.Rodrigues(np.dot(R1, azimuth_vector))

    R = R2 @ R1
    return R


def world_to_camera(points: np.ndarray, K: np.ndarray, R:np.ndarray) -> np.ndarray:
    """

    Transforms the given 3D points from world coordinates to camera coordinates

    Args:
        points (np.ndarray): 3D points in world coordinates
        K (np.ndarray): 3x3 matrix representing the intrinsic camera matrix
        R (np.ndarray): 3x3 matrix representing the extrinsic camera matrix (rotation + translation)

    Returns:
        camera_points (np.ndarray): 3D points in camera coordinates
    """

    # Add translation to the rotation matrix
    # As of now, the translation is zero.
    R = np.hstack([R, np.array([[0,0,0]], np.float32).T])

    camera_points = (points @ R.T) @ K.T
    return camera_points

def spherical2equirect(sp_coords: np.ndarray, width: int, height: int)-> np.ndarray:
    rho, theta, phi = sp_coords[..., 0], sp_coords[..., 1], sp_coords[..., 2]
    x = (theta / (2 * np.pi) + 0.5) * width
    y = (phi / np.pi + 0.5) * height
    
    # Giới hạn tọa độ để không bị tràn biên
    x = np.clip(x, 0, width - 1)
    y = np.clip(y, 0, height - 1)

    return np.stack([x, y], dtype=np.float32).transpose(2, 1, 0)