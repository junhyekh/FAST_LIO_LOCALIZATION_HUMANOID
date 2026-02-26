import open3d as o3d
import tyro

from pathlib import Path
import viser
import dataclasses
import numpy as np
from scipy.spatial.transform import Rotation as R

@dataclasses.dataclass
class Args:
    pcd_src: str
    ## plane segmentation parameters
    distance_threshold: float = 0.05
    ransac_n: int = 3
    num_iterations: int = 1000
    probability: float = 0.999999

    output_path: str = "" # path to save the aligned point cloud (e.g. data/map_aligned.ply). Empty = no save.

    test: bool = False

    align: bool = True # align the point cloud plane to point z direction (0, 0, 1)
    flip_z: bool = False # flip z-axis (180° rotation around X-axis) after ground alignment

def to_numpy(pcd: o3d.geometry.PointCloud):
    points = np.asarray(pcd.points)
    colors = np.asarray(pcd.colors)
    if colors is None or colors.size == 0:
        colors = np.ones_like(points) * 0.5
    return points, colors

def main(args: Args):
    pcd_src = o3d.io.read_point_cloud(args.pcd_src)
    print(type(pcd_src))
    if args.test:
        rand_rot = R.random().as_matrix()
        transform = np.eye(4)
        transform[:3, :3] = rand_rot
        pcd_src = pcd_src.transform(transform)
    pcd_src_coarse = pcd_src.voxel_down_sample(voxel_size=0.1)
    pcd_src.estimate_normals(search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=0.1, max_nn=30), fast_normal_computation=False)
    pcd_src.orient_normals_consistent_tangent_plane(k=30)
    plane_model, inliers = pcd_src.segment_plane(distance_threshold=args.distance_threshold,
                                                ransac_n=args.ransac_n,
                                                num_iterations=args.num_iterations,
                                                probability=args.probability)       
    print(plane_model)
    pcd_src_ground = pcd_src.select_by_index(inliers)
    pcd_src_ground.paint_uniform_color([1, 0, 0])
    pcd_coarse = o3d.geometry.PointCloud(pcd_src)
    pcd_coarse = pcd_coarse.voxel_down_sample(voxel_size=0.1)

    # get the transform matrix to make plane as a ground plane (normal to z direction point with z==0 is on the plane)
    a, b, c, d = plane_model
    n = np.array([a, b, c], dtype=float)
    n_norm = np.linalg.norm(n)
    if n_norm < 1e-9:
        raise SystemExit("Degenerate plane normal from RANSAC")
    n = n / n_norm
    # Closest point on plane to origin
    p0 = -d * n
    # Rodrigues rotation to align n -> +Z
    z = np.array([0.0, 0.0, 1.0])
    v = np.cross(n, z)
    s = np.linalg.norm(v)
    ccos = float(np.dot(n, z))
    if s < 1e-9:
        rot = np.eye(3)
    else:
        vx = np.array([[0.0, -v[2], v[1]], [v[2], 0.0, -v[0]], [-v[1], v[0], 0.0]])
        rot = np.eye(3) + vx + vx @ vx * ((1.0 - ccos) / (s ** 2))
    p0_rot = rot @ p0
    tz = -p0_rot[2]
    T = np.eye(4)
    T[:3, :3] = rot
    T[:3, 3] = np.array([0.0, 0.0, tz])
    print("Align-to-ground transform (4x4):\n", T)

    if args.align:
        pcd_algined = pcd_src.transform(T)

        # Track the total transform applied to the point cloud
        T_total = T.copy()

        # Optional: flip Z-axis (180° rotation around X-axis)
        if args.flip_z:
            T_flip = np.eye(4)
            T_flip[1, 1] = -1.0  # y -> -y
            T_flip[2, 2] = -1.0  # z -> -z
            pcd_algined = pcd_algined.transform(T_flip)
            T_total = T_flip @ T_total
            print("Applied Z-axis flip (180° rotation around X-axis)")

        # Transform the original start point (0,0,0) with the same transform
        origin_old = np.array([0.0, 0.0, 0.0, 1.0])
        origin_new = T_total @ origin_old
        print(f"\n=== Start Point (FAST-LIO origin) ===")
        print(f"  Before: [0.0, 0.0, 0.0]")
        print(f"  After:  [{origin_new[0]:.4f}, {origin_new[1]:.4f}, {origin_new[2]:.4f}]")
        print(f"  Total transform (4x4):\n{T_total}\n")

        pcd_algined_ground = pcd_algined.select_by_index(inliers)
        pcd_algined_ground.paint_uniform_color([1, 0, 0])
        pcd_algined_coarse = o3d.geometry.PointCloud(pcd_algined)
        pcd_algined_coarse = pcd_algined_coarse.voxel_down_sample(voxel_size=0.1)

        # Save aligned point cloud if output_path is specified
        if args.output_path:
            output = Path(args.output_path)
            output.parent.mkdir(parents=True, exist_ok=True)
            o3d.io.write_point_cloud(str(output), pcd_algined)
            print(f"Saved aligned point cloud to: {output}")
            print(f"NOTE: The FAST-LIO start point is now at "
                  f"[{origin_new[0]:.4f}, {origin_new[1]:.4f}, {origin_new[2]:.4f}] "
                  f"in the new map coordinates.")

    plane_model, inliers = pcd_algined.segment_plane(distance_threshold=args.distance_threshold,
                                                ransac_n=args.ransac_n,
                                                num_iterations=args.num_iterations,
                                                probability=args.probability)       
    print("after align plane_model", plane_model)


    server = viser.ViserServer()
    points, colors = to_numpy(pcd_src_coarse)
    server.add_point_cloud("pcd_coarse", points=points, colors=colors, point_size=0.005)
    points, colors = to_numpy(pcd_algined_ground)
    server.add_point_cloud("pcd_algined_ground", points=points, colors=colors[0], point_size=0.02)
    points, colors = to_numpy(pcd_algined_coarse)
    server.add_point_cloud("pcd_algined_coarse", points=points, colors=colors, point_size=0.005)
    
    while True:
        pass




if __name__ == "__main__":
    args = tyro.cli(Args)
    main(args)