"""
HK-MEMS dataset, calculate the error of 6 slam methods and draw a figure,
include 2D RMSE, start to end error
"""

import os
import glob
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle
from scipy.io import loadmat
import re


# Helper functions (placeholders - need implementation)
def preprocess_position(slam_2d_positions, slam_3d_positions, seq):
    """Preprocess position data"""
    return slam_2d_positions, slam_3d_positions


def compute_start_to_end_error(slam_3d_positions, visualize, threshold):
    """Compute start to end error"""
    if len(slam_3d_positions) <= 1:
        return 0, 0
    start = slam_3d_positions[0]
    end = slam_3d_positions[-1]
    error = np.linalg.norm(end - start)
    error_3d = error
    return error, error_3d

"""
def simple_2d_icp(source, target, max_iterations, tolerance):

    aligned_source = source.copy()
    aligned_target = target.copy()
    prev_rmse = float('inf')
    
    for iteration in range(max_iterations):
        # Find nearest neighbors
        from scipy.spatial.distance import cdist
        distances = cdist(aligned_source, aligned_target)
        nearest_indices = np.argmin(distances, axis=1)
        
        # Compute transformation
        R, t = align_point_clouds(aligned_source, aligned_target[nearest_indices, :])
        aligned_source = (R @ aligned_source.T + t.reshape(-1, 1)).T
        
        # Compute RMSE
        rmse = np.sqrt(np.mean(np.sum((aligned_source - aligned_target[nearest_indices, :]) ** 2, axis=1)))
        
        # Check convergence
        if abs(prev_rmse - rmse) < tolerance:
            break
        prev_rmse = rmse
    
    return R, t, aligned_source, aligned_target, rmse


def compute_rmse(source, target):

    min_len = min(len(source), len(target))
    diff = source[:min_len] - target[:min_len]
    rmse = np.sqrt(np.mean(np.sum(diff ** 2, axis=1)))
    trajectory_length = np.linalg.norm(target[-1] - target[0]) + 1e-6
    rmsre = rmse / trajectory_length
    return rmse, rmsre
"""
def simple_2d_icp(source, target, max_iterations, tolerance):
    # Initialize variables
    source_length = source.shape[0]
    target_length = target.shape[0]  # en MATLAB aparece size(target,1') -> typo

    # Downsampling
    if source_length > target_length:
        step = source_length / target_length
        sample_index = np.ceil(
            np.arange(0, source_length, step)
        ).astype(int)
        # sample_index = np.clip(sample_index, 0, source_length - 1)

        source_sampled = source[sample_index, :]
        target_sampled = target
    else:
        step = target_length / source_length
        sample_index = np.ceil(
            np.arange(0, target_length, step)
        ).astype(int)
        # sample_index = np.clip(sample_index, 0, target_length - 1)

        source_sampled = source
        target_sampled = target[sample_index, :]

    num_points_source = source_sampled.shape[0]
    num_points_target = target_sampled.shape[0]

    aligned_source = source_sampled.copy()
    match_in_target = np.zeros((num_points_source, 2))

    R = np.eye(2)
    t = np.zeros((2, 1))

    # Transformación homogénea acumulada
    T = np.array([
        [1, 0, 0],
        [0, 1, 0],
        [0, 0, 0]   # tal como en MATLAB (aunque no es una homogénea estándar)
    ])

    RMSE = []

    for _ in range(max_iterations):
        # Nearest neighbor search (brute force)
        for i_source in range(num_points_source):
            min_dist = np.inf
            nearest_point_idx = 0

            for j_target in range(num_points_target):
                dist = np.linalg.norm(
                    aligned_source[i_source, :] - target_sampled[j_target, :]
                )
                if dist < min_dist:
                    min_dist = dist
                    nearest_point_idx = j_target

            match_in_target[i_source, :] = target_sampled[nearest_point_idx, :]

        # Centroids
        centroid_source = np.mean(aligned_source, axis=0)
        centroid_target = np.mean(match_in_target, axis=0)

        # Centered points
        centered_source = aligned_source - centroid_source
        centered_target = match_in_target - centroid_target

        # Covariance matrix
        H = centered_source.T @ centered_target

        # SVD
        U, _, Vt = np.linalg.svd(H)
        V = Vt.T

        # Rotation
        R_i = V @ U.T

        # Avoid reflection
        if np.linalg.det(R_i) < 0:
            V[:, -1] *= -1
            R_i = V @ U.T

        # Translation
        t_i = centroid_target.reshape(2, 1) - R_i @ centroid_source.reshape(2, 1)

        # OJO: en el MATLAB se fuerza a cero
        t_i = np.zeros((2, 1))

        # Update aligned source
        aligned_source = (R_i @ aligned_source.T + t_i).T

        # Update transform
        T_i = np.block([
            [R_i, t_i],
            [np.zeros((1, 3))]
        ])
        T = T_i @ T

        # RMSE
        differences = aligned_source - match_in_target
        squared_errors = np.sum(differences ** 2, axis=1)
        rmse_iter = np.sqrt(np.mean(squared_errors))
        RMSE.append(rmse_iter)

        if rmse_iter < tolerance:
            break

    # Final R and t (tal como MATLAB)
    R = T[0:2, 0:2]
    t = T[2, 0:2]    # esto también es raro, pero replica el original

    aligned_target = target
    aligned_source = (R @ source.T + t.reshape(2, 1)).T

    return R, t, aligned_source, aligned_target, np.array(RMSE)

def align_point_clouds(source, target):

    # Calculate centroids
    source_center = np.mean(source, axis=0)
    target_center = np.mean(target, axis=0)
    
    # Initialize centered point clouds
    centered_source = source.copy()
    centered_target = target.copy()
    
    # Determine point cloud lengths
    source_length = centered_source.shape[0]
    target_length = centered_target.shape[0]
    
    # Resample to match lengths
    if source_length > target_length:
        step = source_length / target_length
        sample_index = np.ceil(np.arange(0, source_length, step)).astype(int)
        # sample_index = np.minimum(sample_index, source_length - 1)
        centered_source_sampled = centered_source[sample_index, :]
        centered_target_sampled = centered_target
    else:
        step = target_length / source_length
        sample_index = np.ceil(np.arange(0, target_length , step)).astype(int)
        # sample_index = np.minimum(sample_index, target_length - 1)
        centered_source_sampled = centered_source
        centered_target_sampled = centered_target[sample_index, :]
    
    # Determine minimum length and truncate
    min_length = min(centered_source_sampled.shape[0], centered_target_sampled.shape[0])
    centered_source_sampled = centered_source_sampled[:min_length, :]
    centered_target_sampled = centered_target_sampled[:min_length, :]
    
    # Compute rotation matrix
    # Filter out NaN and Inf values from source 
    # SUS
    valid_source = ~(np.isnan(centered_source_sampled).any(axis=1) | np.isinf(centered_source_sampled).any(axis=1))
    centered_source_sampled_filtered = centered_source_sampled[valid_source, :]
    
    # Filter out NaN and Inf values from target
    valid_target = ~(np.isnan(centered_target_sampled).any(axis=1) | np.isinf(centered_target_sampled).any(axis=1))
    centered_target_sampled_filtered = centered_target_sampled[valid_target, :]
    
    # Perform SVD
    U, _, V = np.linalg.svd(centered_source_sampled_filtered.T @ centered_target_sampled_filtered)
    R_init = V.T @ U.T
    
    # Correct for reflection (ensure proper rotation, not reflection)
    det_R = np.linalg.det(R_init)
    if det_R < 0:
        V[-1, :] = -V[-1, :]
        R_init = V.T @ U.T
    
    # Compute translation
    t_init = target_center - R_init @ source_center
    R = R_init
    t = t_init
    return R, t

def compute_rmse(source, target):
    num_points_source = source.shape[0]
    num_points_target = target.shape[0]

    match_in_target = np.zeros((num_points_source, 2))

    # Buscar el punto más cercano en target para cada punto en source
    for i_source in range(num_points_source):
        min_dist = np.inf
        nearest_point_idx = 0

        for j_target in range(num_points_target):
            dist = np.linalg.norm(source[i_source, :] - target[j_target, :])
            if dist < min_dist:
                min_dist = dist
                nearest_point_idx = j_target

        match_in_target[i_source, :] = target[nearest_point_idx, :]

    # Errores
    differences = source - match_in_target
    squared_errors = np.sum(differences ** 2, axis=1)

    # Calcular longitud de trayectoria
    trajectory_lengths = np.zeros(match_in_target.shape[0])
    for i in range(1, match_in_target.shape[0]):
        distance = np.sqrt(
            np.sum((match_in_target[i, :] - match_in_target[i - 1, :]) ** 2)
        )
        trajectory_lengths[i] = trajectory_lengths[i - 1] + distance

    # Diferencias relativas
    sqrt_squared_errors = np.sqrt(squared_errors)
    valid_indices = trajectory_lengths != 0
    relative_differences = np.zeros_like(squared_errors)

    for i in range(squared_errors.shape[0]):
        if valid_indices[i]:
            relative_differences[i] = (
                sqrt_squared_errors[i] / trajectory_lengths[i]
            )

    rmse = np.sqrt(np.mean(squared_errors))
    rmsre = np.mean(relative_differences)

    return rmse, rmsre

def align_and_compute_2d_rmse(slam_2d_positions, grt_2d_positions, move_slam_path):
    """
    Align SLAM trajectory to ground truth and compute RMSE
    
    Args:
        slam_2d_positions: SLAM 2D positions (N x 2)
        grt_2d_positions: Ground truth 2D positions (M x 2)
        move_slam_path: If 1, move SLAM path; if 0, move ground truth
    
    Returns:
        aligned_slam_2d_positions_icp: Aligned SLAM positions
        aligned_grt_2d_positions_icp: Aligned ground truth positions
        rmse: Root mean square error
        rmsre: Relative root mean square error
    """
    # print(grt_2d_positions)
    # Preprocess grt_2d_positions
    grt_2d_positions = np.array([[g[0]- grt_2d_positions[0][0], g[1] - grt_2d_positions[0][1]] for g in grt_2d_positions])
    
    # Get partial trajectory points
    align_partial = 1
    slam_num_points = int(np.round(align_partial * slam_2d_positions.shape[0]))
    grt_num_points = int(np.round(align_partial * grt_2d_positions.shape[0]))
    
    # Initial alignment
    aligned_slam_2d_positions = slam_2d_positions.copy()
    aligned_grt_2d_positions = grt_2d_positions.copy()
    
    if move_slam_path == 1:
        R_init, t_init = align_point_clouds(
            slam_2d_positions[:slam_num_points, :],
            grt_2d_positions[:grt_num_points, :]
        )
        aligned_slam_2d_positions = (R_init @ aligned_slam_2d_positions.T + t_init.reshape(-1, 1)).T
    else:
        R_init, t_init = align_point_clouds(
            grt_2d_positions[:grt_num_points, :],
            slam_2d_positions[:slam_num_points, :]
        )
        aligned_grt_2d_positions = (R_init @ aligned_grt_2d_positions.T + t_init.reshape(-1, 1)).T
    
    # ICP iterative optimization
    max_iterations = 10
    tolerance = 1e-6
    
    if move_slam_path == 1:
        R, t, aligned_slam_2d_positions_icp, aligned_grt_2d_positions_icp, RMSE = \
            simple_2d_icp(aligned_slam_2d_positions, aligned_grt_2d_positions, max_iterations, tolerance)
    else:
        R, t, aligned_grt_2d_positions_icp, aligned_slam_2d_positions_icp, RMSE = \
            simple_2d_icp(aligned_grt_2d_positions, aligned_slam_2d_positions, max_iterations, tolerance)
    
    rmse_source, rmsre_source = compute_rmse(aligned_slam_2d_positions_icp, aligned_grt_2d_positions_icp)
    rmse_target, rmsre_target = compute_rmse(aligned_grt_2d_positions_icp, aligned_slam_2d_positions_icp)
    
    rmse = (rmse_source + rmse_target) / 2
    rmsre = (rmsre_source + rmsre_target) / 2
    
    return aligned_slam_2d_positions_icp, aligned_grt_2d_positions_icp, rmse, rmsre

# def align_and_compute_2d_rmse(slam_2d_positions, grt_2d_positions, move_slam_path):
#     """Align SLAM trajectory to ground truth and compute RMSE"""
#     if move_slam_path:
#         # Simple alignment: translate slam to match grt start
#         # print(slam_2d_positions, grt_2d_positions)
#         slam_translated =  [x - slam_2d_positions[0] + grt_2d_positions for x in slam_2d_positions] 
#     else:
#         slam_translated = slam_2d_positions
    
#     # Compute RMSE
#     min_len = min(len(slam_translated), len(grt_2d_positions))
#     diff = [slam_translated[i] - grt_2d_positions[i] for i in range(min_len)]
#     rmse = np.sqrt(np.mean(np.sum([d**2 for d in diff], axis=1)))
#     rmsre = rmse / (np.linalg.norm(grt_2d_positions[-1] - grt_2d_positions[0]) + 1e-6)
    
#     return slam_translated, grt_2d_positions, rmse, rmsre


def tight_subplot(rows, cols, gap, margin_h, margin_v):
    """Create tight subplot layout similar to MATLAB"""
    fig = plt.gcf()
    axes_list = []
    for i in range(1, rows * cols + 1):
        row = (i - 1) // cols
        col = (i - 1) % cols
        left = margin_v[0] + col * (1 - margin_v[0] - margin_v[1]) / cols + col * gap[0] / cols
        bottom = 1 - margin_h[0] - (row + 1) * (1 - margin_h[0] - margin_h[1]) / rows + row * gap[1] / rows
        width = (1 - margin_v[0] - margin_v[1]) / cols - gap[0] / cols
        height = (1 - margin_h[0] - margin_h[1]) / rows - gap[1] / rows
        ax = fig.add_axes([left, bottom, width, height])
        axes_list.append(ax)
    return axes_list


# Read ground truth data
grt_folder_path = 'grt/grt/bus_2d_path/'
grt_files = glob.glob(os.path.join(grt_folder_path, '*.txt'))
num_grt = len(grt_files)

grt_data_array = {}

for seq, filepath in enumerate(grt_files):
    filename = os.path.basename(filepath)
    with open(filepath, 'r') as f:
        header = f.readline()  # Skip header
        positions = []
        for line in f:
            data = line.strip().split(' ')
            data[1] = data[1].replace(',', '')  # Remove commas from longitude
            positions.append(data)
        grt_data_array[seq] = {
            'fileName': filename,
            'latitude': [float(d[0]) for d in positions],
            'longitude': [float(d[1]) for d in positions]
        }        

#   data = np.loadtxt(filepath, dtype=float, delimiter=',\n', skiprows=1)
    # latitude = data[:, 0]
    # longitude = data[:, 1]
    # path_name = filename.split('_')[0]
    
    

# seq_name = {  'HH1', 'HH2', 'HH3', 'MK1', 'MK2', 'MK3', 'CB1', 'CD1', 'NP1', 'TST1', 'PFL1', 'TY1', 'AT1', 'CT1', 'CT2', 'CT3', 'ET1', 'ET2', 'ET3', 'ET4', 'HT1', 'HT2', 'HT3', 'WT1', 'WT2', 'WT3'};
# traj_legth = [2776    865    1415   5215   5215   5215   4030	  4269   7255   4269    964     450     249   2964   2921   3157   503    530    766    1142   5845   1340   4092   3273   3476   3273];
# Trajectory lengths
traj_length = np.array([ 2776, 450, 3273])

# SLAM dataset paths and names
slam_folder_paths = ['F-LOAM/F-LOAM/all']

slam_names = ['F-LOAM']

num_slam = len(slam_folder_paths)
num_slam_path = 26
error_all = np.zeros((num_slam, 26))

# Process each SLAM method
for slam_idx, slam_folder_path in enumerate(slam_folder_paths):
    if not os.path.exists(slam_folder_path):
        print(f"Warning: {slam_folder_path} does not exist")
        continue
    
    slam_files = glob.glob(os.path.join(slam_folder_path, '*.txt'))
    one_slam_errors = np.zeros(num_slam_path)
    
    # Create figure with subplots
    fig = plt.figure(figsize=(10, 7))
    fig.suptitle(f'Comparison of the {slam_names[slam_idx]} path and the ground truth', fontsize=12)
    
    ax_list = tight_subplot(2, 2, [0.1, 0.1], [0.1, 0.15], [0.1, 0.1])
    
    for seq in range(min(num_slam_path, len(slam_files))):
        filepath = slam_files[seq]
        filename = os.path.basename(filepath)
        
        # Read SLAM data
        slam_data = np.loadtxt(filepath, delimiter=' ')
        timestamp = slam_data[:, 0]
        tx = slam_data[:, 1]
        ty = slam_data[:, 2]
        tz = slam_data[:, 3]
        
        slam_2d_positions = np.column_stack([tx, ty])
        slam_3d_positions = np.column_stack([tx, ty, tz])
        
        # Preprocess
        slam_2d_positions, slam_3d_positions = preprocess_position(slam_2d_positions, slam_3d_positions, seq)
        
        # Ground truth indexing
        slam_grt_index = np.array([0, 2, 3])
        grt_index = slam_grt_index[seq]
        grt_available = False
        aligned_slam_2d_positions_icp = slam_2d_positions.copy()
        aligned_grt_2d_positions_icp = None
        error = 100
        
        # # Handle cases without ground truth
        # if grt_index == 0:
        #         visualize = False
        #         start_to_end_error, _ = compute_start_to_end_error(slam_3d_positions, visualize, 302.7)
        #         error = start_to_end_error / traj_length[seq] * 100
            
        # Ground truth from GPS
        if seq == 0:
            gps_file = 'grt/grt/gps_path/hh1_gps_path_grt.txt'            
            
            if os.path.exists(gps_file):
                gps_data = np.loadtxt(gps_file)
                timestamps = gps_data[:, 0]
                timestamps = timestamps - timestamps[0]
                positions = gps_data[:, 1:4]
                quaternions = gps_data[:, 4:8]
                grt_2d_positions = positions[:, :2]
                grt_available = True
        
        # Ground truth from bus path
        if grt_index != 0 and grt_index - 1 in grt_data_array:
            grt_latitude = grt_data_array[grt_index - 1]['latitude']
            grt_longitude = grt_data_array[grt_index - 1]['longitude']
            grt_2d_positions = [grt_longitude, grt_latitude]
            grt_available = True
        
        # Compute error if data is available
        if slam_3d_positions.shape[0] <= 1:
            error = 100
        elif grt_available:
            print("grt available")
            move_slam_path = 1
            aligned_slam_2d_positions_icp, aligned_grt_2d_positions_icp, rmse, rmsre = \
                align_and_compute_2d_rmse(slam_2d_positions, grt_2d_positions, move_slam_path)
            error = rmse / traj_length[seq] * 100 * 2
        
        one_slam_errors[seq] = error
        error_all[slam_idx, seq] = error
        
        # Extract sequence name
        parts = filename.split('_')
        if len(parts) >= 2:
            seq_name = parts[1]
        else:
            seq_name = filename.split('.')[0]
        
        print(f'File: {seq_name}, Index: {seq}, error: {error:.2f}')
        
        # Visualization
        # figure_index = np.array([1,2,3]) - 1
        
        this_index = seq
        
        ax = ax_list[this_index]
        grt_color = np.array([0.3, 0.3, 0.7])
        slam_color = np.array([0.9, 0.3, 0.3])
        print(aligned_grt_2d_positions_icp)
        if grt_available and aligned_grt_2d_positions_icp is not None:
            print("plotting grt")
            ax.plot(aligned_grt_2d_positions_icp[ 0], aligned_grt_2d_positions_icp[1], color="blue", linewidth=2)
        
        ax.plot(aligned_slam_2d_positions_icp[0], aligned_slam_2d_positions_icp[1], color=slam_color, linewidth=2)
        rmse_formatted = f'{error:.1f}'
        
        # if this_index == 0:
        #     seq_name_display = 'hh1'
        # elif this_index == 1:
        #     seq_name_display = 'hh2'
        # elif this_index == 2:
        #     seq_name_display = 'hh3'
        # else:
        seq_name_display = seq_name
        
        ax.set_title(f'{this_index + 1}, {seq_name_display}, error: {rmse_formatted}%', fontsize=10)
        # ax.set_aspect('equal')
        
        # Set background color based on error
        borders = [5, 10, 100]
        back_colors = [(0.99, 1, 0.99), (1, 1, 0.99), (1, 0.99, 0.99)]
        border_colors = [(0, 0.5, 0), (0.8, 0.8, 0), (1, 0.5, 0)]
        
        if error < borders[0]:
            back_color = back_colors[0]
            border_color = border_colors[0]
        elif error < borders[1]:
            back_color = back_colors[1]
            border_color = border_colors[1]
        else:
            back_color = back_colors[2]
            border_color = border_colors[2]
        
        ax.set_facecolor(back_color)
        
        # Add border
        xlim = ax.get_xlim()
        ylim = ax.get_ylim()
        ax.plot([xlim[0], xlim[0]], [ylim[0], ylim[1]], color=border_color, linewidth=1.5)  # left
        ax.plot([xlim[0], xlim[1]], [ylim[1], ylim[1]], color=border_color, linewidth=1.5)  # top
        ax.plot([xlim[1], xlim[1]], [ylim[0], ylim[1]], color=border_color, linewidth=1.5)  # right
        ax.plot([xlim[0], xlim[1]], [ylim[0], ylim[0]], color=border_color, linewidth=1.5)  # bottom
    

    # Add legend in the n-th subplot
    n = 3

    if len(ax_list) > n:
        ax = ax_list[n]
        ax.plot([0, 0.5], [0.9, 0.9], color=grt_color, linewidth=2)
        ax.plot([0, 0.5], [0.7, 0.7], color=slam_color, linewidth=2)
        ax.text(0.6, 0.9, 'Ground truth', fontsize=10, ha='left')
        ax.text(0.6, 0.7, 'SLAM path', fontsize=10, ha='left')
        ax.set_xlim([0, 1])
        ax.set_ylim([0, 1])
        
        border_width = 1.5
        
        for i, back_color in enumerate(back_colors):
            rect = Rectangle((0, 0.4 - i*0.2), 0.5, 0.16, facecolor=back_color, 
                            edgecolor=border_colors[i], linewidth=border_width)
            ax.add_patch(rect)
        
        ax.text(0.6, 0.52, 'Precise', fontsize=10, ha='left')
        ax.text(0.6, 0.32, 'Acceptable', fontsize=10, ha='left')
        ax.text(0.6, 0.12, 'Fail', fontsize=10, ha='left')
        ax.axis('off')
    
    if len(ax_list) > (n + 1):
        ax_list[n + 1].axis('off')
    
    plt.tight_layout()
    plt.savefig(f'slam_comparison_{slam_names[slam_idx]}.png', dpi=150, bbox_inches='tight')
    plt.show()

# Draw error comparison figure
# fig, ax = plt.subplots(figsize=(13, 3))
# fig.subplots_adjust(left=0.05, right=0.95, bottom=0.15, top=0.85)

# colors = [
#     np.array([0, 191, 255]) / 255,
#     np.array([255, 165, 0]) / 255,
#     np.array([220, 20, 60]) / 255,
#     np.array([50, 205, 50]) / 255,
# ]

# h_list = []

# for slam_idx in range(num_slam):
#     reordered_indices = np.array([12, 13, 11, 17, 18, 19, 2, 3, 20, 22, 21, 23, 1, 4, 5, 6, 7, 8, 9, 10, 14, 15, 16, 24, 25, 26]) - 1
#     one_slam_errors = error_all[slam_idx, :]
#     constrained_errors = np.clip(one_slam_errors, 0, 100)
#     reordered_errors = constrained_errors[reordered_indices]
    
#     h, = ax.plot(range(1, num_slam_path + 1), reordered_errors, linewidth=2, color=colors[slam_idx], label=slam_names[slam_idx])
#     h_list.append(h)

# ax.set_xlabel('Sequence')
# ax.set_xlim([0.5, 26.5])
# max_y = 100
# ax.set_ylim([0, max_y])
# ax.set_ylabel('Relative 2D RMSE error  (%)')
# ax.set_title('Relative 2D RMSE error of SLAM methods in 26 sequences', fontsize=14)
# ax.set_xticks(range(1, num_slam_path + 1))
# ax.set_xticklabels(['HH1', 'HH2', 'HH3', 'MK1', 'MK2', 'MK3', 'CB1', 'CD1', 'NP1', 'TST1', 'PFL1', 'TY1', 'AT1', 'CT1', 'CT2', 'CT3', 'ET1', 'ET2', 'ET3', 'ET4', 'HT1', 'HT2', 'HT3', 'WT1', 'WT2', 'WT3'], rotation=45)

# # Add background regions
# basic_color = 0.32
# change_color = 0.35
# background_colors = [
#     [change_color + 0.05, basic_color, basic_color],
#     [basic_color, change_color, basic_color],
#     [basic_color, basic_color, change_color]
# ]
# boundary_lines = [0.5, 6.5, 12.5, 26.5]
# labels = ['Dyna-pedestrian', 'Dyna-vehicle', 'Tunnel']

# for seq in range(len(boundary_lines) - 1):
#     area_x = [boundary_lines[seq], boundary_lines[seq + 1]]
#     area_y = [max_y, max_y]
    
#     mid_x = (area_x[0] + area_x[1]) / 2
#     ax.text(mid_x, 0.9 * max_y, labels[seq], ha='center', va='bottom', fontsize=12, color='white', weight='bold')
    
#     ax.axvspan(area_x[0], area_x[1], alpha=0.2, color=background_colors[seq])
#     ax.axvline(boundary_lines[seq + 1], linestyle='--', color=[0.5, 0.5, 0.5], linewidth=1.5)

# ax.legend(h_list, slam_names, loc='upper right')
# ax.grid(True, alpha=0.3)

# plt.savefig('slam_error_comparison.png', dpi=150, bbox_inches='tight')
# plt.show()

print("Visualization complete!")
