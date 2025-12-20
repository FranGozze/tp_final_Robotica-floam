#!/usr/bin/env python3
"""
Este script realiza el trabajo de reformatear los archivos de trayectoria TUM
y además alinea el camino aproximado con el groundtruth calculando la transformación 
óptima entre dos nubes de puntos 2D usando ICP y Kabsch.

La mayor parte de las funciones que se encuentran en este script son traducciones directas
de las funciones MATLAB que se utilizan en el dataset HK_MEMS.

"""

import argparse
import sys
from typing import List, Optional
import numpy as np
import transforms3d

# Formatea el archivo salida removiendo las ',' y remplazandolas por espacios
def parse_line(line: str, delimiter: Optional[str] = None) -> Optional[List[str]]:
    line = line.strip()
    if not line or line.startswith('#'):
        return None
    if delimiter == 'comma' or (delimiter is None and ',' in line):
        parts = [p.strip() for p in line.split(',') if p.strip() != '']
    else:
        parts = line.split()
    return parts



def simple_2d_icp(source, target, max_iterations, tolerance):
    """
    Traducción directa de simple_2d_icp de MATLAB a Python.
    
    source: (Ns, 2)
    target: (Nt, 2)
    """

    source = np.asarray(source, dtype=float)
    target = np.asarray(target, dtype=float)

    source_length = source.shape[0]
    target_length = target.shape[0]

    # -----------------------------------------
    # Downsampling
    # -----------------------------------------
    if source_length > target_length:
        step = source_length / target_length
        sample_index = np.ceil(np.arange(1, source_length + 1, step)).astype(int) - 1
        source_sampled = source[sample_index]
        target_sampled = target
    else:
        step = target_length / source_length
        sample_index = np.ceil(np.arange(1, target_length + 1, step)).astype(int) - 1
        source_sampled = source
        target_sampled = target[sample_index]

    num_points_source = source_sampled.shape[0]
    num_points_target = target_sampled.shape[0]

    aligned_source = source_sampled.copy()
    match_in_target = np.zeros((num_points_source, 2))

    R = np.eye(2)
    t = np.zeros((2, 1))

    T = np.array([
        [1.0, 0.0, 0.0],
        [0.0, 1.0, 0.0],
        [0.0, 0.0, 0.0]
    ])

    RMSE = []

    # -----------------------------------------
    # ICP loop
    # -----------------------------------------
    for _ in range(max_iterations):

        # Nearest neighbor (fuerza bruta)
        for i in range(num_points_source):
            min_dist = np.inf
            nearest_idx = 0
            for j in range(num_points_target):
                dist = np.linalg.norm(aligned_source[i] - target_sampled[j])
                if dist < min_dist:
                    min_dist = dist
                    nearest_idx = j
            match_in_target[i] = target_sampled[nearest_idx]

        # Centroides
        centroid_source = np.mean(aligned_source, axis=0)
        centroid_target = np.mean(match_in_target, axis=0)

        # Centrados
        centered_source = aligned_source - centroid_source
        centered_target = match_in_target - centroid_target

        # Covarianza
        H = centered_source.T @ centered_target

        # SVD
        U, _, Vt = np.linalg.svd(H)

        R_i = Vt.T @ U.T

        # Evitar reflexión
        if np.linalg.det(R_i) < 0:
            Vt[-1, :] *= -1
            R_i = Vt.T @ U.T

        t_i = centroid_target - R_i @ centroid_source

        # Actualizar puntos
        aligned_source = (R_i @ aligned_source.T).T + t_i

        # Transformación homogénea acumulada
        T_i = np.array([
            [R_i[0, 0], R_i[0, 1], t_i[0]],
            [R_i[1, 0], R_i[1, 1], t_i[1]],
            [0.0,       0.0,       0.0]
        ])
        T = T_i @ T

        # RMSE
        diff = aligned_source - match_in_target
        rmse = np.sqrt(np.mean(np.sum(diff ** 2, axis=1)))
        RMSE.append(rmse)

        if rmse < tolerance:
            break

    # Resultado final
    R = T[:2, :2]
    t = T[2, :2]

    aligned_target = target
    aligned_source = (R @ source.T).T + t

    return R, t, aligned_source, aligned_target, np.array(RMSE)


def align_point_clouds(source, target):
    """
    Traducción directa de la funcion align_point_clouds del archivo align_and_compute_2d_rmse.m  a Python.
    Alinea source con target cuando tienen distinta cantidad de puntos,
    usando downsampling por índice + SVD (Kabsch).
    
    source: (N, D)
    target: (M, D)
    
    Retorna:
        R: (D, D) matriz de rotación
        t: (D,) vector de traslación
    """

    source = np.asarray(source, dtype=float)
    target = np.asarray(target, dtype=float)

    Ns, D = source.shape
    Nt, Dt = target.shape
    assert D == Dt, "Source y target deben tener la misma dimensión"

    # -------------------------------------------------
    # 1. Downsampling para igualar longitudes
    # -------------------------------------------------
    if Ns > Nt:
        step = Ns / Nt
        idx = np.floor(np.arange(Nt) * step).astype(int)
        source_ds = source[idx]
        target_ds = target
    elif Nt > Ns:
        step = Nt / Ns
        idx = np.floor(np.arange(Ns) * step).astype(int)
        source_ds = source
        target_ds = target[idx]
    else:
        source_ds = source
        target_ds = target

    # Seguridad extra
    N = min(len(source_ds), len(target_ds))
    source_ds = source_ds[:N]
    target_ds = target_ds[:N]

    # -------------------------------------------------
    # 2. Filtrar NaN / Inf manteniendo correspondencia
    # -------------------------------------------------
    valid = (
        np.isfinite(source_ds).all(axis=1) &
        np.isfinite(target_ds).all(axis=1)
    )

    source_ds = source_ds[valid]
    target_ds = target_ds[valid]

    if len(source_ds) < D:
        raise ValueError("No hay suficientes puntos válidos para alinear")

    # -------------------------------------------------
    # 3. Kabsch clásico
    # -------------------------------------------------
    source_center = np.mean(source_ds, axis=0)
    target_center = np.mean(target_ds, axis=0)

    X = source_ds - source_center
    Y = target_ds - target_center

    H = X.T @ Y
    U, _, Vt = np.linalg.svd(H)

    R = Vt.T @ U.T

    # Evitar reflexión
    if np.linalg.det(R) < 0:
        Vt[-1, :] *= -1
        R = Vt.T @ U.T

    t = target_center - R @ source_center

    return R, t



def preprocess_position(slam_positions):
    """
    Traducción directa de preprocess_position.m a Python
    
    slam_2d_positions: (N, 2)
    """

    slam_2d_positions = np.asarray([[p[1], p[2]] for p in slam_positions], dtype=float)
    slam_3d_positions = np.asarray([[p[1], p[2], p[3]] for p in slam_positions], dtype=float)

    # -------------------------------------------------
    # Remove NaN rows
    # -------------------------------------------------
    nan_rows = np.any(np.isnan(slam_3d_positions), axis=1)
    negative_nan_rows = np.any(
        (slam_3d_positions < 0) & np.isnan(slam_3d_positions),
        axis=1
    )

    rows_to_remove = nan_rows | negative_nan_rows

    slam_3d_positions = slam_3d_positions[~rows_to_remove]
    slam_2d_positions = slam_2d_positions[~rows_to_remove]

    # -------------------------------------------------
    # Skip too close points
    # -------------------------------------------------
    threshold_min = 2
    threshold_max = 10  # (no usado, igual que en MATLAB)

    new_slam_2d_positions = [slam_2d_positions[0]]
    new_slam_3d_positions = [slam_3d_positions[0]]

    i = 0
    while i <= slam_2d_positions.shape[0] - 2:
        point_kept = False

        for j in range(i + 1, slam_2d_positions.shape[0] - 1):
            distance = np.linalg.norm(
                slam_2d_positions[j] - slam_2d_positions[i]
            )

            if distance < threshold_min:
                # Skip points closer than threshold_min
                continue
            else:
                new_slam_2d_positions.append(slam_2d_positions[j])
                new_slam_3d_positions.append(slam_3d_positions[j])
                i = j
                point_kept = True
                break

        if not point_kept:
            i += 1

    slam_2d_positions = np.array(new_slam_2d_positions)
    slam_3d_positions = np.array(new_slam_3d_positions)

    return slam_2d_positions, slam_3d_positions


def process_data(source_file, target_file):

    source = [list(map(float, parse_line(line, None))) for line in source_file if line.strip()]
    source_points = [[p[1],p[2]] for p in source]
    target_points = [list(map(float, parse_line(line, None))) for line in target_file if line.strip()]

    target_points_2d, target_points_3d = preprocess_position(target_points)
    R, t = align_point_clouds(target_points_2d, source_points)
    
    
    aligned_points = []
    for i,p in enumerate(target_points_2d):
        p_np = np.array(p)
        p_aligned = R @ p_np + t
        aligned_points.append(p_aligned)
    max_iterations = 10
    tolerance = 1e-6
    R_icp, t_icp, aligned_target_positions_icp, aligned_grt_2d_positions_icp,  RMSE = simple_2d_icp(
        [[p[0], p[1]] for p in aligned_points],
        source_points,
        max_iterations,
        tolerance
    )
    aligned_points_icp = []
    x_first, y_first   = aligned_target_positions_icp[0]

    aligned_target_positions_icp = aligned_target_positions_icp - np.array([x_first, y_first])
    
    
    for i, tum in enumerate(aligned_target_positions_icp):
        x = tum[0]
        y = tum[1]
        z = target_points[i][3]  
        qw, qx, qy, qz = target_points[i][7], target_points[i][4], target_points[i][5], target_points[i][6]
        aligned_points_icp.append(f"{target_points[i][0]/1000000000} {x:.6f} {y:.6f} {z:.6f} {qx:.6f} {qy:.6f} {qz:.6f} {qw:.6f}")

    return aligned_points_icp

def main():
    parser = argparse.ArgumentParser(description='Formatea y alinea archivos TUM usando ICP y Kabsch.')
    parser.add_argument('input', nargs='?', help='Input TUM file (default: stdin)')
    parser.add_argument('--source', required=True, help='Source TUM file (default: stdin)')
    parser.add_argument('-o', '--output', help='Output file (default: stdout)')    

    args = parser.parse_args()

    f_target_in = sys.stdin if not args.input else open(args.input, 'r')
    f_source_in = sys.stdin if not args.source else open(args.source, 'r')
    fout = sys.stdout if not args.output else open(args.output, 'w')

    try:
        data = process_data(f_source_in, f_target_in)
        for line in data:
            fout.write(line + "\n")
    finally:
        if f_source_in is not sys.stdin:
            f_source_in.close()
        if f_target_in is not sys.stdin:
            f_target_in.close()
        if fout is not sys.stdout:
            fout.close()

    if args.output:
        print(f'Wrote {len(data)} lines to {args.output}', file=sys.stderr)


if __name__ == '__main__':
    main()
