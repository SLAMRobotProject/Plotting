import numpy as np
import pandas as pd
import scipy.io as sio
import matplotlib.pyplot as plt
from scipy.spatial import cKDTree
from scipy.stats import chi2
from scipy.interpolate import interp1d
import os
from numpy.linalg import LinAlgError

# Load the CSV file
def load_csv(file_path):
    data = pd.read_csv(file_path, delimiter=' ', header=0, skiprows=0, engine='python')

    '''
    Coordinate frame csv data:
        - z down
        - y = forward
        - x = right
    '''

    timestamps = pd.to_datetime(data.iloc[:, 0], format='%H:%M:%S.%f', errors="coerce")

    x_est = data.iloc[:, 2].astype(float).to_numpy() / 100 
    y_est = data.iloc[:, 3].astype(float).to_numpy() / 100  

    P_matrices = np.array([list(map(float, row.split(','))) for row in data.iloc[:, 5]])

    P_matrices = P_matrices.reshape(-1, 5, 5)
    
    timestamps = pd.Series(timestamps)

    try:
        np.linalg.cholesky(P_matrices[0][:2, :2])
        print("Covariance matrix is positive definite.")
    except LinAlgError:
        print("Covariance matrix is not positive definite!")

    return timestamps, x_est, y_est, P_matrices


# Load the .MAT file
def load_mat(file_path):
    mat_data = sio.loadmat(file_path)
    
    filename = os.path.splitext(os.path.basename(file_path))[0].replace('.', '_')
    
    trajectories = mat_data[filename][0, 0]['Trajectories'][0, 0]['Labeled'][0, 0]['Data']
    
    '''
    Coordinate frame mat data:
        - x = forward
        - y = left 
        - z = up
    '''
    
    X = trajectories[0, 0, :]
    Y = trajectories[0, 1, :]

    X_rotated = -Y
    Y_rotated = X

    X_offset = X_rotated[0]
    Y_offset = Y_rotated[0]
    X_rotated = X_rotated - X_offset
    Y_rotated = Y_rotated - Y_offset

    X_rotated = X_rotated / 1000
    Y_rotated = Y_rotated / 1000

    start_time_str = mat_data[filename][0, 0]['Timestamp'][0]
    start_time_str = start_time_str.split('\t')[0]
    start_time_str = start_time_str.split(', ')[1]
    start_time = pd.to_datetime(start_time_str, format='%H:%M:%S.%f')
    
    timestamps = pd.date_range(start=start_time, periods=len(X_rotated), freq='10ms')  # 100Hz data

    timestamps = pd.Series(timestamps)

    return timestamps, X_rotated, Y_rotated


def calculate_refresh_rate(timestamps):
    if len(timestamps) < 1000:
        raise ValueError("Not enough data points to calculate refresh rate.")
    time_diffs = np.diff(timestamps[:100].astype(np.int64)) / 1e9  
    avg_time_diff = np.mean(time_diffs)
    refresh_rate = 1 / avg_time_diff
    return refresh_rate


def interpolate_ground_truth(est_timestamps, true_timestamps, x_true, y_true):
    est_timestamps = pd.to_datetime(est_timestamps)
    true_timestamps = pd.to_datetime(true_timestamps)

    est_seconds = (est_timestamps - est_timestamps.iloc[0]).dt.total_seconds()
    true_seconds = (true_timestamps - true_timestamps.iloc[0]).dt.total_seconds()

    interp_x = interp1d(true_seconds, x_true, kind='linear', fill_value="extrapolate")
    interp_y = interp1d(true_seconds, y_true, kind='linear', fill_value="extrapolate")

    x_true_interp = interp_x(est_seconds)
    y_true_interp = interp_y(est_seconds)

    return x_true_interp, y_true_interp


def compute_nees(x_true, y_true, x_est, y_est, P_matrices):
    errors = np.vstack([x_true - x_est, y_true - y_est])
    nees_values = []
    for err, P in zip(errors.T, P_matrices):
        P_xy = P[:2, :2]  # Extract the 2x2 submatrix for x and y
        nees = err.T @ np.linalg.inv(P_xy) @ err
        nees_values.append(nees)

    print("NEES values (first 5):", nees_values[:5])
    print("First P_xy matrix:\n", P_matrices[0][:2, :2])
    print("First error vector:", errors[:, 0])
    print("First NEES value:", nees_values[0])
    print("________________________________________")

    return np.array(nees_values)


def plot_nees(nees_values, alpha=0.95):
    dof = 2  
    N = len(nees_values)
    
    lower_bound_95 = chi2.ppf((1 - alpha) / 2, dof * N) / N
    upper_bound_95 = chi2.ppf((1 + alpha) / 2, dof * N) / N
    
    lower_bound_100 = chi2.ppf(0.000001, dof * N) / N  # Approximation for maximum possible value (0.0 = 0)
    upper_bound_100 = chi2.ppf(0.999999, dof * N) / N  # Approximation for maximum possible value (1.0 = inf)
    
    anees = np.mean(nees_values)
    mean_nees = chi2.ppf(0.5, dof * N) / N 

    above_mean = np.sum(nees_values > mean_nees) / len(nees_values) * 100
    below_mean = np.sum(nees_values < mean_nees) / len(nees_values) * 100
    within_bounds = np.sum((nees_values >= lower_bound_95) & (nees_values <= upper_bound_95)) / len(nees_values) * 100

    plt.figure(figsize=(12, 6))
    plt.plot(nees_values, label='NEES')
    plt.axhline(y=upper_bound_95, color='orange', linestyle='--', label=f'95% CI (DOF={dof})')
    plt.axhline(y=lower_bound_95, color='orange', linestyle='--', label=None)
    plt.axhline(y=upper_bound_100, color='red', linestyle='--', label=f'100% CI (DOF={dof})')
    plt.axhline(y=lower_bound_100, color='red', linestyle='--', label=None)
    plt.axhline(y=anees, color='green', linestyle=':', label=f'ANEES (Average NEES, {anees:.2f})')
    plt.xlabel('Time step')
    plt.ylabel('NEES')
    plt.title('NEES Analysis')
    plt.legend()
    plt.grid(True)

    text_str = (
        f"Upper 95% bound: {upper_bound_95:.2f}\n"
        f"Lower 95% bound: {lower_bound_95:.2f}\n"
        f"Upper 100% bound (approx): {upper_bound_100:.2f}\n"
        f"Lower 100% bound (approx): {lower_bound_100:.2f}\n"
        f"Mean NEES (chi-squared): {mean_nees:.2f}\n"
        f"ANEES (Average NEES): {anees:.2f}\n"
        f"Percentage above mean: {above_mean:.2f}%\n"
        f"Percentage below mean: {below_mean:.2f}%\n"
        f"Percentage within 95% bounds: {within_bounds:.2f}%"
    )
    plt.gcf().text(0.15, 0.4, text_str, fontsize=10, bbox=dict(facecolor='white', alpha=0.8))

    plt.show()


def trim_start_times(est_timestamps, x_est, y_est, true_timestamps, x_true, y_true):
    est_timestamps = pd.to_datetime(est_timestamps)
    true_timestamps = pd.to_datetime(true_timestamps)

    while len(est_timestamps) > 0 and len(true_timestamps) > 0 and est_timestamps.iloc[0] < true_timestamps.iloc[0]:
        est_timestamps = est_timestamps.iloc[1:].reset_index(drop=True)
        x_est = x_est[1:]
        y_est = y_est[1:]

    while len(est_timestamps) > 0 and len(true_timestamps) > 0 and true_timestamps.iloc[0] < est_timestamps.iloc[0]:
        true_timestamps = true_timestamps.iloc[1:].reset_index(drop=True)
        x_true = x_true[1:]
        y_true = y_true[1:]

    if len(est_timestamps) == 0 or len(true_timestamps) == 0:
        raise ValueError("One of the datasets is empty after trimming start times. Check your input data.")

    return est_timestamps, x_est, y_est, true_timestamps, x_true, y_true


def main(csv_file, mat_file):
    est_timestamps, x_est, y_est, P_matrices = load_csv(csv_file)
    true_timestamps, x_true, y_true = load_mat(mat_file)

    est_timestamps, x_est, y_est, true_timestamps, x_true, y_true = trim_start_times(
        est_timestamps, x_est, y_est, true_timestamps, x_true, y_true
    )

    
    refresh_rate = calculate_refresh_rate(est_timestamps)
    resample_rate = f'{1000 / refresh_rate:.2f}L'
    print(f"Sample rate estimates: {resample_rate} Hz")

    x_true_interp, y_true_interp = interpolate_ground_truth(est_timestamps, true_timestamps, x_true, y_true)

    min_length = min(len(est_timestamps), len(x_true_interp))
    est_timestamps = est_timestamps[:min_length]
    x_est = x_est[:min_length]
    y_est = y_est[:min_length]
    P_matrices = P_matrices[:min_length]
    x_true_interp = x_true_interp[:min_length]
    y_true_interp = y_true_interp[:min_length]

    nees_values = compute_nees(x_true_interp, y_true_interp, x_est, y_est, P_matrices)
    plot_nees(nees_values)

main('data/odometry_data/NEES_data/nrf6_new_P_0_and_pose_est_measured_R.csv', 'data/OptiTrack_data/NEES_data/nrf6_new_P_0_and_pose_est_measured_R.mat')
