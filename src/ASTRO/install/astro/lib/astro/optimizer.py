#!/usr/bin/env python3
"""
optimizer.py — Bayesian optimizer for ASTRO sim-to-real parameter tuning.
Modifies robot_core.xacro and diff_drive_controller.yaml, runs headless Gazebo,
replays real cmd_vel, compares sim vs real odometry, finds best parameters.

Usage:
    source ~/astro_ws_pas/sim_env.sh
    python3 optimizer.py
"""

import os
import sys
import time
import shutil
import signal
import subprocess
import math
import re
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from scipy.interpolate import interp1d

# ===================================================================
# USER-CONFIGURABLE PARAMETERS
# ===================================================================
PARAM_BOUNDS = {
    'wheel_radius':     (0.025, 0.04),
    'wheel_separation': (0.280, 0.320),
    'wheel_thickness':  (0.015, 0.025),
    'chassis_mass':     (1.5,   4.0),
    'friction':         (0.3,   1.0),
}
PARAM_NAMES = list(PARAM_BOUNDS.keys())
PARAM_DIMENSIONS = [PARAM_BOUNDS[k] for k in PARAM_NAMES]

N_EPOCHS = 167
SIM_TIMEOUT = 180         # seconds wall-time to wait per epoch
REAL_TIME_FACTOR = 5.0
SIM_STARTUP_WAIT = 20     # seconds to wait for Gazebo + controllers

BAG_PATH = '/home/tona/astro_ws_pas/rosbag2_2026_08_19-15_45_34'
WS_PATH = '/home/tona/astro_ws_pas'
INSTALL_DESC = os.path.join(WS_PATH, 'install/astro/share/astro/description')
INSTALL_CFG = os.path.join(WS_PATH, 'install/astro/share/astro/config')
OUTPUT_DIR = os.path.join(WS_PATH, 'optimizer_output')
SCRIPTS_DIR = os.path.dirname(os.path.abspath(__file__))
WORLD_FILE = os.path.join(WS_PATH, 'worlds/optimizer_world.world')

# ===================================================================
# BAG DATA EXTRACTION
# ===================================================================
def extract_bag_data(bag_path, output_path):
    """Extract cmd_vel and odom from rosbag, save to .npz."""
    if os.path.exists(output_path):
        print(f'[INFO] Bag data already extracted at {output_path}')
        return

    print('[INFO] Extracting bag data...')
    import rosbag2_py
    from rclpy.serialization import deserialize_message
    from geometry_msgs.msg import Twist
    from nav_msgs.msg import Odometry

    reader = rosbag2_py.SequentialReader()
    storage_options = rosbag2_py.StorageOptions(uri=bag_path, storage_id='sqlite3')
    converter_options = rosbag2_py.ConverterOptions(
        input_serialization_format='cdr', output_serialization_format='cdr'
    )
    reader.open(storage_options, converter_options)

    # Filter topics
    filter_ = rosbag2_py.StorageFilter(topics=['/cmd_vel', '/odom'])
    reader.set_filter(filter_)

    cmd_times, cmd_lin, cmd_ang = [], [], []
    odom_times, odom_x, odom_y, odom_yaw = [], [], [], []
    t0 = None

    while reader.has_next():
        topic, data, timestamp = reader.read_next()
        t_sec = timestamp * 1e-9
        if t0 is None:
            t0 = t_sec

        if topic == '/cmd_vel':
            msg = deserialize_message(data, Twist)
            cmd_times.append(t_sec - t0)
            cmd_lin.append(msg.linear.x)
            cmd_ang.append(msg.angular.z)
        elif topic == '/odom':
            msg = deserialize_message(data, Odometry)
            odom_times.append(t_sec - t0)
            odom_x.append(msg.pose.pose.position.x)
            odom_y.append(msg.pose.pose.position.y)
            q = msg.pose.pose.orientation
            siny = 2.0 * (q.w * q.z + q.x * q.y)
            cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
            odom_yaw.append(math.atan2(siny, cosy))

    np.savez(output_path,
             cmd_vel_times=np.array(cmd_times),
             cmd_vel_linear=np.array(cmd_lin),
             cmd_vel_angular=np.array(cmd_ang),
             odom_times=np.array(odom_times),
             odom_x=np.array(odom_x),
             odom_y=np.array(odom_y),
             odom_yaw=np.array(odom_yaw))
    print(f'[INFO] Extracted {len(cmd_times)} cmd_vel, {len(odom_times)} odom msgs')


# ===================================================================
# FILE MODIFICATION
# ===================================================================
XACRO_TEMPLATE = '''<?xml version="1.0"?>
<robot xmlns:xacro="http://www.ros.org/wiki/xacro">

    <xacro:property name="wheel_radius" value="{wheel_radius}"/>
    <xacro:property name="wheel_separation" value="{wheel_separation}"/>
    <xacro:property name="wheel_thickness" value="{wheel_thickness}"/>
    <xacro:property name="wheel_mass" value="0.050"/>
    <xacro:property name="ground_clearance" value="0.0146"/>
    <xacro:property name="caster_wheel_mass" value="0.02"/>

    <xacro:property name="lidar_x_offset" value="0.038471"/>
    <xacro:property name="lidar_z_offset" value="0.0838"/>


    <xacro:include filename="inertial_macros.xacro"/>
    <xacro:include filename="materials.xacro"/>
    <xacro:include filename="caster_wheels.xacro"/>

    <link name="base_footprint"/>

    <joint name="base_link_joint" type="fixed">
        <parent link="base_footprint"/>
        <child link="base_link"/>
        <origin xyz="0 0 ${{wheel_radius}}"/>
    </joint>

    <link name="base_link"/>

    <joint name="chassis_joint" type="fixed">
        <parent link="base_link"/>
        <child link="chassis"/>
        <origin xyz="0 0 0"/>
    </joint>

    <link name="chassis">
        <visual>
            <origin rpy="0 0 0" xyz="0 0 0"/>
            <geometry>
                <mesh filename="package://astro/meshes/ASTRO_Chassis.stl" scale="0.001 0.001 0.001"/>
            </geometry>
            <material name="gray"/>
        </visual>
        <collision>
            <origin rpy="0 0 0" xyz="0 0 0"/>
            <geometry>
                <mesh filename="package://astro/meshes/ASTRO_Chassis.stl" scale="0.001 0.001 0.001"/>
            </geometry>
        </collision>
        <xacro:inertial_box mass="{chassis_mass}" x="0.3" y="0.25" z="0.15">
            <origin xyz="0 0 0.05" rpy="0 0 0"/>
        </xacro:inertial_box>
    </link>

    <joint name="left_wheel_joint" type="continuous">
        <parent link="base_link"/>
        <child link="left_wheel"/>
        <origin xyz="0 ${{wheel_separation/2}} 0" rpy="-${{pi/2}} 0 0"/>
        <axis xyz="0 0 1"/>
    </joint>

    <link name="left_wheel">
        <visual>
            <geometry>
                <cylinder length="${{wheel_thickness}}" radius="${{wheel_radius}}" />
            </geometry>
            <material name="orange"/>
        </visual>
        <collision>
            <geometry>
                <cylinder length="${{wheel_thickness}}" radius="${{wheel_radius}}" />
            </geometry>
        </collision>
        <xacro:inertial_cylinder mass="${{wheel_mass}}" length="${{wheel_thickness}}" radius="${{wheel_radius}}">
            <origin xyz="0 0 0" rpy="0 0 0"/>
        </xacro:inertial_cylinder>
    </link>

    <joint name="right_wheel_joint" type="continuous">
        <parent link="base_link"/>
        <child link="right_wheel"/>
        <origin xyz="0 -${{wheel_separation/2}} 0" rpy="${{pi/2}} 0 0"/>
        <axis xyz="0 0 -1"/>
    </joint>

    <link name="right_wheel">
        <visual>
            <geometry>
                <cylinder length="${{wheel_thickness}}" radius="${{wheel_radius}}" />
            </geometry>
            <material name="orange"/>
        </visual>
        <collision>
            <geometry>
                <cylinder length="${{wheel_thickness}}" radius="${{wheel_radius}}" />
            </geometry>
        </collision>
        <xacro:inertial_cylinder mass="${{wheel_mass}}" length="${{wheel_thickness}}" radius="${{wheel_radius}}">
            <origin xyz="0 0 0" rpy="0 0 0"/>
        </xacro:inertial_cylinder>
    </link>

    <gazebo reference="left_wheel">
        <mu1>{friction}</mu1>
        <mu2>{friction}</mu2>
    </gazebo>

    <gazebo reference="right_wheel">
        <mu1>{friction}</mu1>
        <mu2>{friction}</mu2>
    </gazebo>

</robot>
'''

def write_xacro(params):
    """Write modified robot_core.xacro to install directory."""
    content = XACRO_TEMPLATE.format(**params)
    path = os.path.join(INSTALL_DESC, 'robot_core.xacro')
    with open(path, 'w') as f:
        f.write(content)

def write_yaml(params):
    """Write modified diff_drive_controller.yaml to install directory."""
    yaml_path = os.path.join(INSTALL_CFG, 'diff_drive_controller.yaml')
    with open(yaml_path, 'r') as f:
        content = f.read()
    content = re.sub(r'wheel_separation:\s*[\d.]+',
                     f'wheel_separation: {params["wheel_separation"]:.4f}', content)
    content = re.sub(r'wheel_radius:\s*[\d.]+',
                     f'wheel_radius: {params["wheel_radius"]:.4f}', content)
    with open(yaml_path, 'w') as f:
        f.write(content)

def set_world_rtf(rtf):
    """Set real_time_factor in the optimizer world file."""
    with open(WORLD_FILE, 'r') as f:
        content = f.read()
    content = re.sub(r'<real_time_factor>[\d.]+</real_time_factor>',
                     f'<real_time_factor>{rtf}</real_time_factor>', content)
    with open(WORLD_FILE, 'w') as f:
        f.write(content)


# ===================================================================
# SIMULATION MANAGEMENT
# ===================================================================
def kill_sim_processes():
    """Kill all Gazebo and related processes."""
    for proc_name in ['gz sim', 'ruby', 'parameter_bridge',
                      'robot_state_publisher', 'spawner',
                      'sim_epoch_runner']:
        subprocess.run(['pkill', '-f', proc_name],
                      capture_output=True, timeout=5)
    time.sleep(3)
    for proc_name in ['gz sim', 'ruby']:
        subprocess.run(['pkill', '-9', '-f', proc_name],
                      capture_output=True, timeout=5)
    time.sleep(2)

def run_epoch(epoch_num, params, bag_data_path, bag_duration):
    """Run a single simulation epoch. Returns RMSE error or large penalty."""
    print(f'\n{"="*60}')
    print(f'EPOCH {epoch_num}')
    print(f'Parameters: { {k: f"{v:.4f}" for k,v in params.items()} }')
    print(f'{"="*60}')

    # 1. Modify files
    write_xacro(params)
    write_yaml(params)

    sim_odom_path = os.path.join(OUTPUT_DIR, 'sim_odom_result.npz')
    if os.path.exists(sim_odom_path):
        os.remove(sim_odom_path)

    # 2. Launch simulation
    env = os.environ.copy()
    env['ROS_DOMAIN_ID'] = '42'

    print('[INFO] Launching headless simulation...')
    sim_proc = subprocess.Popen(
	    ['bash', '-c',
	     f'source /opt/ros/humble/setup.bash && '
	     f'source {WS_PATH}/install/setup.bash && '
	     f'export ROS_DOMAIN_ID=42 && '
	     f'ros2 launch astro optimizer_sim.launch.py'],
	    env=env,
	    stdout=None,
	    stderr=None,
	    preexec_fn=os.setsid
	)

    # 3. Wait for simulation startup
    print(f'[INFO] Waiting {SIM_STARTUP_WAIT}s for simulation to initialize...')
    time.sleep(SIM_STARTUP_WAIT)

    if sim_proc.poll() is not None:
        print('[ERROR] Simulation process died during startup!')
        kill_sim_processes()
        return 1000.0

    # 4. Launch epoch runner
    print('[INFO] Launching epoch runner...')
    runner_cmd = (
        f'source {WS_PATH}/install/setup.bash && '
        f'export ROS_DOMAIN_ID=42 && '
        f'python3 {os.path.join(SCRIPTS_DIR, "sim_epoch_runner.py")} '
        f'--bag-data {bag_data_path} '
        f'--output {sim_odom_path} '
        f'--duration {bag_duration:.2f}'
    )
    runner_proc = subprocess.Popen(
	    ['bash', '-c', runner_cmd],
	    env=env,
	    stdout=None,
	    stderr=None,
	    preexec_fn=os.setsid
	)

    # 5. Wait for runner to complete
    start_wait = time.time()
    while time.time() - start_wait < SIM_TIMEOUT:
        if os.path.exists(sim_odom_path):
            time.sleep(2)  # let file finish writing
            break
        if runner_proc.poll() is not None:
            break
        time.sleep(1)

    # 6. Cleanup
    try:
        os.killpg(os.getpgid(runner_proc.pid), signal.SIGTERM)
    except Exception:
        pass
    try:
        os.killpg(os.getpgid(sim_proc.pid), signal.SIGTERM)
    except Exception:
        pass
    time.sleep(2)
    kill_sim_processes()

    # 7. Read results
    if not os.path.exists(sim_odom_path):
        print('[ERROR] No sim odom data produced!')
        return 1000.0

    return sim_odom_path


# ===================================================================
# ERROR COMPUTATION & PLOTTING
# ===================================================================
def compute_error(real_data, sim_odom_path):
    """Compute RMSE between real and sim odometry."""
    sim = np.load(sim_odom_path)
    sim_t = sim['odom_times']
    sim_x = sim['odom_x']
    sim_y = sim['odom_y']
    sim_yaw = sim['odom_yaw']

    if len(sim_t) < 10:
        print('[WARN] Too few sim odom samples')
        return 1000.0, sim

    real_t = real_data['odom_times']
    real_x = real_data['odom_x']
    real_y = real_data['odom_y']
    real_yaw = real_data['odom_yaw']

    # Interpolate sim to real timestamps
    t_min = max(sim_t[0], real_t[0])
    t_max = min(sim_t[-1], real_t[-1])
    if t_max <= t_min:
        print('[WARN] No time overlap between sim and real odom')
        return 1000.0, sim

    mask = (real_t >= t_min) & (real_t <= t_max)
    eval_t = real_t[mask]
    if len(eval_t) < 5:
        return 1000.0, sim

    fx = interp1d(sim_t, sim_x, kind='linear', fill_value='extrapolate')
    fy = interp1d(sim_t, sim_y, kind='linear', fill_value='extrapolate')
    fyaw = interp1d(sim_t, sim_yaw, kind='linear', fill_value='extrapolate')

    sim_x_i = fx(eval_t)
    sim_y_i = fy(eval_t)
    sim_yaw_i = fyaw(eval_t)
    real_x_i = real_x[mask]
    real_y_i = real_y[mask]
    real_yaw_i = real_yaw[mask]

    pos_err = np.sqrt((sim_x_i - real_x_i)**2 + (sim_y_i - real_y_i)**2)
    yaw_err = np.abs(np.arctan2(np.sin(sim_yaw_i - real_yaw_i),
                                 np.cos(sim_yaw_i - real_yaw_i)))

    pos_rmse = np.sqrt(np.mean(pos_err**2))
    yaw_rmse = np.sqrt(np.mean(yaw_err**2))
    total = pos_rmse + 0.3 * yaw_rmse

    print(f'[RESULT] pos_RMSE={pos_rmse:.4f}m, yaw_RMSE={yaw_rmse:.4f}rad, '
          f'total={total:.4f}')
    return total, sim


def plot_comparison(epoch_num, params, error, real_data, sim_data):
    """Generate 4-panel comparison plot."""
    fig, axes = plt.subplots(2, 2, figsize=(14, 10))

    real_t = real_data['odom_times']
    sim_t = sim_data['odom_times']

    # XY trajectory
    ax = axes[0, 0]
    ax.plot(real_data['odom_x'], real_data['odom_y'], 'b-', label='Real', linewidth=1.5)
    ax.plot(sim_data['odom_x'], sim_data['odom_y'], 'r--', label='Sim', linewidth=1.5)
    ax.set_xlabel('X (m)')
    ax.set_ylabel('Y (m)')
    ax.set_title('X-Y Trajectory')
    ax.legend()
    ax.set_aspect('equal')
    ax.grid(True, alpha=0.3)

    # X vs time
    ax = axes[0, 1]
    ax.plot(real_t, real_data['odom_x'], 'b-', label='Real', linewidth=1)
    ax.plot(sim_t, sim_data['odom_x'], 'r--', label='Sim', linewidth=1)
    ax.set_xlabel('Time (s)')
    ax.set_ylabel('X (m)')
    ax.set_title('X vs Time')
    ax.legend()
    ax.grid(True, alpha=0.3)

    # Y vs time
    ax = axes[1, 0]
    ax.plot(real_t, real_data['odom_y'], 'b-', label='Real', linewidth=1)
    ax.plot(sim_t, sim_data['odom_y'], 'r--', label='Sim', linewidth=1)
    ax.set_xlabel('Time (s)')
    ax.set_ylabel('Y (m)')
    ax.set_title('Y vs Time')
    ax.legend()
    ax.grid(True, alpha=0.3)

    # Yaw vs time
    ax = axes[1, 1]
    ax.plot(real_t, real_data['odom_yaw'], 'b-', label='Real', linewidth=1)
    ax.plot(sim_t, sim_data['odom_yaw'], 'r--', label='Sim', linewidth=1)
    ax.set_xlabel('Time (s)')
    ax.set_ylabel('Yaw (rad)')
    ax.set_title('Yaw vs Time')
    ax.legend()
    ax.grid(True, alpha=0.3)

    param_str = ' | '.join([f'{k}={v:.4f}' for k, v in params.items()])
    fig.suptitle(f'Epoch {epoch_num} | RMSE={error:.4f} | {param_str}',
                 fontsize=10, y=0.98)
    plt.tight_layout(rect=[0, 0, 1, 0.96])

    plot_path = os.path.join(OUTPUT_DIR, f'epoch_{epoch_num:03d}_plot.png')
    plt.savefig(plot_path, dpi=120)
    plt.close()
    print(f'[INFO] Plot saved: {plot_path}')


# ===================================================================
# MAIN OPTIMIZER
# ===================================================================
def main():
    from skopt import gp_minimize
    from skopt.space import Real

    os.makedirs(OUTPUT_DIR, exist_ok=True)

    # Backup original files
    xacro_orig = os.path.join(INSTALL_DESC, 'robot_core.xacro')
    yaml_orig = os.path.join(INSTALL_CFG, 'diff_drive_controller.yaml')
    xacro_backup = xacro_orig + '.optimizer_backup'
    yaml_backup = yaml_orig + '.optimizer_backup'

    if not os.path.exists(xacro_backup):
        shutil.copy2(xacro_orig, xacro_backup)
    if not os.path.exists(yaml_backup):
        shutil.copy2(yaml_orig, yaml_backup)

    # Set world RTF
    set_world_rtf(REAL_TIME_FACTOR)

    # Extract bag data
    bag_data_path = os.path.join(OUTPUT_DIR, 'bag_data.npz')
    extract_bag_data(BAG_PATH, bag_data_path)
    real_data = np.load(bag_data_path)
    bag_duration = float(real_data['cmd_vel_times'][-1])
    print(f'[INFO] Bag duration: {bag_duration:.2f}s')

    # Kill any leftover sim processes
    kill_sim_processes()

    # Track results
    all_results = []
    epoch_counter = [0]

    def objective(param_values):
        epoch_counter[0] += 1
        epoch = epoch_counter[0]

        params = dict(zip(PARAM_NAMES, param_values))
        result = run_epoch(epoch, params, bag_data_path, bag_duration)

        if isinstance(result, float):
            # Error case
            all_results.append((epoch, params.copy(), result))
            with open(os.path.join(OUTPUT_DIR, f'epoch_{epoch:03d}_params.txt'), 'w') as f:
                f.write(f'RMSE: {result}\n')
                for k, v in params.items():
                    f.write(f'{k}: {v:.6f}\n')
            return result

        sim_odom_path = result
        error, sim = compute_error(real_data, sim_odom_path)
        all_results.append((epoch, params.copy(), error))

        if error < 999:
            plot_comparison(epoch, params, error, real_data, sim)

        with open(os.path.join(OUTPUT_DIR, f'epoch_{epoch:03d}_params.txt'), 'w') as f:
            f.write(f'RMSE: {error:.6f}\n')
            for k, v in params.items():
                f.write(f'{k}: {v:.6f}\n')

        return error

    # Run Bayesian optimization
    print(f'\n{"#"*60}')
    print(f'# STARTING BAYESIAN OPTIMIZATION ({N_EPOCHS} epochs)')
    print(f'# Parameters: {PARAM_NAMES}')
    print(f'# Bounds: {PARAM_DIMENSIONS}')
    print(f'{"#"*60}\n')

    space = [Real(low, high, name=name)
             for name, (low, high) in PARAM_BOUNDS.items()]

    try:
        result = gp_minimize(
            objective,
            space,
            n_calls=N_EPOCHS,
            n_initial_points=min(5, N_EPOCHS),
            random_state=42,
            verbose=True
        )

        # Report
        print(f'\n{"="*60}')
        print('OPTIMIZATION COMPLETE')
        print(f'{"="*60}')
        best_params = dict(zip(PARAM_NAMES, result.x))
        print(f'Best RMSE: {result.fun:.6f}')
        print(f'Best parameters:')
        for k, v in best_params.items():
            print(f'  {k}: {v:.6f}')

        # Save summary
        with open(os.path.join(OUTPUT_DIR, 'optimization_results.txt'), 'w') as f:
            f.write(f'Best RMSE: {result.fun:.6f}\n\n')
            f.write('Best parameters:\n')
            for k, v in best_params.items():
                f.write(f'  {k}: {v:.6f}\n')
            f.write(f'\nAll epochs:\n')
            for epoch, params, error in all_results:
                p_str = ', '.join([f'{k}={v:.4f}' for k, v in params.items()])
                f.write(f'  Epoch {epoch}: RMSE={error:.6f} | {p_str}\n')

        print(f'\nResults saved to {OUTPUT_DIR}/')

    except KeyboardInterrupt:
        print('\n[INFO] Optimization interrupted by user.')
    finally:
        # Restore original files
        print('[INFO] Restoring original files...')
        if os.path.exists(xacro_backup):
            shutil.copy2(xacro_backup, xacro_orig)
        if os.path.exists(yaml_backup):
            shutil.copy2(yaml_backup, yaml_orig)
        kill_sim_processes()
        print('[INFO] Done.')


if __name__ == '__main__':
    main()
