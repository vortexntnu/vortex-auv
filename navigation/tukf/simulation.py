from rosbags.highlevel import AnyReader
from rosbags.typesys import get_types_from_msg, register_types
import numpy as np, pandas as pd, pathlib
from tukf import TUKF
import tukf_class as ukf

bag_folder = pathlib.Path(r"/home/talha/vortex_auv_ws/bags/sim_data_no_dvl")

# Initialize UKF with StateQuat
initial_position = np.array([0.0, 0.0, 0.0])  # x, y, z
initial_velocity = np.array([0.0, 0.0, 0.0])  # vx, vy, vz
initial_quaternion = np.array([0.0, 0.0, 0.0])  # w, x, y, z (identity quaternion)
initial_angular_velocity = np.array([0.0, 0.0, 0.0])  # wx, wy, wz
initial_g_eta = np.array([0.01, 0.01, 0.01, 0.01])  # g_eta parameters
initial_intertia = np.array([0.2, 0.2, 0.1, 
                            0.2, 0.2, 0.2, 
                            0.1, 0.2, 0.2])
initial_damping = np.array([0.01, 0.01, 0.01,
                            0.01, 0.01, 0.01])
initla_added_mass = np.array([0.02, 0.02, 0.02,
                             0.02, 0.02, 0.02])

p_diag = np.concatenate([
    2*np.ones(3),  # x position
    2*np.ones(3),  # orientation
    2*np.ones(3),  # velocity
    2*np.ones(3),  # angular velocity
    2*np.ones(9),             # inertia
    2*np.ones(6),             # added mass
    2*np.ones(6),             # damping
    2*np.ones(4)             # g_eta
])

initial_covariance = np.diag(p_diag) 

state = ukf.AUVState(initial_position.copy(), initial_quaternion.copy(), initial_velocity.copy(), initial_angular_velocity.copy(), 
                      initial_intertia.copy(), initla_added_mass.copy(), initial_damping.copy(), initial_g_eta.copy())

state.covariance = initial_covariance.copy()

Q_diag = np.concatenate([
    0.1*np.ones(3),      # position
    0.1*np.ones(9),     # kinematic (η & ν)
    0.001*np.ones(9),      # inertia
    0.001*np.ones(6),      # added mass
    0.001*np.ones(6),      # damping
    0.001*np.ones(4),        # g_eta
])

UKF_model = TUKF(state, np.diag(Q_diag))  # Process noise covariance

def dvl_h(state: ukf.AUVState) -> 'ukf.MeasModel':
    H_matrix = np.zeros((3, 12))
    H_matrix[:, 6:9] = np.eye(3)
    z_i = ukf.MeasModel()
    z_i.measurement = np.dot(H_matrix, state.dynamic_part())
    return z_i

dvl_measurement = ukf.MeasModel(H=dvl_h)

def ang_h(state: ukf.AUVState) -> 'ukf.MeasModel':
    H_matrix = np.zeros((3, 12))
    H_matrix[:, 9:12] = np.eye(3)
    z_i = ukf.MeasModel()
    z_i.measurement = np.dot(H_matrix, state.dynamic_part())
    return z_i

ang_measurement = ukf.MeasModel(H=ang_h) 

R_corr = np.array([[0.0, 0.0, -1.0],
                    [0.0, -1.0, 0.0],
                    [-1.0, 0.0, 0.0]])

# Storage for trajectory
positions = []
velocities = []
quaternions = []
angular_velocities = []
okid_params = []

# Storage for trajectory estimates
positions_est = []
velocities_est = []
quaternions_est = []
angular_velocities_est = []
okid_params_est = []

with AnyReader([bag_folder]) as reader:
    # Filter topics once
    conns = [c for c in reader.connections
             if c.topic in ("/imu/data_raw", "/dvl_twist", "/orca/wrench_input")]

    last_time = None
    log       = []
    coutner = 0

    for conn, ts_raw, raw in reader.messages(conns):
        t_ns = ts_raw  # already nanoseconds integer
        coutner += 1

        if conn.topic == "/orca/wrench_input":
            
            # Get the wrench input message
            msg = reader.deserialize(raw, conn.msgtype)
            wrench = msg.wrench  # Extract the wrench field from the message
            forces = np.array([wrench.force.x, wrench.force.y, wrench.force.z])
            torques = np.array([wrench.torque.x, wrench.torque.y, wrench.torque.z])

            control_input = np.concatenate([forces, torques])

            if last_time is not None:
                UKF_model.dt = (t_ns - last_time) / 1e9  # Convert nanoseconds to seconds

            # 1. prediction step
            state = UKF_model.unscented_transform(state, control_input)

        # 2. measurement update
        msg = reader.deserialize(raw, conn.msgtype)
        if conn.topic == "/imu/data_raw":
            print("IMU data received")

            # Get the IMU data
            measurement_imu = np.array([msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z])

            ang_measurement.measurement = np.dot(R_corr,measurement_imu)

            ang_measurement.covariance = np.eye(3) * (0.03**2) 

            UKF_model.measurement_update(state, ang_measurement)
            state = UKF_model.posteriori_estimate(state, ang_measurement)

        if conn.topic == "/dvl_twist":
            print("DVL data received")

            msg = reader.deserialize(raw, conn.msgtype)
            dvl_measurement.measurement = np.array([msg.twist.twist.linear.x, msg.twist.twist.linear.y, msg.twist.twist.linear.z])
            dvl_measurement.measurement = np.dot(R_corr, dvl_measurement.measurement)
            
            dvl_measurement.covariance = np.eye(3) * (0.01**2) 
            
            # Update UKF with measurement
            UKF_model.measurement_update(state, dvl_measurement)
            state = UKF_model.posteriori_estimate(state, dvl_measurement)


        # Store the state estimates
        positions_est.append(state.position.copy())
        velocities_est.append(state.velocity.copy())
        quaternions_est.append(state.quaternion.copy())
        angular_velocities_est.append(state.angular_velocity.copy())
        okid_params_est.append(state.okid_part().copy())

        last_time = t_ns

        