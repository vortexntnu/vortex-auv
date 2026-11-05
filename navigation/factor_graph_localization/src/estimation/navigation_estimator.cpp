// TODO: Select the IMU pose, STARTUP-expressed IMU-origin velocity, and IMU bias
// keys at each accepted beam's timestamp. Pass the corresponding gyro sample
// with estimated bias still present to build_dvl_beam_factor, then insert it.

// TODO: Estimate one shared IMU_P_DVL across beams and timestamps with a
// configured calibration prior, preserving correlated mounting uncertainty.
