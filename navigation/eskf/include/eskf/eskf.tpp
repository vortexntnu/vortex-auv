

template <SensorModelConcept SensorT>
void ESKF::measurement_update(const SensorT& meas)
{
    Eigen::VectorXd innovation = meas.innovation(current_nom_state_);
    Eigen::MatrixXd H = meas.jacobian(current_nom_state_);
    Eigen::MatrixXd R = meas.noise_covariance();

    Eigen::MatrixXd P = current_error_state_.covariance;

    Eigen::MatrixXd S = H * P * H.transpose() + R;
    Eigen::MatrixXd K = P * H.transpose() * S.inverse();

    #ifndef NDEBUG
    nis_ = compute_nis(innovation, S);
    #endif

    current_error_state_.set_from_vector(K * innovation);

    Eigen::MatrixXd I_KH = Eigen::MatrixXd::Identity(P.rows(), P.cols()) - K * H;
    current_error_state_.covariance =
        I_KH * P * I_KH.transpose() +
        K * R * K.transpose();  // Used joseph form for more stable calculations
}
