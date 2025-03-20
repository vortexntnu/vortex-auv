def mean_set(set_points: list[StateQuat], weights: np.ndarray = None) -> np.ndarray:
    """
    Function that calculates the mean of a set of points
    """
    n = len(set_points[0].as_vector()) - 1
    mean_value = StateQuat()

    if weights is None:
        for i in range(2 * n + 1):
            weight_temp_list = (1/ (2 * n + 1)) * np.ones(2 * n + 1)
            mean_value.add_without_quaternions(weight_temp_list[i] * set_points[i])
        
        mean_value.orientation = iterative_quaternion_mean_statequat(set_points, weight_temp_list)
    
    else:
        for i in range(2 * n + 1):
            mean_value.add_without_quaternions(weights[i] * set_points[i])

        mean_value.orientation = iterative_quaternion_mean_statequat(set_points, weights)
    
    return mean_value.as_vector()

def mean_measurement(set_points: list[MeasModel], weights: np.ndarray = None) -> np.ndarray:
    """
    Function that calculates the mean of a set of points
    """
    n = len(set_points)
    mean_value = MeasModel()

    if weights is None:
        for i in range(n):
            mean_value = mean_value + set_points[i]
    else:
        for i in range(n):
            mean_value = mean_value + (weights[i] * set_points[i])
    
    return mean_value.measurement