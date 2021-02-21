#include "mcu_interface_new/interp.h"

#include <vector>

// Returns interpolated value at x from parallel arrays ( x_data, y_data )
//   Assumes that x_data has at least two elements, is sorted and is strictly monotonic increasing
//   boolean argument extrapolate determines behaviour beyond ends of array (if needed)
/**
 *  Returns interpolated value at x
*/
double interpolate(double x, std::vector<double> &x_data, std::vector<double> &y_data)
{
    int size = x_data.size();

    // Find left end of interval for interpolation 
    int i = 0;  
                                              
    if (x >= x_data[size - 2]) { // Special case, supplied point is beyond right end
        i = size - 2;
    }
    else { // Normal case
        while (x > x_data[i+1]) i++;
    }

    double xL = x_data[i];
    double yL = y_data[i];
    double xR = x_data[i+1];
    double yR = y_data[i+1];      // points on either side (unless beyond ends)
    double dydx = ( yR - yL ) / ( xR - xL ); //grad

    return yL + dydx * ( x - xL );                                              // linear interpolation
}