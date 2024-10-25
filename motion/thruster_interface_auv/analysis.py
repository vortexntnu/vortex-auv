import matplotlib.pyplot as plt
import numpy as np

# --------------------------------
# load data and transform to numpy

files = [
    "motion/thruster_interface_auv/resources/T200-Thrusters-10V.csv",
    "motion/thruster_interface_auv/resources/T200-Thrusters-12V.csv",
    "motion/thruster_interface_auv/resources/T200-Thrusters-14V.csv",
    "motion/thruster_interface_auv/resources/T200-Thrusters-16V.csv",
    "motion/thruster_interface_auv/resources/T200-Thrusters-18V.csv",
    "motion/thruster_interface_auv/resources/T200-Thrusters-20V.csv",
]
data = []
for f in files:
    data = np.genfromtxt(f, delimiter=",", skip_header=1, usecols=(5, 0))
    data.append(data)


# --------------------------------
# plot pwm values vs force
def plot_csvs(data):
    for i, single_data in enumerate(data):
        plt.plot(single_data[:, 0], single_data[:, 1], label=i * 2 + 10)

    plt.xlabel("force")
    plt.ylabel("pwm")
    plt.legend()
    plt.show()


# plot_csvs(data)

# --------------------------------
# calculate the two halves deg-order poly approx and plot them on data


def plot_single_vs_poly(xdata, ydata, deg):
    zero_indices = np.where(xdata == 0.00)[0]
    begin = zero_indices[0]
    end = zero_indices[-1]

    xL = xdata[:begin]
    xR = xdata[end:]
    yL = ydata[:begin]
    yR = ydata[end:]

    coeffs_L = np.polyfit(xL, yL, deg)
    print("LEFT:", coeffs_L)
    coeffs_R = np.polyfit(xR, yR, deg)
    print("RIGHT:", coeffs_R)
    polyL = np.poly1d(coeffs_L)
    polyR = np.poly1d(coeffs_R)

    yL_fit = polyL(xL)
    yR_fit = polyR(xR)

    rms = np.sqrt(np.mean((yL - yL_fit) ** 2)) + np.sqrt(np.mean((yR - yR_fit) ** 2))

    plt.scatter(xdata, ydata, s=2)
    plt.plot(xL, yL_fit, color="red")
    plt.plot(xR, yR_fit, color="red")
    plt.title(f"deg: {deg} rms {rms}")

    plt.xlabel("force")
    plt.ylabel("pwm")
    plt.show()

    return coeffs_L, coeffs_R


np.set_printoptions(precision=5, suppress=True)
for single_data in data:
    plot_single_vs_poly(single_data[:, 0], single_data[:, 1], 3)
