import numpy as np
import matplotlib.pyplot as plt

from scenarios import aircraft_3


def GNSS_measurements(pos, sigma_gnss=2.0):
    """
    Simulate GNSS measurements with Gaussian noise.

    Parameters:
    pos : ndarray
        True position of the aircraft (shape: [N, 2]).
    sigma_gnss : float
        Standard deviation of the GNSS noise.

    Returns:
    ndarray
        Noisy GNSS measurements (shape: [N, 2]).
    """
    noise = np.random.normal(0, sigma_gnss, pos.shape)
    return pos + noise

def radar_measurements(pos, sigma_range=100.0, sigma_azimuth=0.1):
    """
    Simulate radar measurements with Gaussian noise.

    Parameters:
    pos : ndarray
        True position of the aircraft (shape: [N, 2]).
    sigma_range : float
        Standard deviation of the range noise.
    sigma_azimuth : float
        Standard deviation of the azimuth noise.

    Returns:
    tuple of ndarray
        range1, azimuth1, range2, azimuth2,
        each of shape (N,)
    """
    sigma_azimuth = np.deg2rad(sigma_azimuth)  # Convert azimuth noise from degrees to radians
    position_radar1 = np.array([-10 * 1852 / np.sqrt(2), -10 * 1852 / np.sqrt(2)])  # Assuming radar is 10 NM away from the left bottom corner of the area
    position_radar2 = np.array([10 * 1852 / np.sqrt(2), -10 * 1852 / np.sqrt(2)])  # Assuming radar is 10 NM away from the right bottom corner of the area
    range1 = np.sqrt((pos[:, 0] - position_radar1[0])**2 + (pos[:, 1] - position_radar1[1])**2)
    range2 = np.sqrt((pos[:, 0] - position_radar2[0])**2 + (pos[:, 1] - position_radar2[1])**2)
    range_noise1 = np.random.normal(0, sigma_range, pos.shape[0])
    range_noise2 = np.random.normal(0, sigma_range, pos.shape[0])
    azimuth1 = np.arctan2(pos[:, 0] - position_radar1[0], pos[:, 1] - position_radar1[1])
    azimuth2 = 2*np.pi - np.arctan2(position_radar2[0] - pos[:, 0], pos[:, 1] - position_radar2[1])
    azimuth_noise1 = np.random.normal(0, sigma_azimuth, pos.shape[0])
    azimuth_noise2 = np.random.normal(0, sigma_azimuth, pos.shape[0])
    return range1 + range_noise1, azimuth1 + azimuth_noise1, range2 + range_noise2, azimuth2 + azimuth_noise2


if __name__ == "__main__":
    t, pos, vel, acc = aircraft_3()

    gnss_measurements = GNSS_measurements(pos)

    range1, azimuth1, range2, azimuth2 = radar_measurements(pos)

    fig = plt.figure(figsize=(18, 7))

    # Trajectory plot on the left
    ax_xy = plt.subplot(1, 3, 1)
    ax_xy.plot( pos[:, 0] / 1852, pos[:, 1] / 1852,label="True trajectory")
    ax_xy.scatter(gnss_measurements[:, 0] / 1852, gnss_measurements[:, 1] / 1852, c="red", s=10, alpha=0.5, label="GNSS bruité")
    ax_xy.set_title("Plane trajectory and GNSS measurements")
    ax_xy.set_xlabel("x (nm)")
    ax_xy.set_ylabel("y (nm)")
    ax_xy.grid(True)
    ax_xy.axis("equal")
    ax_xy.legend()

    # First radar measurements in the middle
    ax_radar1 = plt.subplot(1, 3, 2)
    ax_radar1.plot(range1/1852, azimuth1*180/np.pi, label="Radar 1")
    ax_radar1.set_title("Radar 1 - Azimuth vs Range")
    ax_radar1.set_xlabel("Range (nm)")
    ax_radar1.set_ylabel("Azimuth (deg)")
    ax_radar1.grid(True)
    ax_radar1.legend()
    # Second radar measurements on the right
    ax_radar2 = plt.subplot(1, 3, 3)
    ax_radar2.plot(range2/1852, azimuth2*180/np.pi, label="Radar 2")
    ax_radar2.set_title("Radar 2 - Azimuth vs Range")
    ax_radar2.set_xlabel("Range (nm)")
    ax_radar2.set_ylabel("Azimuth (deg)")
    ax_radar2.grid(True)
    ax_radar2.legend()


    #plt.savefig("figures/noisy_measurements.png", dpi=300, bbox_inches="tight")
    plt.tight_layout()
    plt.show()