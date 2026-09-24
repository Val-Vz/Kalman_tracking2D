import numpy as np
import matplotlib.pyplot as plt

TOTAL_TIME = 10 * 60 
DT = 0.05          

def turn_rate_from_g(n_g, v, g=9.81):
    """
    Turn rate (rad/s) for a coordinated turn with a load factor 
    n_g (e.g., n_g=6 -> 6g turn) at speed v (m/s).
    omega = g * sqrt(n_g^2 - 1) / v
    """
    n_g = max(n_g, 1.0001)
    return g * np.sqrt(n_g**2 - 1.0) / v


def _integrate_segment(state, dt, duration, a_tan=0.0, n_g=None, turn_sign=1.0, g=9.81):
    """
    Integrates a flight segment with constant control inputs.

    state = [x, y, v, heading]   (v = speed, heading in rad)

    If n_g is provided: coordinated turn targeting this load factor; the
    turn rate omega is recalculated at EACH step from the current speed
    (therefore remaining physically consistent even if a_tan changes v
    during the turn). turn_sign = +1 (right turn) or -1 (left turn).
    If n_g is None: straight flight (omega = 0).
    """
    n = max(int(round(duration / dt)), 1)
    x, y, v, heading = state
    pos = np.zeros((n, 2))
    vel = np.zeros((n, 2))
    acc = np.zeros((n, 2))

    for k in range(n):
        omega = turn_sign * turn_rate_from_g(n_g, max(v, 1.0), g) if n_g is not None else 0.0

        # Tangential and centripetal components of acceleration
        ax = a_tan * np.cos(heading) - v * omega * np.sin(heading)
        ay = a_tan * np.sin(heading) + v * omega * np.cos(heading)

        acc[k] = (ax, ay)
        vel[k] = (v * np.cos(heading), v * np.sin(heading))
        pos[k] = (x, y)

        # Explicit Euler method (dt small compared to the time constants => negligible error)
        v       += a_tan * dt
        heading += omega * dt
        x       += vel[k, 0] * dt
        y       += vel[k, 1] * dt

    return pos, vel, acc, [x, y, v, heading]

def build_trajectory(segments, dt=DT, x0=0.0, y0=0.0, v0= 120.0 * 1852/3600, heading0 = 45 * np.pi / 180, g=9.81):
    """
    Builds a trajectory by chaining a list of segments (dicts).
    Each segment: {'duration': ..., 'a_tan': ..., 'n_g': ..., 'turn_sign': ...}
    (only 'duration' is required; the others have default values).

    Returns t (N,), pos (N,2), vel (N,2), acc (N,2)
    """
    state = [x0, y0, v0, heading0]
    pos_all, vel_all, acc_all = [], [], []

    for seg in segments:
        seg = dict(seg)
        duration = seg.pop("duration")
        pos, vel, acc, state = _integrate_segment(state, dt, duration, g=g, **seg)
        pos_all.append(pos)
        vel_all.append(vel)
        acc_all.append(acc)

    pos = np.vstack(pos_all)
    vel = np.vstack(vel_all)
    acc = np.vstack(acc_all)
    t = np.arange(pos.shape[0]) * dt
    return t, pos, vel, acc

#Light aircraft
def aircraft_1(dt=DT):
    """
    Straight-line motion at constant speed (zero acceleration).
    """
    segments = [
        dict(duration=TOTAL_TIME),   # rien ne change : a_tan=0, pas de virage
    ]
    return build_trajectory(segments, dt=dt)

#Commercial aircraft
def aircraft_2(dt=DT):
    """
    One gentle turn (~1.15g), separated by straight-line segments. Accelerating in straight line, then decelerating in straight line.
    """
    segments = [
        dict(duration=120., a_tan=0.10),                  # accelerating straight line
        dict(duration=360.0, n_g=1.15, turn_sign=+1),    # gentle right turn
        dict(duration=120., a_tan=-0.10),                 # decelerating straight line
    ]
    return build_trajectory(segments, dt=dt, v0=120.0)

#Fighter aircraft
def aircraft_3(dt=DT):
    """
    more abrupt and unpredictable maneuvers
    """
    segments = [
        dict(duration=60.0),                              # straight flight

        dict(duration=30.0, a_tan=0.15),                 # strong acceleration

        dict(duration=45.0, n_g=3.0, turn_sign=+1),      # high-G right turn
        dict(duration=30.0, n_g=4.0, turn_sign=+1),      # stronger right turn

        dict(duration=30.0, a_tan=-0.10),                # deceleration
        dict(duration=45.0, n_g=5.0, turn_sign=-1),      # high-G left turn

        dict(duration=30.0, a_tan=0.20),                 # strong acceleration

        dict(duration=40.0, n_g=6.0, turn_sign=+1),      # very high-G right turn

        dict(duration=30.0, a_tan=-0.15),                # strong deceleration

        dict(duration=40.0, n_g=5.0, turn_sign=-1),      # high-G left turn

        dict(duration=30.0, a_tan=0.20),                 # strong acceleration

        dict(duration=40.0, n_g=6.0, turn_sign=+1),      # very high-G right turn

        dict(duration=30.0, n_g=4.0, turn_sign=-1),      # high-G left turn

        dict(duration=60.0),                              # straight flight

        dict(duration=30.0, a_tan=-0.10),                # deceleration

        dict(duration=30.0),                              # straight flight
    ]
    return build_trajectory(segments, dt=dt)




#To plot the trajectory of aircraft and the evolution of its state
if __name__ == "__main__":

    # Options: True = display, False = hide
    show_aircraft_1 = True
    show_aircraft_2 = True
    show_aircraft_3 = True

    fig, axs = plt.subplots(3, 3, figsize=(15, 12))

    aircrafts = [
        (show_aircraft_1, aircraft_1, "Aircraft 1"),
        (show_aircraft_2, aircraft_2, "Aircraft 2"),
        (show_aircraft_3, aircraft_3, "Aircraft 3"),
    ]

    for row, (show, aircraft, name) in enumerate(aircrafts):

        if not show:
            # Hide the three subplots of this aircraft
            for ax in axs[row]:
                ax.set_visible(False)
            continue

        # Simulation
        t, pos, vel, acc = aircraft(dt=DT)

        # Unit conversions
        pos_nm = pos / 1852.0
        speed_kt = np.linalg.norm(vel, axis=1) * 3600.0 / 1852.0

        # 1 - Trajectory
        axs[row, 0].plot(pos_nm[:, 0], pos_nm[:, 1])
        axs[row, 0].set_title(f"{name} - Trajectory")
        axs[row, 0].set_xlabel("x (NM)")
        axs[row, 0].set_ylabel("y (NM)")
        axs[row, 0].axis("equal")

        # 2 - Speed
        axs[row, 1].plot(t, speed_kt)
        axs[row, 1].set_title(f"{name} - Speed")
        axs[row, 1].set_xlabel("t (s)")
        axs[row, 1].set_ylabel("Speed (kt)")

        # 3 - Acceleration
        axs[row, 2].plot(t, acc[:, 0], label="ax")
        axs[row, 2].plot(t, acc[:, 1], label="ay")
        axs[row, 2].set_title(f"{name} - Acceleration")
        axs[row, 2].set_xlabel("t (s)")
        axs[row, 2].set_ylabel("Acceleration (m/s²)")
        axs[row, 2].legend()

    for ax in axs.flat:
        ax.grid(True)
    
    plt.tight_layout()
    plt.show()

    """
    plt.tight_layout()
    plt.savefig("aircraft_1_trajectory.png", dpi=150)
    print("Figure enregistrée : aircraft_1_trajectory.png")
    """

