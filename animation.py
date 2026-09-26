
"""Animate the 3 scenarios from `scenarios.py` over 60 seconds with 
a plane (a triangle oriented according to its heading) moves along 
each trajectory, leaving a trail that is drawn behind it.
"""


import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from matplotlib.patches import Polygon
from tqdm import tqdm

from scenarios import aircraft_1, aircraft_2, aircraft_3, DT

def _aircraft_shape(x, y, heading, size):
    """Vertices of a small triangle pointing in the direction of heading."""
    pts = np.array([
        [ size,       0.0],
        [-0.6 * size,  0.5 * size],
        [-0.6 * size, -0.5 * size],
    ])
    c, s = np.cos(heading), np.sin(heading)
    rot = np.array([[c, -s], [s, c]])
    return pts @ rot.T + [x, y]

def make_animation(save_path="trajectories_animation.gif"):
    titles = ["Light Aircraft", "Commercial Aircraft", "Fighter Jet"]
    data = [aircraft_1(), aircraft_2(), aircraft_3()]   # (t, pos, vel, acc)

    fig, axes = plt.subplots(1, 3, figsize=(5.5 * 3, 5))
    time_label = fig.text(0.98, 0.98 ,"" , ha="right", va="top")

    panels = []
    for (t, pos, vel, acc), ax, title in zip(data, axes, titles):
        pos_nm = pos / 1852.0
        span = max(np.ptp(pos_nm[:, 0]), np.ptp(pos_nm[:, 1]), 0.1) #span determines the size of the display area
        margin = 0.15 * span #we take 15% of the span as a margin to ensure that the aircraft is not too close to the edge of the display area

        ax.set_xlim(pos_nm[:, 0].min() - margin, pos_nm[:, 0].max() + margin) #definition of the x-axis limits of the display area
        ax.set_ylim(pos_nm[:, 1].min() - margin, pos_nm[:, 1].max() + margin) # // y-axis limits //
        ax.set_aspect("equal")
        ax.set_title(title)
        ax.set_xlabel("x (nm)")
        ax.set_ylabel("y (nm)")
        ax.plot(pos_nm[:, 0], pos_nm[:, 1], "--", color="lightgray", lw=1, zorder=1)

        trail, = ax.plot([], [], "-", color="tab:blue", lw=2, zorder=2)
        size = 0.05 * span   #aircraft is 5% of the span of the display area
        plane = Polygon(_aircraft_shape(pos_nm[0, 0], pos_nm[0, 1], 0.0, size),
                         closed=True, color="tab:red", zorder=3)
        ax.add_patch(plane)
        #time_label = ax.text(0.02, 0.96, "", transform=ax.transAxes, va="top")

        panels.append((pos_nm, vel, trail, plane, size))

    n_frames = min(len(pos_nm) for pos_nm, *_ in panels)

    def update(frame):
        artists = []
        for pos_nm, vel, trail, plane, size in panels:
            trail.set_data(pos_nm[:frame + 1, 0], pos_nm[:frame + 1, 1])
            heading = np.arctan2(vel[frame, 1], vel[frame, 0])
            plane.set_xy(_aircraft_shape(pos_nm[frame, 0], pos_nm[frame, 1], heading, size))
            artists.extend([trail, plane])
        time_label.set_text(f"t = {frame * DT:4.1f} s")
        artists.append(time_label)  
        return artists

    fig.suptitle("Simulated trajectories over 60 seconds", fontsize=16)
    fig.tight_layout()

    ani = animation.FuncAnimation(fig, update, frames=n_frames,
                                   interval=DT * 1000, blit=False)
    with tqdm(total=n_frames, desc="Creation of the GIF") as progress: #allows to display a progress bar during the GIF creation
            ani.save(
                save_path,
                writer="pillow",
                fps=int(round(1 / DT)),
                progress_callback=lambda i, n: progress.update(1)
            )
    
    print(f"GIF done : {save_path}")
    print(f"Animation saved : {save_path}")
    return ani

if __name__ == "__main__":
    make_animation()
    plt.show()