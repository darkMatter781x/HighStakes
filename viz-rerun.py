import rerun as rr  # NOTE: `rerun`, not `rerun-sdk`!
import numpy as np

print("running rr.init()")
rr.init("rerun_example_my_data", spawn=False, )
rr.connect_tcp()
print("ran rr.connect_tcp()")

positions = np.zeros((10, 3))
positions[:, 0] = np.linspace(-10, 10, 10)

colors = np.zeros((10, 3), dtype=np.uint8)
colors[:, 0] = np.linspace(0, 255, 10)

rr.log("my_points", rr.Points3D(positions, colors=colors, radii=0.5))

print("done")
