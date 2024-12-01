#!/usr/bin/env python3
import os
import signal
import threading
import matplotlib
from matplotlib import animation
import matplotlib.pyplot as plt
import numpy as np
from matplotlib.widgets import CheckButtons, Slider
import subprocess
import atexit
"""
To set up run: 

sudo apt-get install python3-tk 
pip install matplotlib --break-system
sudo apt-get install python3-pyqt6
"""

matplotlib.use('QtAgg')

fig, scat_ax = plt.subplots()

scat_ax.set_xlim(-72,72)
scat_ax.set_ylim(-72,72)

iter_slider_axis = plt.axes([0.1, 0.01, 0.65, 0.03])
iter_slider = Slider(
    iter_slider_axis, 'Iteration', 0, 10, valinit=0., valstep=1., animated=True
)

particle_scat = scat_ax.scatter(np.array(0)*500, np.array(0)*500, s=np.array(.075)*500, animated=True)
mean_scat = scat_ax.scatter(0, 0, s=1, c='red', animated=True)

if subprocess.run("./scripts/build-sim.sh", shell=True).returncode != 0:
    print("Error building monolith, exiting...")
    exit(1)
else:
    print("Monolith built successfully!")

checkboxes_ax = plt.axes([0.05, 0.4, 0.1, 0.15])
checkboxes = CheckButtons(checkboxes_ax, ['Show Real Time', 'Show Particles'], [True, False])
def get_checkbox_status(label):
    return checkboxes.get_status()[
        next((i for i, textObj in enumerate(checkboxes.labels) if textObj.get_text() == label))
    ]

# When enabled, the visualizer will update in real time
def get_real_time_status():
    return get_checkbox_status('Show Real Time')
def get_particles_status():
    return get_checkbox_status('Show Particles')

def update_plot(_=None, from_slider=False):
    real_time = get_real_time_status()
    iter_slider.set_active(not real_time)
    max = len(states) - 1
    iter_slider.valmin = 0
    iter_slider.valmax = max
    iter_slider.ax.set_xlim(iter_slider.valmin,iter_slider.valmax)
    iter_slider.valfmt = f'%i/{max}'

    if real_time:
        index = max
        if not from_slider:
            iter_slider.set_val(index) 
    else:
        index = int(iter_slider.val)
    index %= len(states)
    states[index].plot()
        
checkboxes.on_clicked(update_plot)
iter_slider.on_changed(lambda _ : update_plot(from_slider=True))

plt.show(block=False)

# stop to admire our empty window axes and ensure it is rendered at
# least once.
#
# We need to fully draw the figure at its final size on the screen
# before we continue on so that :
#  a) we have the correctly sized and drawn background to grab
#  b) we have a cached renderer so that ``ax.draw_artist`` works
# so we spin the event loop to let the backend process any pending operations
plt.pause(0.1)

# get copy of entire figure (everything inside fig.bbox) sans animated artist
scat_bg = fig.canvas.copy_from_bbox(scat_ax.bbox)

def draw_plot():
    fig.canvas.restore_region(scat_bg)

    # draw the animated artist, this uses a cached renderer
    if get_particles_status():
        scat_ax.draw_artist(particle_scat)
    scat_ax.draw_artist(mean_scat)

    # show the result to the screen, this pushes the updated RGBA buffer from the
    # renderer to the GUI framework so you can see it
    fig.canvas.blit(scat_ax.bbox)
    # flush any pending GUI events, re-painting the screen if needed
    fig.canvas.flush_events()

class ScatterState:
    def __init__(self, x: list[float], y: list[float], weights: list[float]):
        self.particles = np.array(list(zip(x, y)))
        self.weights = np.array(weights)
        self.mean = np.average(self.particles, weights=self.weights, axis=0)
    def plot(self):
        particle_scat.set_offsets(self.particles)
        particle_scat.set_alpha(self.weights * 10)
        mean_scat.set_offsets(self.mean)
        draw_plot()
        # # reset the background back in the canvas state, screen unchanged
        # fig.canvas.restore_region(bg)
        # particle_scat.set_offsets(self.particles)
        # mean_scat.set_offsets(self.mean)
        # particle_scat.set_alpha(self.weights * 10)
        # # re-render the artist, updating the canvas state, but not the screen
        # scat_ax.draw_artist(particle_scat)
        # scat_ax.draw_artist(mean_scat)
        # # copy the image to the GUI state, but screen might not be changed yet
        # fig.canvas.blit(fig.bbox)
            
        # plt.pause(0.001)
        
        
states = list[ScatterState]()

class PacketType:
    def __init__(self, identifier=None, shouldPrint=True, handler=None, matcher=None, removeIdentifier=True):
        if identifier is not None:
            self.startPattern = "[{identifier}]:".format(identifier=identifier)
        else: 
            self.startPattern = None
        self.shouldPrint = shouldPrint
        self.handler = handler
        self.matcher = matcher
        self.removeIdentifier = removeIdentifier
    def matches(self, line):
        if self.matcher is not None:
            return self.matcher(line)
        if self.startPattern is None:
            return True
        return line.startswith(self.startPattern)
    def handle(self, line):
        if self.shouldPrint:
            print(line, flush=True, end="")
        if self.handler is not None:
            if self.startPattern is not None and self.removeIdentifier:
                self.handler(line[len(self.startPattern):])
            else:
                self.handler(line)


def handle_pf_data_telemetry(dataStr: str):
    # dataStr is formatted like: "<x_float_1>,<y_float_1>;<x_float_2>,<y_float_2>;..." 
    x = []
    y = []
    weights = []
    for particle in dataStr.split(";"):
        coords = particle.split(",")
        if len(coords) == 1 and coords[0].strip() == "":
            continue
        if len(coords) != 3:
            raise ValueError("Invalid particle telemetry: ", particle)
        x_new, y_new, weight_new = coords
        x.append(float(x_new))
        y.append(float(y_new))
        weights.append(float(weight_new ))
    states.append(ScatterState(x, y, weights))
    update_plot()
    
packet_types = [
    PacketType("TELEMETRY.PF.DATA", handler=(lambda dataStr: handle_pf_data_telemetry(dataStr)), shouldPrint=False),
    # Matches all and prints all
    PacketType(),
]

def on_receive(line):
    for type in packet_types:
        if type.matches(line):
            try:
                type.handle(line)
            except Exception as e:
                print("Error handling packet: ", e)
                raise e
            return

# Used to ensure errors kill the simulator
process = None
def cleanup(_=None):
    if process is not None and process.poll() is None:
        os.killpg(os.getpgid(process.pid), signal.SIGTERM)
atexit.register(cleanup)
fig.canvas.mpl_connect('close_event', cleanup)


def run_simulator():
    global process
    process = subprocess.Popen("simulator monolith ./bin/sim/monolith.bin", shell=True, preexec_fn=os.setsid, text=True, stdout=subprocess.PIPE)
    while True:
        line = process.stdout.readline()
        if line == '' and process.poll() is not None:
            break
        on_receive(line)
    print("Simulation finished??") 

run_simulator()
# simulator_thread = threading.Thread(target=run_simulator)
# simulator_thread.start()



# Wait for simulator to finish
# simulator_thread.join()