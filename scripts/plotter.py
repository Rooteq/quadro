import odrive
# from odrive.plotting import start_liveplotter
from odrive.utils import start_liveplotter
import matplotlib as plt

print(odrive.__version__)

odrv0 = odrive.find_any()


class ShiftedProperty:
    def __init__(self, prop, offset):
        self.prop = prop
        self.offset = offset
    
    def __call__(self):
        return [self.prop()[0] + self.offset]

offset = odrv0.axis0.encoder.config.index_offset

properties_to_plot = [
    odrv0.axis0.encoder._pos_estimate_property,
    odrv0.axis0.encoder._vel_estimate_property
]

print("starting liveplotter...")
# Pass the list of properties to the start_liveplotter function
start_liveplotter(properties_to_plot)

# offset = 10.0  # your desired offset
# start_liveplotter([
#     odrv0.axis0.encoder._pos_estimate_property,
#     ShiftedProperty(odrv0.axis0.controller._pos_setpoint_property, offset)
# ], '1')