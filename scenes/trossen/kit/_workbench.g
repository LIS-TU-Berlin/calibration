# Arbeitstisch 825 x 1250 x 800 mm (HxBxT), Traglast 500 kg
# Buche massiv top (28mm), frame Telegrau 1 (RAL 7045), legs 45x45mm steel tube
# https://lagertechnik-becker.de/shop/Lager-Betrieb/Betriebsausstattung/Werkbaenke/Werkbaenke/Arbeitstische/Arbeitstisch-825-x-1-250-x-800-mm-HxBxT-Traglast-500-kg-Buche-Telegrau-1-RAL-7045-Telegrau-1.html

workbench: { X:[0, 0, 0], shape: marker, size: [.2] }

# solid beech tabletop, 1250 x 800 x 28 mm, top surface at z=0.825
# (plain box shape: rai auto-generates texture coords only for boxes, not ssBox)
workbench_top(workbench): {
 Q:[0, 0, .811], shape: box, size: [1.25, .8, .028],
 color: [.85, .7, .5], contact: 1  # texture: <textures/beech_preview_tiled.png> -- resolved relative to the cwd, not this file
}

# legs: 45 x 45 mm profiled steel tube, powder-coated RAL 7045
workbench_leg_fl(workbench): { Q:[ .5625,  .3375, .3985], shape: ssBox, size: [.045, .045, .797, .005], color: [.55, .57, .58], contact: 1 }
workbench_leg_fr(workbench): { Q:[ .5625, -.3375, .3985], shape: ssBox, size: [.045, .045, .797, .005], color: [.55, .57, .58], contact: 1 }
workbench_leg_bl(workbench): { Q:[-.5625,  .3375, .3985], shape: ssBox, size: [.045, .045, .797, .005], color: [.55, .57, .58], contact: 1 }
workbench_leg_br(workbench): { Q:[-.5625, -.3375, .3985], shape: ssBox, size: [.045, .045, .797, .005], color: [.55, .57, .58], contact: 1 }

# upper struts front and rear (along width)
workbench_strut_upper_front(workbench): { Q:[0,  .3375, .7745], shape: ssBox, size: [1.08, .045, .045, .005], color: [.55, .57, .58] }
workbench_strut_upper_rear(workbench):  { Q:[0, -.3375, .7745], shape: ssBox, size: [1.08, .045, .045, .005], color: [.55, .57, .58] }

# upper depth bracing left and right
workbench_strut_upper_left(workbench):  { Q:[ .5625, 0, .7745], shape: ssBox, size: [.045, .63, .045, .005], color: [.55, .57, .58] }
workbench_strut_upper_right(workbench): { Q:[-.5625, 0, .7745], shape: ssBox, size: [.045, .63, .045, .005], color: [.55, .57, .58] }

# rear lower strut (along width) and lower depth bracing
workbench_strut_lower_rear(workbench):  { Q:[0, -.3375, .15], shape: ssBox, size: [1.08, .045, .045, .005], color: [.55, .57, .58] }
workbench_strut_lower_left(workbench):  { Q:[ .5625, 0, .15], shape: ssBox, size: [.045, .63, .045, .005], color: [.55, .57, .58] }
workbench_strut_lower_right(workbench): { Q:[-.5625, 0, .15], shape: ssBox, size: [.045, .63, .045, .005], color: [.55, .57, .58] }
