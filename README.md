# quadro
ros2 quadruped

TODO:
WHEN WALK GAIT (CRAWL) - let the body shift in y axis instead of rotate! - it's way more stable

Walking controller redesign and refactor - it should be possible to have walking on a plane and yaw happen at the same time,
also the controller should take in walking speed (in m/s) and generate corresponding gait patterns


Hardware: 
- make space for USB cable in each link,
- change link 1 design so that it can rotate clockwise more (For BR)
- Make space for a stop in link 4 !
- in link 3 top, make the stopper longer (?)

Each motor setup:
- calibrate and save,
- decrease current to 10A max,
- decrease max vel to 420 rpm,
- enable can / disable can resistor on some,
- change filter bandwidth to 25