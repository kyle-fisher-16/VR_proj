Intended for Raspberry Pi V3 B and accompanying PC (to run java visualization)

To run this demo, attach the Arducam 5MP camera via the ribbon connector, and ensure that the MPU-9250 is attached using i2c. 

The following flags may be set using the source code:

PLOT_KEYPOINTS = False
SHOW_IMAGE = True
SIGNAL_EXIT = False

Python: run on Raspi
calib_capture/process.py: used for calibrating the Arducam
main.py: r 

Java: run on PC
demo_loop.java: visualization
DispFrame.java: helper class for demo_loop.java
