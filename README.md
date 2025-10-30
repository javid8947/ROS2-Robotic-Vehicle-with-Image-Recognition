[readme.txt](https://github.com/user-attachments/files/23239029/readme.txt)
open javid_robot folder in terminal (not the one with .py files)
run to install ros2, and update everything else:
sudo chmod +x install_ros.sh && ./install_ros.sh

after this, run for python requirements:
pip install -r requirements.txt

then, build package:
colcon build

the package should now work
you will need to source files:
sudo chmod +x install/setup.bash

then, open 3 terminal windows in same javid_robot folder
in all 3, run:
source install/setup.bash

this will allow you to run the files:
ros2 run javid_robot ai
ros2 run javid_robot camera
ros2 run javid_robot arduino

you will need to edit javid_robot/javid_robot/arduino_code/arduino_code.ino
edit pins at the top of the file to fit your arduino robot
then, put ino file on to arduino via usb

you may also need to edit javid_robot/javid_robot/arduino.py
in the line:

    try:
        self.serial_port = serial.Serial('/dev/ttyACM0', 9600)  # Change the port accordingly
        self.get_logger().info('Serial port initialized')

/dev/ttyACM0 might not be right for your arduino

the arduino ide (found at https://www.arduino.cc/en/software you want the linux appimage 64 bits)
will tell you what /dev/ the arduino is plugged in at
you will need to make this file executable too
open terminal in your downloads folder
sudo chmod +x arduino(press tab here and it should autocomplete the rest of the name)
