# Steps:

## 1. Build the pacakege:

colcon build --packages-select omega_ros2_get_position

## 2. Source the workspace

. install/setup.bash

## 3. Elevate usb privilege to superuser

sudo su

## 4. Source 

. install/setup.bash

## 5. Run node

ros2 run omega_ros2_get_position omega_ros2_get_position

## 6. Result

The position information of your device should be published.
