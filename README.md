# odrive_can_ros_drive

ROS driver for the [ODrive](https://odriverobotics.com) motor controller connecting through CAN bus

This is a basic driver node for ROS which builds an interface to control multiple [ODrive](https://odriverobotics.com) motor controllers through a CAN bus. This driver depends on [socketcan_bridge](http://wiki.ros.org/socketcan_bridge) package to connect to physical CAN bus.

Right now driver only supports velocity control mode as this was my prime area of interest for my own project but it can be extended to provide support for other modes of control. I would be happy to accept any pull requests extending funtionality of this driver.

This driver was successfully tested with multiple ODrive controllers on a CAN bus connected to Raspberry Pi 4 through Waveshare CAN board on Ubuntu 18.04 and 20.04 with ROS melodic and noetic.

## Usage

### Getting the code

To use the driver just check it out to your ROS workspace:
```
git clone https://github.com/belovictor/odrive_can_ros_driver.git
```

### Dependancies

The driver depends on socketcan_bridge package. Please install it prior to using the driver:
```
sudo apt install ros-<ros distribution name>-socketcan-bridge
```
The package name would depend on your ROS version. This driver has been tested and is known to work with melodic and noetic.

### Configuration

Driver is configured with the following configuration paramters:

| Parameter name    | Type     | Description                                                                        |
|-------------------|----------|------------------------------------------------------------------------------------|
| can_device        | String   | CAN device name as it appears on ifconfig interface list, default is can0          |
| can_rx_topic      | String   | Topic name through which driver receives CAN messages from socketcan_bridge        |
| can_tx_ropic      | String   | Topic name through which driver sends CAN messages to socketcan_bridge             |
| update_rate       | Double   | Update rate in Hz of how frequently driver gets data from ODrive controllers       |
| engage_on_startup | Boolean  | If driver should engage all motors on startup (not implemented yet)                |
| axis_names        | String[] | Array of names of axises which are then used to name topics corresponding to axises|
| axis_can_ids      | Int[]    | Array of axis CAN ids as they are configured on ODrive controllers for every axis  |

An example configuration is presented in config/odrive.yaml.

### Running the driver

```
roslaunch odrive_can_ros_driver odrive_can_ros.launch
```
