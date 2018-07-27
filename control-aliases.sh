# Bunch of rmp440 control aliases
rmp440-connect-joystick() {
    rosservice call /rmp440/connect_port {'Joystick','/joystick/device/Logitech_Gamepad_F710'}
}
rmp440-connect-jtrack () {
    rosservice call /joystick_to_twist/connect_port {'joystick','/joystick/device/Logitech_Gamepad_F710'}
    rosservice call /rmp440/connect_port {'cmd_vel_Infuse','/joystick_to_twist/twist'}
}
rmp440-init() {
    rosaction call /rmp440/Init "10.40.40.40:8080"
}
rmp440-gyro() {
    rosaction call /rmp440/Gyro '{params: {mode: {value: 2}, port: "/dev/ttyS0", type: {value: 3}, latitude: 43.56192, woffset: 0.0 }}'
}
rmp440-update-gyro-bias() {
    rosaction call /rmp440/GyroBiasUpdate ''$1''
}
rmp440-mti() {
    rosaction call /rmp440/InitMTI '{params: {mode: {value: 2}, port: "/dev/ttyS1", outputMode: 6, outputFormat: 4}}'
}
rmp440-toggle-odometry-mode() {
    rosaction call /rmp440/ToggleOdometryMode {}
}
rmp440-joystick-on() {
    rosaction call /rmp440/JoystickOn {}
}
rmp440-toggle-track-mode() {
    rosservice call /rmp440/toggleInfuseTrackMode {}
}
rmp440-track-mode() {
    rosaction call /rmp440/Track {}
}
rmp440-init-track-mode() {
    rmp440-connect-jtrack
    rmp440-gyro
    rmp440-update-gyro-bias 500
    rmp440-toggle-track-mode
    rmp440-init
    rmp440-track-mode
}
rmp440-init-joystick-mode() {
    rmp440-connect-joystick
    rmp440-gyro
    rmp440-update-gyro-bias 500
    rmp440-init
    rmp440-joystick-on
}
rmp440-odo2d() {
    rmp440-connect-joystick
    rmp440-gyro
    rmp440-update-gyro-bias 5000
    rmp440-init
    rmp440-joystick-on
}
rmp440-odo3d() {
    rmp440-connect-joystick
    rmp440-gyro
    rmp440-update-gyro-bias 5000
    rmp440-mti
    rmp440-toggle-odometry-mode
    rmp440-init
    rmp440-joystick-on
}
