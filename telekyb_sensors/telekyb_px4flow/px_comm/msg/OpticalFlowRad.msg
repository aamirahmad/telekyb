Header header
uint32 integration_time_us #Integration time in microseconds. Divide integrated_x and integrated_y by the integration time to obtain average flow. The integration time also indicates the.
float32 integrated_x #Flow in radians around X axis (Sensor RH rotation about the X axis induces a positive flow. Sensor linear motion along the positive Y axis induces a negative flow.)
float32 integrated_y #Flow in radians around Y axis (Sensor RH rotation about the Y axis induces a positive flow. Sensor linear motion along the positive X axis induces a positive flow.)
float32 integrated_xgyro #RH rotation around X axis (rad)
float32 integrated_ygyro #RH rotation around Y axis (rad)
float32 integrated_zgyro #RH rotation around Z axis (rad)
uint32 time_delta_distance_us #Time in microseconds since the distance was sampled.
float32 distance #Distance to the center of the flow field in meters. Positive value (including zero): distance known. Negative value: Unknown distance.
float32 temperature #Temperature degrees Celsius
uint8 quality #Optical flow quality / confidence. 0: no valid flow, 255: maximum quality