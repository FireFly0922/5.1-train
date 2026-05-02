#include "sensor_drv.h"

#include "status.h"

void Sensor_Init(void) {
    /* 传感器状态统一存放在全局 STATUS 中，这里只负责初始化默认值。 */
    STATUS.sensor.gyro.yaw = 0.0f;
    STATUS.sensor.gyro.pitch = 0.0f;
    STATUS.sensor.gyro.roll = 0.0f;
    STATUS.sensor.vision_target = 0.0f;
}
