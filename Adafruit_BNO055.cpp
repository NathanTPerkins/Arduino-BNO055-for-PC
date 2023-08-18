#include "Arduino.h"

#include <limits.h>
#include <math.h>

#include "Adafruit_BNO055.h"

#define ACC_X 0
#define ACC_Y 1
#define ACC_Z 2
#define GYRO_X 3
#define GYRO_Y 4
#define GYRO_Z 5
#define MAG_X 6
#define MAG_Y 7
#define MAG_Z 8
#define LIN_ACC_X 9
#define LIN_ACC_Y 10
#define LIN_ACC_Z 11
#define ANG_VEL_X 12
#define ANG_VEL_Y 13
#define ANG_VEL_Z 14
#define EULER_X 15
#define EULER_Y 16
#define EULER_Z 17
#define GRAVITY_X 18
#define GRAVITY_Y 19
#define GRAVITY_Z 20
#define ALT 21
#define TEMP 22

/*

Column Names for CSV


acc_x,
acc_y,
acc_z,
gyro_x,
gyro_y,
gyro_z,
mag_x,
mag_y,
mag_z,
lin_acc_x,
lin_acc_y,
lin_acc_z,
ang_vel_x,
ang_vel_y,
ang_vel_z,
euler_x,
euler_y,
euler_z,
gravity_x,
gravity_y,
gravity_z,
alt,
temp,


*/

Adafruit_BNO055::Adafruit_BNO055(int32_t sensorID = -1, uint8_t address = BNO055_ADDRESS_A, const char * filename = NULL){
    if(filename == NULL){
        this->random_values = true;
        return;
    }
    this->random_values = false;
    this->_filename = new char[strlen(filename) + 1];
    strncpy(this->_filename, filename, strlen(filename) + 1);
    sensor_data = new csv_parser::parser(filename, 10);
    this->file_index = 0;
    this->_running = false;
}

bool Adafruit_BNO055::begin(adafruit_bno055_opmode_t mode){
    this->_mode = mode;
    this->_running = true;
    return true;
}

void Adafruit_BNO055::setMode(adafruit_bno055_opmode_t mode){
    this->_mode = mode;
}

adafruit_bno055_opmode_t Adafruit_BNO055::getMode(){
    return this->_mode;
}

void Adafruit_BNO055::setAxisRemap(adafruit_bno055_axis_remap_config_t remapcode){}

void Adafruit_BNO055::setAxisSign(adafruit_bno055_axis_remap_sign_t remapsign){}

void Adafruit_BNO055::setExtCrystalUse(boolean useextal){}

void Adafruit_BNO055::getSystemStatus(uint8_t *system_status, uint8_t* self_test_result, uint8_t* system_error){
    /* System Status (see section 4.3.58)
     0 = Idle
     1 = System Error
     2 = Initializing Peripherals
     3 = System Iniitalization
     4 = Executing Self-Test
     5 = Sensor fusio algorithm running
     6 = System running without fusion algorithms
   */

  *system_status = 3;

  /* Self Test Results
     1 = test passed, 0 = test failed

     Bit 0 = Accelerometer self test
     Bit 1 = Magnetometer self test
     Bit 2 = Gyroscope self test
     Bit 3 = MCU self test

     0x0F = all good!
   */

  *self_test_result = 0x0F;

  /* System Error (see section 4.3.59)
     0 = No error
     1 = Peripheral initialization error
     2 = System initialization error
     3 = Self test result failed
     4 = Register map value out of range
     5 = Register map address out of range
     6 = Register map write error
     7 = BNO low power mode not available for selected operat ion mode
     8 = Accelerometer power mode not available
     9 = Fusion algorithm configuration error
     A = Sensor configuration error
   */

  *system_error = 0;
}

void Adafruit_BNO055::getRevInfo(adafruit_bno055_rev_info_t *info) {
    memset(info, 0, sizeof(adafruit_bno055_rev_info_t));
}

void Adafruit_BNO055::getCalibration(uint8_t* sys, uint8_t* gyro, uint8_t* accel, uint8_t* mag){
    *sys = 3;
    *gyro = 3;
    *accel = 3;
    *mag = 3;
}

int8_t Adafruit_BNO055::getTemp(){
    if(this->file_index >= this->sensor_data.getSize()){
        return 0;
    }
    return this->sensor_data[TEMP][this->file_index++];
}

imu::Vector<3> Adafruit_BNO055::getVector(adafruit_vector_type_t vector_type){
    imu::Vector<3> v;

    if(this->file_index >= this->sensor_data.getSize()){
        v[0] = 0;
        v[1] = 0;
        v[2] = 0;
        return v;
    }

    switch (vector_type)
    {
    case VECTOR_ACCELEROMETER:
    v[0] = this->sensor_data[ACC_X][this->file_index];
    v[1] = this->sensor_data[ACC_Y][this->file_index];
    v[2] = this->sensor_data[ACC_Z][this->file_index];
    break;
    case VECTOR_GYROSCOPE:
    v[0] = this->sensor_data[GYRO_X][this->file_index];
    v[1] = this->sensor_data[GYRO_Y][this->file_index];
    v[2] = this->sensor_data[GYRO_Z][this->file_index];
    break;
    case VECTOR_MAGNETOMETER:
    v[0] = this->sensor_data[MAG_X][this->file_index];
    v[1] = this->sensor_data[MAG_Y][this->file_index];
    v[2] = this->sensor_data[MAG_Z][this->file_index];
    break;
    case VECTOR_LINEARACCEL:
    v[0] = this->sensor_data[LIN_ACC_X][this->file_index];
    v[1] = this->sensor_data[LIN_ACC_Y][this->file_index];
    v[2] = this->sensor_data[LIN_ACC_Z][this->file_index];
    break;
    case VECTOR_EULER:
    v[0] = this->sensor_data[EULER_X][this->file_index];
    v[1] = this->sensor_data[EULER_Y][this->file_index];
    v[2] = this->sensor_data[EULER_Z][this->file_index];
    break;
    case VECTOR_GRAVITY:
    v[0] = this->sensor_data[GRAVITY_X][this->file_index];
    v[1] = this->sensor_data[GRAVITY_Y][this->file_index];
    v[2] = this->sensor_data[GRAVITY_Z][this->file_index];
    break;
    
    default:
    v[0] = 0;
    v[1] = 0;
    v[2] = 0;
    break;
    }
    return v;
}

imu::Quaternion Adafruit_BNO055::getQuat(){
    const double scale = (1.0 / (1 << 14));
    imu::Quaternion quat(scale * 1, scale * 1, scale * 1, scale * 1);
    return quat;
}

void Adafruit_BNO055::getSensor(sensor_t *sensor){
    memset(sensor, 0, sizeof(sensor_t));
    strncpy(sensor->name, "BNO055", sizeof(sensor->name) - 1);
    sensor->name[sizeof(sensor->name) - 1] = 0;
    sensor->version = 1;
    sensor->sensor_id = -1;
    sensor->type = SENSOR_TYPE_ORIENTATION;
    sensor->min_delay = 0;
    sensor->max_value = 0.0F;
    sensor->min_value = 0.0F;
    sensor->resolution = 0.01F;
}

bool Adafruit_BNO055::getEvent(sensors_event_t *event) {
    /* Clear the event */
    memset(event, 0, sizeof(sensors_event_t));

    event->version = sizeof(sensors_event_t);
    event->sensor_id = -1;
    event->type = SENSOR_TYPE_ORIENTATION;
    event->timestamp = millis();

    /* Get a Euler angle sample for orientation */
    imu::Vector<3> euler = getVector(Adafruit_BNO055::VECTOR_EULER);
    event->orientation.x = euler.x();
    event->orientation.y = euler.y();
    event->orientation.z = euler.z();
    ++this->file_index;
    return true;
}

bool Adafruit_BNO055::getEvent(sensors_event_t *event, adafruit_vector_type_t vec_type) {
    /* Clear the event */
    memset(event, 0, sizeof(sensors_event_t));

    event->version = sizeof(sensors_event_t);
    event->sensor_id = -1;
    event->timestamp = millis();

    // read the data according to vec_type
    imu::Vector<3> vec;
    if (vec_type == Adafruit_BNO055::VECTOR_LINEARACCEL) {
        event->type = SENSOR_TYPE_LINEAR_ACCELERATION;
        vec = getVector(Adafruit_BNO055::VECTOR_LINEARACCEL);

        event->acceleration.x = vec.x();
        event->acceleration.y = vec.y();
        event->acceleration.z = vec.z();
    } else if (vec_type == Adafruit_BNO055::VECTOR_ACCELEROMETER) {
        event->type = SENSOR_TYPE_ACCELEROMETER;
        vec = getVector(Adafruit_BNO055::VECTOR_ACCELEROMETER);

        event->acceleration.x = vec.x();
        event->acceleration.y = vec.y();
        event->acceleration.z = vec.z();
    } else if (vec_type == Adafruit_BNO055::VECTOR_GRAVITY) {
        event->type = SENSOR_TYPE_GRAVITY;
        vec = getVector(Adafruit_BNO055::VECTOR_GRAVITY);

        event->acceleration.x = vec.x();
        event->acceleration.y = vec.y();
        event->acceleration.z = vec.z();
    } else if (vec_type == Adafruit_BNO055::VECTOR_EULER) {
        event->type = SENSOR_TYPE_ORIENTATION;
        vec = getVector(Adafruit_BNO055::VECTOR_EULER);

        event->orientation.x = vec.x();
        event->orientation.y = vec.y();
        event->orientation.z = vec.z();
    } else if (vec_type == Adafruit_BNO055::VECTOR_GYROSCOPE) {
        event->type = SENSOR_TYPE_GYROSCOPE;
        vec = getVector(Adafruit_BNO055::VECTOR_GYROSCOPE);

        event->gyro.x = vec.x() * SENSORS_DPS_TO_RADS;
        event->gyro.y = vec.y() * SENSORS_DPS_TO_RADS;
        event->gyro.z = vec.z() * SENSORS_DPS_TO_RADS;
    } else if (vec_type == Adafruit_BNO055::VECTOR_MAGNETOMETER) {
        event->type = SENSOR_TYPE_MAGNETIC_FIELD;
        vec = getVector(Adafruit_BNO055::VECTOR_MAGNETOMETER);

        event->magnetic.x = vec.x();
        event->magnetic.y = vec.y();
        event->magnetic.z = vec.z();
    }
    ++this->file_index;
    return true;
}

bool Adafruit_BNO055::getSensorOffsets(uint8_t *calibData) {
    return true;
}

bool Adafruit_BNO055::getSensorOffsets(adafruit_bno055_offsets_t &offsets_type){
    offsets_type.accel_offset_x = 1;
    offsets_type.accel_offset_y = 1;
    offsets_type.accel_offset_z = 1;
    offsets_type.accel_radius = 1;
    offsets_type.gyro_offset_x = 1;
    offsets_type.gyro_offset_y = 1;
    offsets_type.gyro_offset_z = 1;
    offsets_type.mag_offset_x = 1;
    offsets_type.mag_offset_y = 1;
    offsets_type.mag_offset_z = 1;
    offsets_type.mag_radius = 1;
    return true;
}

bool Adafruit_BNO055::isFullyCalibrated(){
    return true;
}

void Adafruit_BNO055::enterSuspendMode() {}
void Adafruit_BNO055::enterNormalMode() {}


