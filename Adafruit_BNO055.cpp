#include "Arduino.h"

#include <limits.h>
#include <string.h>
#include <time.h>
#include <math.h>

#include "Adafruit_BNO055.h"

Adafruit_BNO055::Adafruit_BNO055(const char * filename){
    if(filename == NULL){
        this->random_values = true;
        srand(time(NULL));
        return;
    }
    this->random_values = false;
    
    #ifdef USING_CSV
    this->_filename = new char[strlen(filename) + 1];
    strncpy(this->_filename, filename, strlen(filename) + 1);
    this->sensor_data = new csv_parser::arduino_parser(filename, 10);
    this->file_index = 0;
    #else
    srand(time(NULL));
    #endif
    
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
    if(this->random_values){
        return rand() / rand();
    }
    #ifdef USING_CSV
    if(this->file_index >= this->sensor_data->getSize()){
        return 0;
    }
    int8_t temp = (int8_t)atoi(this->sensor_data->operator[](this->file_index)[TEMP]);
    #else   
    int8_t temp = rand()/rand();
    #endif
    return temp;
}

imu::Vector<3> Adafruit_BNO055::getVector(adafruit_vector_type_t vector_type){
    imu::Vector<3> v;

    if(this->random_values){
        v[0] = rand() / rand();
        v[1] = rand() / rand();
        v[2] = rand() / rand();
        return v;
    }
    
    #ifdef USING_CSV
    if(this->file_index >= this->sensor_data->getSize()){
        v[0] = 0;
        v[1] = 0;
        v[2] = 0;
        return v;
    }

    switch (vector_type){
        case VECTOR_ACCELEROMETER:
        v[0] = atof(this->sensor_data->operator[](this->file_index)[ACC_X]);
        v[1] = atof(this->sensor_data->operator[](this->file_index)[ACC_Y]);
        v[2] = atof(this->sensor_data->operator[](this->file_index)[ACC_Z]);
        break;
        case VECTOR_GYROSCOPE:
        v[0] = atof(this->sensor_data->operator[](this->file_index)[GYRO_X]);
        v[1] = atof(this->sensor_data->operator[](this->file_index)[GYRO_Y]);
        v[2] = atof(this->sensor_data->operator[](this->file_index)[GYRO_Z]);
        break;
        case VECTOR_MAGNETOMETER:
        v[0] = atof(this->sensor_data->operator[](this->file_index)[MAG_X]);
        v[1] = atof(this->sensor_data->operator[](this->file_index)[MAG_Y]);
        v[2] = atof(this->sensor_data->operator[](this->file_index)[MAG_Z]);
        break;
        case VECTOR_LINEARACCEL:
        v[0] = atof(this->sensor_data->operator[](this->file_index)[LIN_ACC_X]);
        v[1] = atof(this->sensor_data->operator[](this->file_index)[LIN_ACC_Y]);
        v[2] = atof(this->sensor_data->operator[](this->file_index)[LIN_ACC_Z]);
        break;
        case VECTOR_EULER:
        v[0] = atof(this->sensor_data->operator[](this->file_index)[EULER_X]);
        v[1] = atof(this->sensor_data->operator[](this->file_index)[EULER_Y]);
        v[2] = atof(this->sensor_data->operator[](this->file_index)[EULER_Z]);
        break;
        case VECTOR_GRAVITY:
        v[0] = atof(this->sensor_data->operator[](this->file_index)[GRAVITY_X]);
        v[1] = atof(this->sensor_data->operator[](this->file_index)[GRAVITY_Y]);
        v[2] = atof(this->sensor_data->operator[](this->file_index)[GRAVITY_Z]);
        break;
        
        default:
        v[0] = 0;
        v[1] = 0;
        v[2] = 0;
        break;
    }
    #else
    v[0] = rand() / rand();
    v[1] = rand() / rand();
    v[2] = rand() / rand();
    #endif
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

void Adafruit_BNO055::increase_file_index(){
    ++this->file_index;
}

void Adafruit_BNO055::show_simulated_file(){
    #ifdef USING_CSV
    this->sensor_data->head(this->sensor_data->getSize());
    #endif
}


