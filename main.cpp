// ----------------------------------------------------------------------------
// Copyright 2016-2020 ARM Ltd.
//
// SPDX-License-Identifier: Apache-2.0
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
// ----------------------------------------------------------------------------
#ifndef MBED_TEST_MODE
#include "mbed.h"
#include "kv_config.h"
#include "mbed-cloud-client/MbedCloudClient.h" // Required for new MbedCloudClient()
#include "factory_configurator_client.h"       // Required for fcc_* functions and FCC_* defines
#include "m2mresource.h"                       // Required for M2MResource
#include "key_config_manager.h"                // Required for kcm_factory_reset

#include "mbed-trace/mbed_trace.h"             // Required for mbed_trace_*

#include "LSM9DS1.h"
#include "icm20602_i2c.h"
#include "Si7021.h"
#include "BME680_BSEC.h"
#include "VL53L0X.h"

// Pointers to the resources that will be created in main_application().
static MbedCloudClient *cloud_client;
static bool cloud_client_running = true;
static NetworkInterface *network = NULL;

// Fake entropy needed for non-TRNG boards. Suitable only for demo devices.
const uint8_t MBED_CLOUD_DEV_ENTROPY[] = { 0xf6, 0xd6, 0xc0, 0x09, 0x9e, 0x6e, 0xf2, 0x37, 0xdc, 0x29, 0x88, 0xf1, 0x57, 0x32, 0x7d, 0xde, 0xac, 0xb3, 0x99, 0x8c, 0xb9, 0x11, 0x35, 0x18, 0xeb, 0x48, 0x29, 0x03, 0x6a, 0x94, 0x6d, 0xe8, 0x40, 0xc0, 0x28, 0xcc, 0xe4, 0x04, 0xc3, 0x1f, 0x4b, 0xc2, 0xe0, 0x68, 0xa0, 0x93, 0xe6, 0x3a };

static M2MResource* m2m_get_res;
static M2MResource* m2m_put_res;
static M2MResource* m2m_post_res;
static M2MResource* m2m_deregister_res;
static M2MResource* m2m_factory_reset_res;
static M2MResource *m2m_lsm9ds1_accelerometer_x_res;
static M2MResource *m2m_lsm9ds1_accelerometer_y_res;
static M2MResource *m2m_lsm9ds1_accelerometer_z_res;
static M2MResource *m2m_lsm9ds1_gyrometer_x_res;
static M2MResource *m2m_lsm9ds1_gyrometer_y_res;
static M2MResource *m2m_lsm9ds1_gyrometer_z_res;
static M2MResource *m2m_lsm9ds1_magnetometer_x_res;
static M2MResource *m2m_lsm9ds1_magnetometer_y_res;
static M2MResource *m2m_lsm9ds1_magnetometer_z_res;
static M2MResource *m2m_icm20602_accelerometer_x_res;
static M2MResource *m2m_icm20602_accelerometer_y_res;
static M2MResource *m2m_icm20602_accelerometer_z_res;
static M2MResource *m2m_icm20602_gyrometer_x_res;
static M2MResource *m2m_icm20602_gyrometer_y_res;
static M2MResource *m2m_icm20602_gyrometer_z_res;
static M2MResource *m2m_si7021_temperature_res;
static M2MResource *m2m_si7021_humidity_res;
static M2MResource *m2m_bme680_temperature_res;
static M2MResource *m2m_bme680_humidity_res;
static M2MResource *m2m_bme680_pressure_res;
static M2MResource *m2m_bme680_gas_resistance_res;
static M2MResource *m2m_bme680_co2_equivalent_res;
static M2MResource *m2m_bme680_breath_voc_equivalent_res;
static M2MResource *m2m_bme680_iaq_score_res;
static M2MResource *m2m_bme680_iaq_accuracy_res;
static M2MResource *m2m_vl53l0x_distance_res;
static M2MResource *m2m_led_res;
static SocketAddress sa;

EventQueue queue(32 * EVENTS_EVENT_SIZE);
Thread t;
Mutex value_increment_mutex;

DigitalOut sensor_power_enable(PIN_NAME_SENSOR_POWER_ENABLE);

I2C i2c(PIN_NAME_SDA, PIN_NAME_SCL);
DevI2C devi2c(PIN_NAME_SDA, PIN_NAME_SCL);

static const uint8_t LSM9DS1_ACCEL_GYRO_ADDRESS = 0x6A << 1;
static const uint8_t LSM9DS1_MAG_ADDRESS = 0x1C << 1;
LSM9DS1 lsm9ds1(i2c, LSM9DS1_ACCEL_GYRO_ADDRESS, LSM9DS1_MAG_ADDRESS);
ICM20602 icm20602(i2c);
Si7021 si7021(i2c);
BME680_BSEC *bme680 = BME680_BSEC::get_instance();
static const uint8_t VL53L0X_ADDRESS = 0x52;
VL53L0X vl53l0x(&devi2c, PIN_NAME_INT_LIGHT_TOF, VL53L0X_ADDRESS);

DigitalOut status_led(PIN_NAME_LED_RED);

void print_client_ids(void)
{
    printf("Account ID: %s\n", cloud_client->endpoint_info()->account_id.c_str());
    printf("Endpoint name: %s\n", cloud_client->endpoint_info()->internal_endpoint_name.c_str());
    printf("Device ID: %s\n\n", cloud_client->endpoint_info()->endpoint_name.c_str());
}

void value_increment(void)
{
    value_increment_mutex.lock();
    m2m_get_res->set_value(m2m_get_res->get_value_int() + 1);
    printf("Counter %" PRIu64 "\n", m2m_get_res->get_value_int());
    value_increment_mutex.unlock();
}

void get_res_update(const char* /*object_name*/)
{
    printf("Counter resource set to %d\n", (int)m2m_get_res->get_value_int());
}

void put_res_update(const char* /*object_name*/)
{
    printf("PUT update %d\n", (int)m2m_put_res->get_value_int());
}

void execute_post(void* /*arguments*/)
{
    printf("POST executed\n");
}

void deregister_client(void)
{
    printf("Unregistering and disconnecting from the network.\n");
    cloud_client->close();
}

void deregister(void* /*arguments*/)
{
    printf("POST deregister executed\n");
    m2m_deregister_res->send_delayed_post_response();

    deregister_client();
}

void client_registered(void)
{
    printf("Client registered.\n");
    print_client_ids();
}

void client_unregistered(void)
{
    printf("Client unregistered.\n");
    (void) network->disconnect();
    cloud_client_running = false;
}

void factory_reset(void*)
{
    printf("POST factory reset executed\n");
    m2m_factory_reset_res->send_delayed_post_response();

    kcm_factory_reset();
}

void client_error(int err)
{
    printf("client_error(%d) -> %s\n", err, cloud_client->error_description());
}

void update_progress(uint32_t progress, uint32_t total)
{
    uint8_t percent = (uint8_t)((uint64_t)progress * 100 / total);
    printf("Update progress = %" PRIu8 "%%\n", percent);
}

float normalize_acceleration_value(int16_t acceleration_value)
{
    return acceleration_value * aRes;
}

float normalize_gyroscope_value(int16_t gyroscope_value)
{
    return gyroscope_value * gRes;
}

void update_resources(void)
{
    // Update LSM9DS1 values
    lsm9ds1.readAccel();
    lsm9ds1.readGyro();
    lsm9ds1.readMag();
    m2m_lsm9ds1_accelerometer_x_res->set_value_float(lsm9ds1.calcAccel(lsm9ds1.ax));
    m2m_lsm9ds1_accelerometer_y_res->set_value_float(lsm9ds1.calcAccel(lsm9ds1.ay));
    m2m_lsm9ds1_accelerometer_z_res->set_value_float(lsm9ds1.calcAccel(lsm9ds1.az));
    m2m_lsm9ds1_gyrometer_x_res->set_value_float(lsm9ds1.calcGyro(lsm9ds1.gx));
    m2m_lsm9ds1_gyrometer_y_res->set_value_float(lsm9ds1.calcGyro(lsm9ds1.gy));
    m2m_lsm9ds1_gyrometer_z_res->set_value_float(lsm9ds1.calcGyro(lsm9ds1.gz));
    m2m_lsm9ds1_magnetometer_x_res->set_value_float(lsm9ds1.calcMag(lsm9ds1.mx));
    m2m_lsm9ds1_magnetometer_y_res->set_value_float(lsm9ds1.calcMag(lsm9ds1.my));
    m2m_lsm9ds1_magnetometer_z_res->set_value_float(lsm9ds1.calcMag(lsm9ds1.mz));

    // Update ICM20602 values
    m2m_icm20602_accelerometer_x_res->set_value_float(normalize_acceleration_value(icm20602.getAccXvalue()));
    m2m_icm20602_accelerometer_y_res->set_value_float(normalize_acceleration_value(icm20602.getAccYvalue()));
    m2m_icm20602_accelerometer_z_res->set_value_float(normalize_acceleration_value(icm20602.getAccZvalue()));
    m2m_icm20602_gyrometer_x_res->set_value_float(normalize_gyroscope_value(icm20602.getGyrXvalue()));
    m2m_icm20602_gyrometer_y_res->set_value_float(normalize_gyroscope_value(icm20602.getGyrYvalue()));
    m2m_icm20602_gyrometer_z_res->set_value_float(normalize_gyroscope_value(icm20602.getGyrZvalue()));

    // Update SI7021 values
    si7021.measure();
    m2m_si7021_temperature_res->set_value_float(si7021.get_temperature() / 1000.0);
    m2m_si7021_humidity_res->set_value_float(si7021.get_humidity() / 1000.0);

    // Update BME680 values
    m2m_bme680_temperature_res->set_value_float(bme680->get_temperature());
    m2m_bme680_humidity_res->set_value_float(bme680->get_humidity());
    m2m_bme680_pressure_res->set_value_float(bme680->get_pressure() / 1000.0);
    m2m_bme680_gas_resistance_res->set_value_float(bme680->get_gas_resistance() / 1000.0);
    m2m_bme680_co2_equivalent_res->set_value_float(bme680->get_co2_equivalent());
    m2m_bme680_breath_voc_equivalent_res->set_value_float(bme680->get_breath_voc_equivalent());
    m2m_bme680_iaq_score_res->set_value_float(bme680->get_iaq_score());
    m2m_bme680_iaq_accuracy_res->set_value(bme680->get_iaq_accuracy());

    // Update VL53L0X value
    uint32_t distance;
    VL53L0X_Error retcode = vl53l0x.get_distance(&distance);
    switch (retcode) {
        case VL53L0X_ERROR_NONE:
            // Successful read
            m2m_vl53l0x_distance_res->set_value_float(distance / 1000.0);
            break;
        case VL53L0X_ERROR_RANGE_ERROR:
            // Range error, return -1
            m2m_vl53l0x_distance_res->set_value_float(-1.0f);
            break;
        default:
            break;
    }

    printf("--------------------------------------------------------------------------------\n");

    printf("- LSM9DS1 Accelerometer X (3304/0/5702)                 : %0.2f g\n", m2m_lsm9ds1_accelerometer_x_res->get_value_float());
    printf("- LSM9DS1 Accelerometer Y (3304/0/5703)                 : %0.2f g\n", m2m_lsm9ds1_accelerometer_y_res->get_value_float());
    printf("- LSM9DS1 Accelerometer Z (3304/0/5704)                 : %0.2f g\n", m2m_lsm9ds1_accelerometer_z_res->get_value_float());
    printf("- LSM9DS1 Gyroscope X (3334/0/5702)                     : %0.2f dps\n", m2m_lsm9ds1_gyrometer_x_res->get_value_float());
    printf("- LSM9DS1 Gyroscope Y (3334/0/5703)                     : %0.2f dps\n", m2m_lsm9ds1_gyrometer_y_res->get_value_float());
    printf("- LSM9DS1 Gyroscope Z (3334/0/5704)                     : %0.2f dps\n", m2m_lsm9ds1_gyrometer_z_res->get_value_float());
    printf("- LSM9DS1 Magnetometer X (3314/0/5702)                  : %0.2f gauss\n", m2m_lsm9ds1_magnetometer_x_res->get_value_float());
    printf("- LSM9DS1 Magnetometer Y (3314/0/5703)                  : %0.2f gauss\n", m2m_lsm9ds1_magnetometer_y_res->get_value_float());
    printf("- LSM9DS1 Magnetometer Z (3314/0/5704)                  : %0.2f gauss\n", m2m_lsm9ds1_magnetometer_z_res->get_value_float());
    printf("- ICM20602 Accelerometer X (3304/1/5702)                : %0.2f g\n", m2m_icm20602_accelerometer_x_res->get_value_float());
    printf("- ICM20602 Accelerometer Y (3304/1/5703)                : %0.2f g\n", m2m_icm20602_accelerometer_y_res->get_value_float());
    printf("- ICM20602 Accelerometer Z (3304/1/5704)                : %0.2f g\n", m2m_icm20602_accelerometer_z_res->get_value_float());
    printf("- ICM20602 Gyroscope X (3334/1/5702)                    : %0.2f dps\n", m2m_icm20602_gyrometer_x_res->get_value_float());
    printf("- ICM20602 Gyroscope Y (3334/1/5703)                    : %0.2f dps\n", m2m_icm20602_gyrometer_y_res->get_value_float());
    printf("- ICM20602 Gyroscope Z (3334/1/5704)                    : %0.2f dps\n", m2m_icm20602_gyrometer_z_res->get_value_float());
    printf("- SI7021 Temperature (3303/0/5700)                      : %0.2f degC\n", m2m_si7021_temperature_res->get_value_float());
    printf("- SI7021 Relative Humidity (3304/0/5700)                : %0.2f %%RH\n", m2m_si7021_humidity_res->get_value_float());
    printf("- BME680 Temperature (3303/1/5700)                      : %0.2f degC\n", m2m_bme680_temperature_res->get_value_float());
    printf("- BME680 Relative Humidity (3304/1/5700)                : %0.2f %%RH\n", m2m_bme680_humidity_res->get_value_float());
    printf("- BME680 Pressure (3315/0/5700)                         : %0.2f kPa\n", m2m_bme680_pressure_res->get_value_float());
    printf("- BME680 Gas Resistance (26243/0/26248)                 : %0.2f kOhms\n", m2m_bme680_gas_resistance_res->get_value_float());
    printf("- BME680 CO2 Equivalents (26243/0/26250)                : %0.2f ppm\n", m2m_bme680_co2_equivalent_res->get_value_float());
    printf("- BME680 Breath-VOC Equivalents (26243/0/26251)         : %0.2f ppm\n", m2m_bme680_breath_voc_equivalent_res->get_value_float());
    if (m2m_bme680_iaq_score_res->get_value_float() < 51.0) {
        // Good
        printf("- BME680 IAQ Score (26243/0/26252)                      : %0.2f (Good)\n", m2m_bme680_iaq_score_res->get_value_float());
    } else if (m2m_bme680_iaq_score_res->get_value_float() >= 51.0 && m2m_bme680_iaq_score_res->get_value_float() < 101.0 ) {
        // Average
        printf("- BME680 IAQ Score (26243/0/26252)                      : %0.2f (Average)\n", m2m_bme680_iaq_score_res->get_value_float());
    } else if (m2m_bme680_iaq_score_res->get_value_float() >= 101.0 && m2m_bme680_iaq_score_res->get_value_float() < 151.0 ) {
        // Little bad
        printf("- BME680 IAQ Score (26243/0/26252)                      : %0.2f (Little bad)\n", m2m_bme680_iaq_score_res->get_value_float());
    } else if (m2m_bme680_iaq_score_res->get_value_float() >= 151.0 && m2m_bme680_iaq_score_res->get_value_float() < 201.0 ) {
        // Bad
        printf("- BME680 IAQ Score (26243/0/26252)                      : %0.2f (Bad)\n", m2m_bme680_iaq_score_res->get_value_float());
    } else if (m2m_bme680_iaq_score_res->get_value_float() >= 201.0 && m2m_bme680_iaq_score_res->get_value_float() < 301.0 ) {
        // Worse
        printf("- BME680 IAQ Score (26243/0/26252)                      : %0.2f (Worse)\n", m2m_bme680_iaq_score_res->get_value_float());
    } else {
        // Very bad
        printf("- BME680 IAQ Score (26243/0/26252)                      : %0.2f (Very bad)\n", m2m_bme680_iaq_score_res->get_value_float());
    }
    switch (m2m_bme680_iaq_accuracy_res->get_value_int()) {
        default:
        case 0:
            printf("- BME680 IAQ Accuracy (26243/0/26253)                   : 0 (Unreliable)\n");
            break;
        case 1:
            printf("- BME680 IAQ Accuracy (26243/0/26253)                   : 1 (Low accuracy)\n");
            break;
        case 2:
            printf("- BME680 IAQ Accuracy (26243/0/26253)                   : 2 (Medium accuracy)\n");
            break;
        case 3:
            printf("- BME680 IAQ Accuracy (26243/0/26253)                   : 3 (High accuracy)\n");
            break;
    }
    if (m2m_vl53l0x_distance_res->get_value_float() >= 0) {
        printf("- VL53L0X Ranging Time-of-Flight Distance (3330/0/5700) : %0.2f m\n", m2m_vl53l0x_distance_res->get_value_float());
    } else {
        printf("- VL53L0X Ranging Time-of-Flight Distance (3330/0/5700) : OUT OF RANGE\n");
    }

    printf("--------------------------------------------------------------------------------\n\n");
}

void led_res_update(const char* /*object_name*/)
{
    status_led = ((int)m2m_led_res->get_value_int()) == 0 ? 1 : 0; // active low
}

void flush_stdin_buffer(void)
{
    FileHandle *debug_console = mbed::mbed_file_handle(STDIN_FILENO);
    while(debug_console->readable()) {
        char buffer[1];
        debug_console->read(buffer, 1);
    }
}

int main(void)
{
    int status;

    status = mbed_trace_init();
    if (status != 0) {
        printf("mbed_trace_init() failed with %d\n", status);
        return -1;
    }

    // Mount default kvstore
    printf("Application ready\n");
    status = kv_init_storage_config();
    if (status != MBED_SUCCESS) {
        printf("kv_init_storage_config() - failed, status %d\n", status);
        return -1;
    }

    // Connect with NetworkInterface
    printf("Connect to network\n");
    network = NetworkInterface::get_default_instance();
    if (network == NULL) {
        printf("Failed to get default NetworkInterface\n");
        return -1;
    }
    status = network->connect();
    if (status != NSAPI_ERROR_OK) {
        printf("NetworkInterface failed to connect with %d\n", status);
        return -1;
    }
    status = network->get_ip_address(&sa);
    if (status!=0) {
        printf("get_ip_address failed with %d\n", status);
        return -2;
    }
    printf("Network initialized, connected with IP %s\n\n", sa.get_ip_address());

    // Run developer flow
    printf("Start developer flow\n");
    status = fcc_init();
    if (status != FCC_STATUS_SUCCESS) {
        printf("fcc_init() failed with %d\n", status);
        return -1;
    }

    // Inject hardcoded entropy for the device. Suitable only for demo devices.
    (void) fcc_entropy_set(MBED_CLOUD_DEV_ENTROPY, sizeof(MBED_CLOUD_DEV_ENTROPY));
    status = fcc_developer_flow();
    if (status != FCC_STATUS_SUCCESS && status != FCC_STATUS_KCM_FILE_EXIST_ERROR && status != FCC_STATUS_CA_ERROR) {
        printf("fcc_developer_flow() failed with %d\n", status);
        return -1;
    }

    printf("Enable power to the sensors\n");
    sensor_power_enable = 1;

    // Sleep to give sensors time to come online
    ThisThread::sleep_for(500);

    if (lsm9ds1.begin()) {
        printf("LSM9DS1 online\n");
        lsm9ds1.calibrate();
    } else {
        printf("ERROR: LSM9DS1 offline!\n");
    }

    if (icm20602.isOnline()) {
        printf("ICM20602 online\n");
        icm20602.init();
    } else {
        printf("ERROR: ICM20602 offline!\n");
    }

    if (si7021.check()) {
        printf("SI7021 online\n");
    } else {
        printf("ERROR: SI7021 offline!\n");
    }

    if (bme680->init(&i2c)) {
        printf("BME680 online\n");
    } else {
        printf("ERROR: BME680 offline!\n");
    }

    VL53L0X_Error retcode = vl53l0x.init_sensor(VL53L0X_ADDRESS);
    if (retcode == VL53L0X_ERROR_NONE) {
        printf("VL53L0X online\n");
    } else {
        printf("ERROR: VL53L0X offline!\n");
    }

    printf("Create resources\n");
    M2MObjectList m2m_obj_list;

    // GET resource 3200/0/5501
    // PUT also allowed for resetting the resource
    m2m_get_res = M2MInterfaceFactory::create_resource(m2m_obj_list, 3200, 0, 5501, M2MResourceInstance::INTEGER, M2MBase::GET_PUT_ALLOWED);
    if (m2m_get_res->set_value(0) != true) {
        printf("m2m_get_res->set_value() failed\n");
        return -1;
    }
    if (m2m_get_res->set_value_updated_function(get_res_update) != true) {
        printf("m2m_get_res->set_value_updated_function() failed\n");
        return -1;
    }

    // PUT resource 3201/0/5853
    m2m_put_res = M2MInterfaceFactory::create_resource(m2m_obj_list, 3201, 0, 5853, M2MResourceInstance::INTEGER, M2MBase::GET_PUT_ALLOWED);
    if (m2m_put_res->set_value(0) != true) {
        printf("m2m_put_res->set_value() failed\n");
        return -1;
    }
    if (m2m_put_res->set_value_updated_function(put_res_update) != true) {
        printf("m2m_put_res->set_value_updated_function() failed\n");
        return -1;
    }

    // POST resource 3201/0/5850
    m2m_post_res = M2MInterfaceFactory::create_resource(m2m_obj_list, 3201, 0, 5850, M2MResourceInstance::INTEGER, M2MBase::POST_ALLOWED);
    if (m2m_post_res->set_execute_function(execute_post) != true) {
        printf("m2m_post_res->set_execute_function() failed\n");
        return -1;
    }

    // GET resource 3313/0/5702 (Accelerometer X)
    lsm9ds1.readAccel();
    lsm9ds1.readGyro();
    lsm9ds1.readMag();
    m2m_lsm9ds1_accelerometer_x_res = M2MInterfaceFactory::create_resource(m2m_obj_list, 3313, 0, 5702, M2MResourceInstance::FLOAT, M2MBase::GET_ALLOWED);
    m2m_lsm9ds1_accelerometer_x_res->set_observable(true);
    if (m2m_lsm9ds1_accelerometer_x_res->set_value_float(lsm9ds1.calcAccel(lsm9ds1.ax)) != true) {
        printf("m2m_lsm9ds1_accelerometer_x_res->set_value_float() failed\n");
        return -1;
    }

    // GET resource 3313/0/5703 (Accelerometer Y)
    m2m_lsm9ds1_accelerometer_y_res = M2MInterfaceFactory::create_resource(m2m_obj_list, 3313, 0, 5703, M2MResourceInstance::FLOAT, M2MBase::GET_ALLOWED);
    m2m_lsm9ds1_accelerometer_y_res->set_observable(true);
    if (m2m_lsm9ds1_accelerometer_y_res->set_value_float(lsm9ds1.calcAccel(lsm9ds1.ay)) != true) {
        printf("m2m_lsm9ds1_accelerometer_y_res->set_value_float() failed\n");
        return -1;
    }

    // GET resource 3313/0/5704 (Accelerometer Z)
    m2m_lsm9ds1_accelerometer_z_res = M2MInterfaceFactory::create_resource(m2m_obj_list, 3313, 0, 5704, M2MResourceInstance::FLOAT, M2MBase::GET_ALLOWED);
    m2m_lsm9ds1_accelerometer_z_res->set_observable(true);
    if (m2m_lsm9ds1_accelerometer_z_res->set_value_float(lsm9ds1.calcAccel(lsm9ds1.az)) != true) {
        printf("m2m_lsm9ds1_accelerometer_z_res->set_value_float() failed\n");
        return -1;
    }

    // GET resource 3334/0/5702 (Gyrometer X)
    m2m_lsm9ds1_gyrometer_x_res = M2MInterfaceFactory::create_resource(m2m_obj_list, 3334, 0, 5702, M2MResourceInstance::FLOAT, M2MBase::GET_ALLOWED);
    m2m_lsm9ds1_gyrometer_x_res->set_observable(true);
    if (m2m_lsm9ds1_gyrometer_x_res->set_value_float(lsm9ds1.calcGyro(lsm9ds1.gx)) != true) {
        printf("m2m_lsm9ds1_gyrometer_x_res->set_value_float() failed\n");
        return -1;
    }

    // GET resource 3334/0/5703 (Gyrometer Y)
    m2m_lsm9ds1_gyrometer_y_res = M2MInterfaceFactory::create_resource(m2m_obj_list, 3334, 0, 5703, M2MResourceInstance::FLOAT, M2MBase::GET_ALLOWED);
    m2m_lsm9ds1_gyrometer_y_res->set_observable(true);
    if (m2m_lsm9ds1_gyrometer_y_res->set_value_float(lsm9ds1.calcGyro(lsm9ds1.gy)) != true) {
        printf("m2m_lsm9ds1_gyrometer_y_res->set_value_float() failed\n");
        return -1;
    }

    // GET resource 3334/0/5704 (Gyrometer Z)
    m2m_lsm9ds1_gyrometer_z_res = M2MInterfaceFactory::create_resource(m2m_obj_list, 3334, 0, 5704, M2MResourceInstance::FLOAT, M2MBase::GET_ALLOWED);
    m2m_lsm9ds1_gyrometer_z_res->set_observable(true);
    if (m2m_lsm9ds1_gyrometer_z_res->set_value_float(lsm9ds1.calcGyro(lsm9ds1.gz)) != true) {
        printf("m2m_lsm9ds1_gyrometer_z_res->set_value_float() failed\n");
        return -1;
    }

    // GET resource 3314/0/5702 (Magnetometer X)
    m2m_lsm9ds1_magnetometer_x_res = M2MInterfaceFactory::create_resource(m2m_obj_list, 3314, 0, 5702, M2MResourceInstance::FLOAT, M2MBase::GET_ALLOWED);
    m2m_lsm9ds1_magnetometer_x_res->set_observable(true);
    if (m2m_lsm9ds1_magnetometer_x_res->set_value_float(lsm9ds1.calcMag(lsm9ds1.mx)) != true) {
        printf("m2m_lsm9ds1_magnetometer_x_res->set_value_float() failed\n");
        return -1;
    }

    // GET resource 3314/0/5703 (Magnetometer Y)
    m2m_lsm9ds1_magnetometer_y_res = M2MInterfaceFactory::create_resource(m2m_obj_list, 3314, 0, 5703, M2MResourceInstance::FLOAT, M2MBase::GET_ALLOWED);
    m2m_lsm9ds1_magnetometer_y_res->set_observable(true);
    if (m2m_lsm9ds1_magnetometer_y_res->set_value_float(lsm9ds1.calcMag(lsm9ds1.my)) != true) {
        printf("m2m_lsm9ds1_magnetometer_y_res->set_value_float() failed\n");
        return -1;
    }

    // GET resource 3314/0/5704 (Magnetometer Z)
    m2m_lsm9ds1_magnetometer_z_res = M2MInterfaceFactory::create_resource(m2m_obj_list, 3314, 0, 5704, M2MResourceInstance::FLOAT, M2MBase::GET_ALLOWED);
    m2m_lsm9ds1_magnetometer_z_res->set_observable(true);
    if (m2m_lsm9ds1_magnetometer_z_res->set_value_float(lsm9ds1.calcMag(lsm9ds1.mz)) != true) {
        printf("m2m_lsm9ds1_magnetometer_z_res->set_value_float() failed\n");
        return -1;
    }

    // GET resource 3313/1/5702 (Accelerometer X)
    m2m_icm20602_accelerometer_x_res = M2MInterfaceFactory::create_resource(m2m_obj_list, 3313, 1, 5702, M2MResourceInstance::FLOAT, M2MBase::GET_ALLOWED);
    m2m_icm20602_accelerometer_x_res->set_observable(true);
    if (m2m_icm20602_accelerometer_x_res->set_value_float(normalize_acceleration_value(icm20602.getAccXvalue())) != true) {
        printf("m2m_icm20602_accelerometer_x_res->set_value_float() failed\n");
        return -1;
    }

    // GET resource 3313/1/5703 (Accelerometer Y)
    m2m_icm20602_accelerometer_y_res = M2MInterfaceFactory::create_resource(m2m_obj_list, 3313, 1, 5703, M2MResourceInstance::FLOAT, M2MBase::GET_ALLOWED);
    m2m_icm20602_accelerometer_y_res->set_observable(true);
    if (m2m_icm20602_accelerometer_y_res->set_value_float(normalize_acceleration_value(icm20602.getAccYvalue())) != true) {
        printf("m2m_icm20602_accelerometer_y_res->set_value_float() failed\n");
        return -1;
    }

    // GET resource 3313/1/5704 (Accelerometer Z)
    m2m_icm20602_accelerometer_z_res = M2MInterfaceFactory::create_resource(m2m_obj_list, 3313, 1, 5704, M2MResourceInstance::FLOAT, M2MBase::GET_ALLOWED);
    m2m_icm20602_accelerometer_z_res->set_observable(true);
    if (m2m_icm20602_accelerometer_z_res->set_value_float(normalize_acceleration_value(icm20602.getAccZvalue())) != true) {
        printf("m2m_icm20602_accelerometer_z_res->set_value_float() failed\n");
        return -1;
    }

    // GET resource 3334/1/5702 (Gyrometer X)
    m2m_icm20602_gyrometer_x_res = M2MInterfaceFactory::create_resource(m2m_obj_list, 3334, 1, 5702, M2MResourceInstance::FLOAT, M2MBase::GET_ALLOWED);
    m2m_icm20602_gyrometer_x_res->set_observable(true);
    if (m2m_icm20602_gyrometer_x_res->set_value_float(normalize_gyroscope_value(icm20602.getGyrXvalue())) != true) {
        printf("m2m_icm20602_gyrometer_x_res->set_value_float() failed\n");
        return -1;
    }

    // GET resource 3334/1/5703 (Gyrometer Y)
    m2m_icm20602_gyrometer_y_res = M2MInterfaceFactory::create_resource(m2m_obj_list, 3334, 1, 5703, M2MResourceInstance::FLOAT, M2MBase::GET_ALLOWED);
    m2m_icm20602_gyrometer_y_res->set_observable(true);
    if (m2m_icm20602_gyrometer_y_res->set_value_float(normalize_gyroscope_value(icm20602.getGyrYvalue())) != true) {
        printf("m2m_icm20602_gyrometer_y_res->set_value_float() failed\n");
        return -1;
    }

    // GET resource 3334/1/5704 (Gyrometer Z)
    m2m_icm20602_gyrometer_z_res = M2MInterfaceFactory::create_resource(m2m_obj_list, 3334, 1, 5704, M2MResourceInstance::FLOAT, M2MBase::GET_ALLOWED);
    m2m_icm20602_gyrometer_z_res->set_observable(true);
    if (m2m_icm20602_gyrometer_z_res->set_value_float(normalize_gyroscope_value(icm20602.getGyrZvalue())) != true) {
        printf("m2m_icm20602_gyrometer_z_res->set_value_float() failed\n");
        return -1;
    }

    // GET resource 3303/0/5700 (Temperature)
    si7021.measure();
    m2m_si7021_temperature_res = M2MInterfaceFactory::create_resource(m2m_obj_list, 3303, 0, 5700, M2MResourceInstance::FLOAT, M2MBase::GET_ALLOWED);
    m2m_si7021_temperature_res->set_observable(true);
    if (m2m_si7021_temperature_res->set_value_float(si7021.get_temperature() / 1000.0) != true) {
        printf("m2m_si7021_temperature_res->set_value_float() failed\n");
        return -1;
    }

    // GET resource 3304/0/5700 (Humidity)
    m2m_si7021_humidity_res = M2MInterfaceFactory::create_resource(m2m_obj_list, 3304, 0, 5700, M2MResourceInstance::FLOAT, M2MBase::GET_ALLOWED);
    m2m_si7021_humidity_res->set_observable(true);
    if (m2m_si7021_humidity_res->set_value_float(si7021.get_humidity() / 1000.0) != true) {
        printf("m2m_si7021_humidity_res->set_value_float() failed\n");
        return -1;
    }

    // GET resource 3303/1/5700 (Temperature)
    m2m_bme680_temperature_res = M2MInterfaceFactory::create_resource(m2m_obj_list, 3303, 1, 5700, M2MResourceInstance::FLOAT, M2MBase::GET_ALLOWED);
    m2m_bme680_temperature_res->set_observable(true);
    if (m2m_bme680_temperature_res->set_value_float(bme680->get_temperature()) != true) {
        printf("m2m_bme680_temperature_res->set_value_float() failed\n");
        return -1;
    }

    // GET resource 3304/1/5700 (Humidity)
    m2m_bme680_humidity_res = M2MInterfaceFactory::create_resource(m2m_obj_list, 3304, 1, 5700, M2MResourceInstance::FLOAT, M2MBase::GET_ALLOWED);
    m2m_bme680_humidity_res->set_observable(true);
    if (m2m_bme680_humidity_res->set_value_float(bme680->get_humidity()) != true) {
        printf("m2m_bme680_humidity_res->set_value_float() failed\n");
        return -1;
    }

    // GET resource 3315/0/5700 (Pressure)
    m2m_bme680_pressure_res = M2MInterfaceFactory::create_resource(m2m_obj_list, 3315, 0, 5700, M2MResourceInstance::FLOAT, M2MBase::GET_ALLOWED);
    m2m_bme680_pressure_res->set_observable(true);
    if (m2m_bme680_pressure_res->set_value_float(bme680->get_pressure() / 1000.0) != true) {
        printf("m2m_bme680_pressure_res->set_value_float() failed\n");
        return -1;
    }

    // GET resource 26243/0/26248 (Gas Resistance)
    m2m_bme680_gas_resistance_res = M2MInterfaceFactory::create_resource(m2m_obj_list, 26243, 0, 26248, M2MResourceInstance::FLOAT, M2MBase::GET_ALLOWED);
    m2m_bme680_gas_resistance_res->set_observable(true);
    if (m2m_bme680_gas_resistance_res->set_value_float(bme680->get_gas_resistance() / 1000.0) != true) {
        printf("m2m_bme680_gas_resistance_res->set_value_float() failed\n");
        return -1;
    }

    // GET resource 26243/0/26250 (CO2 Equivalents)
    m2m_bme680_co2_equivalent_res = M2MInterfaceFactory::create_resource(m2m_obj_list, 26243, 0, 26250, M2MResourceInstance::FLOAT, M2MBase::GET_ALLOWED);
    m2m_bme680_co2_equivalent_res->set_observable(true);
    if (m2m_bme680_co2_equivalent_res->set_value_float(bme680->get_co2_equivalent()) != true) {
        printf("m2m_bme680_co2_equivalent_res->set_value_float() failed\n");
        return -1;
    }

    // GET resource 26243/0/26251 (Breath-VOC Equivalents)
    m2m_bme680_breath_voc_equivalent_res = M2MInterfaceFactory::create_resource(m2m_obj_list, 26243, 0, 26251, M2MResourceInstance::FLOAT, M2MBase::GET_ALLOWED);
    m2m_bme680_breath_voc_equivalent_res->set_observable(true);
    if (m2m_bme680_breath_voc_equivalent_res->set_value_float(bme680->get_breath_voc_equivalent()) != true) {
        printf("m2m_bme680_breath_voc_equivalent_res->set_value_float() failed\n");
        return -1;
    }

    // GET resource 26243/0/26252 (IAQ Score)
    m2m_bme680_iaq_score_res = M2MInterfaceFactory::create_resource(m2m_obj_list, 26243, 0, 26252, M2MResourceInstance::FLOAT, M2MBase::GET_ALLOWED);
    m2m_bme680_iaq_score_res->set_observable(true);
    if (m2m_bme680_iaq_score_res->set_value_float(bme680->get_iaq_score()) != true) {
        printf("m2m_bme680_iaq_score_res->set_value_float() failed\n");
        return -1;
    }

    // GET resource 26243/0/26253 (IAQ Accuracy)
    m2m_bme680_iaq_accuracy_res = M2MInterfaceFactory::create_resource(m2m_obj_list, 26243, 0, 26253, M2MResourceInstance::INTEGER, M2MBase::GET_ALLOWED);
    m2m_bme680_iaq_accuracy_res->set_observable(true);
    if (m2m_bme680_iaq_accuracy_res->set_value(bme680->get_iaq_accuracy()) != true) {
        printf("m2m_bme680_iaq_accuracy_res->set_value() failed\n");
        return -1;
    }

    // GET resource 3330/0/5700 (Ranging Time-of-Flight Distance)
    uint32_t distance;
    retcode = vl53l0x.get_distance(&distance);
    m2m_vl53l0x_distance_res = M2MInterfaceFactory::create_resource(m2m_obj_list, 3330, 0, 5700, M2MResourceInstance::FLOAT, M2MBase::GET_ALLOWED);
    m2m_vl53l0x_distance_res->set_observable(true);
    switch (retcode) {
        case VL53L0X_ERROR_NONE:
            // Successful read
            if (m2m_vl53l0x_distance_res->set_value_float(distance / 1000.0) != true) {
                printf("m2m_vl53l0x_distance_res->set_value_float() failed\n");
                return -1;
            }
            break;
        case VL53L0X_ERROR_RANGE_ERROR:
            // Range error, return -1
            if (m2m_vl53l0x_distance_res->set_value_float(-1.0f) != true) {
                printf("m2m_vl53l0x_distance_res->set_value_float() failed\n");
                return -1;
            }
            break;
        default:
            break;
    }

    // GET/PUT resource 3311/0/5850 (LED)
    m2m_led_res = M2MInterfaceFactory::create_resource(m2m_obj_list, 3311, 0, 5850, M2MResourceInstance::BOOLEAN, M2MBase::GET_PUT_ALLOWED);
    m2m_led_res->set_observable(true);
    if (m2m_led_res->set_value(status_led == 0) != true) { // active low
        printf("m2m_led_res->set_value() failed\n");
        return -1;
    }

    if (m2m_led_res->set_value_updated_function(led_res_update) != true) {
        printf("m2m_led_res->set_value_updated_function() failed\n");
        return -1;
    }

    // POST resource 5000/0/1 to trigger deregister.
    m2m_deregister_res = M2MInterfaceFactory::create_resource(m2m_obj_list, 5000, 0, 1, M2MResourceInstance::INTEGER, M2MBase::POST_ALLOWED);

    // Use delayed response
    m2m_deregister_res->set_delayed_response(true);

    if (m2m_deregister_res->set_execute_function(deregister) != true) {
        printf("m2m_post_res->set_execute_function() failed\n");
        return -1;
    }

    // optional Device resource for running factory reset for the device. Path of this resource will be: 3/0/6.
    m2m_factory_reset_res = M2MInterfaceFactory::create_device()->create_resource(M2MDevice::FactoryReset);
    if (m2m_factory_reset_res) {
        m2m_factory_reset_res->set_execute_function(factory_reset);
    }

    printf("Register Pelion Device Management Client\n\n");

#ifdef MBED_CLOUD_CLIENT_SUPPORT_UPDATE
    cloud_client = new MbedCloudClient(client_registered, client_unregistered, client_error, NULL, update_progress);
#else
    cloud_client = new MbedCloudClient(client_registered, client_unregistered, client_error);
#endif // MBED_CLOUD_CLIENT_SUPPORT_UPDATE

    cloud_client->add_objects(m2m_obj_list);
    cloud_client->setup(network); // cloud_client->setup(NULL); -- https://jira.arm.com/browse/IOTCLT-3114

    t.start(callback(&queue, &EventQueue::dispatch_forever));
    queue.call_every(30000, update_resources);

    // Flush the stdin buffer before reading from it
    flush_stdin_buffer();

    while(cloud_client_running) {
        int in_char = getchar();
        if (in_char == 'i') {
            print_client_ids(); // When 'i' is pressed, print endpoint info
            continue;
        } else if (in_char == 'r') {
            (void) fcc_storage_delete(); // When 'r' is pressed, erase storage and reboot the board.
            printf("Storage erased, rebooting the device.\n\n");
            ThisThread::sleep_for(1*1000);
            NVIC_SystemReset();
        } else if (in_char > 0 && in_char != 0x03) { // Ctrl+C is 0x03 in Mbed OS and Linux returns negative number
            value_increment(); // Simulate button press
            continue;
        }
        deregister_client();
        break;
    }
    return 0;
}

#endif /* MBED_TEST_MODE */
