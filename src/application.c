#include <application.h>

#define MAX_SOIL_SENSORS                    5

#define SERVICE_MODE_INTERVAL               (15 * 60 * 1000)
#define BATTERY_UPDATE_INTERVAL             (60 * 60 * 1000)

#define TEMPERATURE_PUB_INTERVAL            (15 * 60 * 1000)
#define TEMPERATURE_PUB_DIFFERENCE          1.0f
#define TEMPERATURE_UPDATE_SERVICE_INTERVAL (1 * 1000)
#define TEMPERATURE_UPDATE_NORMAL_INTERVAL  (10 * 1000)

#define SENSOR_UPDATE_SERVICE_INTERVAL      (15 * 1000)
#define SENSOR_UPDATE_NORMAL_INTERVAL       (5 * 60 * 1000)

#define UPDATE_SERVICE_INTERVAL            (5 * 1000)
#define UPDATE_NORMAL_INTERVAL             (10 * 1000)

#define BAROMETER_UPDATE_SERVICE_INTERVAL  (1 * 60 * 1000)
#define BAROMETER_UPDATE_NORMAL_INTERVAL   (5 * 60 * 1000)

#define TEMPERATURE_TAG_PUB_NO_CHANGE_INTEVAL (15 * 60 * 1000)
#define TEMPERATURE_TAG_PUB_VALUE_CHANGE 0.2f

#define HUMIDITY_TAG_PUB_NO_CHANGE_INTEVAL (15 * 60 * 1000)
#define HUMIDITY_TAG_PUB_VALUE_CHANGE 5.0f

#define LUX_METER_TAG_PUB_NO_CHANGE_INTEVAL (15 * 60 * 1000)
#define LUX_METER_TAG_PUB_VALUE_CHANGE 25.0f

#define BAROMETER_TAG_PUB_NO_CHANGE_INTEVAL (15 * 60 * 1000)
#define BAROMETER_TAG_PUB_VALUE_CHANGE 20.0f

struct {
    event_param_t temperature;
    event_param_t humidity;
    event_param_t illuminance;
    event_param_t pressure;

} params;

// LED instance
twr_led_t led;
// Button instance
twr_button_t button;
// Thermometer instance
twr_tmp112_t tmp112;
// Soil sensor instance
twr_soil_sensor_t soil_sensor;
// Sensors array
twr_soil_sensor_sensor_t sensors[MAX_SOIL_SENSORS];

void button_event_handler(twr_button_t *self, twr_button_event_t event, void *event_param)
{
    // Counters for button events
    static uint16_t button_click_count = 0;
    static uint16_t button_hold_count = 0;

    if (event == TWR_BUTTON_EVENT_CLICK)
    {
        // Pulse LED for 100 milliseconds
        twr_led_pulse(&led, 100);

        // Increment press count
        button_click_count++;

        // Publish button message on radio
        twr_radio_pub_push_button(&button_click_count);
    }
    else if (event == TWR_BUTTON_EVENT_HOLD)
    {
        // Pulse LED for 250 milliseconds
        twr_led_pulse(&led, 250);

        // Increment hold count
        button_hold_count++;
        // Publish message on radio
        twr_radio_pub_event_count(TWR_RADIO_PUB_EVENT_HOLD_BUTTON, &button_hold_count);
    }
}

void battery_event_handler(twr_module_battery_event_t event, void *event_param)
{
    (void) event;
    (void) event_param;

    float voltage;

    if (event == TWR_MODULE_BATTERY_EVENT_UPDATE)
    {
        if (twr_module_battery_get_voltage(&voltage))
        {
            twr_radio_pub_battery(&voltage);
        }
    }
}

void tmp112_event_handler(twr_tmp112_t *self, twr_tmp112_event_t event, void *event_param)
{
    // Time of next report
    static twr_tick_t tick_report = 0;

    // Last value used for change comparison
    static float last_published_temperature = NAN;

    if (event == TWR_TMP112_EVENT_UPDATE)
    {
        float temperature;

        if (twr_tmp112_get_temperature_celsius(self, &temperature))
        {
            // Implicitly do not publish message on radio
            bool publish = false;

            // Is time up to report temperature?
            if (twr_tick_get() >= tick_report)
            {
                // Publish message on radio
                publish = true;
            }

            // Is temperature difference from last published value significant?
            if (fabsf(temperature - last_published_temperature) >= TEMPERATURE_PUB_DIFFERENCE)
            {
                // Publish message on radio
                publish = true;
            }

            if (publish)
            {
                // Publish temperature message on radio
                twr_radio_pub_temperature(TWR_RADIO_PUB_CHANNEL_R1_I2C0_ADDRESS_ALTERNATE, &temperature);

                // Schedule next temperature report
                tick_report = twr_tick_get() + TEMPERATURE_PUB_INTERVAL;

                // Remember last published value
                last_published_temperature = temperature;
            }
        }
    }
}

void soil_sensor_event_handler(twr_soil_sensor_t *self, uint64_t device_address, twr_soil_sensor_event_t event, void *event_param)
{
    static char topic[64];
    uint8_t *b = (uint8_t*)&device_address;

    if (event == TWR_SOIL_SENSOR_EVENT_UPDATE)
    {
        int index = twr_soil_sensor_get_index_by_device_address(self, device_address);

        if (index < 0)
        {
            return;
        }

        float temperature;

        if (twr_soil_sensor_get_temperature_celsius(self, device_address, &temperature))
        {
            snprintf(topic, sizeof(topic), "soil-sensor/%02x%02x%02x%02x%02x%02x%02x%02x/temperature", b[7], b[6], b[5], b[4], b[3], b[2], b[1], b[0]);
            //snprintf(topic, sizeof(topic), "soil-sensor/%llx/temperature", device_address);

            // Publish temperature message on radio
            twr_radio_pub_float(topic, &temperature);
        }

        uint16_t raw_cap_u16;

        if (twr_soil_sensor_get_cap_raw(self, device_address, &raw_cap_u16))
        {
            snprintf(topic, sizeof(topic), "soil-sensor/%02x%02x%02x%02x%02x%02x%02x%02x/raw", b[7], b[6], b[5], b[4], b[3], b[2], b[1], b[0]);
            //snprintf(topic, sizeof(topic), "soil-sensor/%llx/raw", device_address);

            // Publish raw capacitance value message on radio
            int raw_cap = (int)raw_cap_u16;
            twr_radio_pub_int(topic, &raw_cap);

            /*
            // Experimental - send percent moisture value based on sensor calibration
            int moisture;
            twr_soil_sensor_get_moisture(self, device_address, &moisture);
            snprintf(topic, sizeof(topic), "soil-sensor/%llx/moisture", device_address);
            twr_radio_pub_int(topic, &moisture);
            */
        }
    }
    else if (event == TWR_SOIL_SENSOR_EVENT_ERROR)
    {
        int error = twr_soil_sensor_get_error(self);
        twr_radio_pub_int("soil-sensor/-/error", &error);
    }
}

void climate_module_event_handler(twr_module_climate_event_t event, void *event_param)
{
    (void) event_param;

    float value;

    if (event == TWR_MODULE_CLIMATE_EVENT_UPDATE_THERMOMETER)
    {
        if (twr_module_climate_get_temperature_celsius(&value))
        {
            if ((fabs(value - params.temperature.value) >= TEMPERATURE_TAG_PUB_VALUE_CHANGE) || (params.temperature.next_pub < twr_scheduler_get_spin_tick()))
            {
                twr_radio_pub_temperature(TWR_RADIO_PUB_CHANNEL_R1_I2C0_ADDRESS_DEFAULT, &value);
                params.temperature.value = value;
                params.temperature.next_pub = twr_scheduler_get_spin_tick() + TEMPERATURE_TAG_PUB_NO_CHANGE_INTEVAL;
            }
        }
    }
    else if (event == TWR_MODULE_CLIMATE_EVENT_UPDATE_HYGROMETER)
    {
        if (twr_module_climate_get_humidity_percentage(&value))
        {
            if ((fabs(value - params.humidity.value) >= HUMIDITY_TAG_PUB_VALUE_CHANGE) || (params.humidity.next_pub < twr_scheduler_get_spin_tick()))
            {
                twr_radio_pub_humidity(TWR_RADIO_PUB_CHANNEL_R3_I2C0_ADDRESS_DEFAULT, &value);
                params.humidity.value = value;
                params.humidity.next_pub = twr_scheduler_get_spin_tick() + HUMIDITY_TAG_PUB_NO_CHANGE_INTEVAL;
            }
        }
    }
    else if (event == TWR_MODULE_CLIMATE_EVENT_UPDATE_LUX_METER)
    {
        if (twr_module_climate_get_illuminance_lux(&value))
        {
            if (value < 1)
            {
                value = 0;
            }
            if ((fabs(value - params.illuminance.value) >= LUX_METER_TAG_PUB_VALUE_CHANGE) || (params.illuminance.next_pub < twr_scheduler_get_spin_tick()) ||
                    ((value == 0) && (params.illuminance.value != 0)) || ((value > 1) && (params.illuminance.value == 0)))
            {
                twr_radio_pub_luminosity(TWR_RADIO_PUB_CHANNEL_R1_I2C0_ADDRESS_DEFAULT, &value);
                params.illuminance.value = value;
                params.illuminance.next_pub = twr_scheduler_get_spin_tick() + LUX_METER_TAG_PUB_NO_CHANGE_INTEVAL;
            }
        }
    }
    else if (event == TWR_MODULE_CLIMATE_EVENT_UPDATE_BAROMETER)
    {
        if (twr_module_climate_get_pressure_pascal(&value))
        {
            if ((fabs(value - params.pressure.value) >= BAROMETER_TAG_PUB_VALUE_CHANGE) || (params.pressure.next_pub < twr_scheduler_get_spin_tick()))
            {
                float meter;

                if (!twr_module_climate_get_altitude_meter(&meter))
                {
                    return;
                }

                twr_radio_pub_barometer(TWR_RADIO_PUB_CHANNEL_R1_I2C0_ADDRESS_DEFAULT, &value, &meter);
                params.pressure.value = value;
                params.pressure.next_pub = twr_scheduler_get_spin_tick() + BAROMETER_TAG_PUB_NO_CHANGE_INTEVAL;
            }
        }
    }
}


void switch_to_normal_mode_task(void *param)
{
    twr_tmp112_set_update_interval(&tmp112, TEMPERATURE_UPDATE_NORMAL_INTERVAL);

    twr_soil_sensor_set_update_interval(&soil_sensor, SENSOR_UPDATE_NORMAL_INTERVAL);

    twr_module_climate_set_update_interval_thermometer(UPDATE_NORMAL_INTERVAL);
    twr_module_climate_set_update_interval_hygrometer(UPDATE_NORMAL_INTERVAL);
    twr_module_climate_set_update_interval_lux_meter(UPDATE_NORMAL_INTERVAL);
    twr_module_climate_set_update_interval_barometer(BAROMETER_UPDATE_NORMAL_INTERVAL);

    twr_scheduler_unregister(twr_scheduler_get_current_task_id());
}

void application_init(void)
{
    twr_log_init(TWR_LOG_LEVEL_DUMP, TWR_LOG_TIMESTAMP_ABS);

    // Initialize LED
    twr_led_init(&led, TWR_GPIO_LED, false, false);

    // Initialize button
    twr_button_init(&button, TWR_GPIO_BUTTON, TWR_GPIO_PULL_DOWN, false);
    twr_button_set_event_handler(&button, button_event_handler, NULL);

    // Initialize thermometer sensor on core module
    twr_tmp112_init(&tmp112, TWR_I2C_I2C0, 0x49);
    twr_tmp112_set_event_handler(&tmp112, tmp112_event_handler, NULL);
    twr_tmp112_set_update_interval(&tmp112, TEMPERATURE_UPDATE_SERVICE_INTERVAL);

    // Initialize soil sensor
    twr_soil_sensor_init_multiple(&soil_sensor, sensors, 5);
    twr_soil_sensor_set_event_handler(&soil_sensor, soil_sensor_event_handler, NULL);
    twr_soil_sensor_set_update_interval(&soil_sensor, SENSOR_UPDATE_SERVICE_INTERVAL);

    // Initialize climate module
    twr_module_climate_init();
    twr_module_climate_set_event_handler(climate_module_event_handler, NULL);
    twr_module_climate_set_update_interval_thermometer(UPDATE_SERVICE_INTERVAL);
    twr_module_climate_set_update_interval_hygrometer(UPDATE_SERVICE_INTERVAL);
    twr_module_climate_set_update_interval_lux_meter(UPDATE_SERVICE_INTERVAL);
    twr_module_climate_set_update_interval_barometer(BAROMETER_UPDATE_SERVICE_INTERVAL);
    twr_module_climate_measure_all_sensors();

    // Initialize battery
    twr_module_battery_init();
    twr_module_battery_set_event_handler(battery_event_handler, NULL);
    twr_module_battery_set_update_interval(BATTERY_UPDATE_INTERVAL);

    // Initialize radio
    twr_radio_init(TWR_RADIO_MODE_NODE_SLEEPING);
    twr_radio_pairing_request("balcony-monitor", FW_VERSION);

    twr_scheduler_register(switch_to_normal_mode_task, NULL, SERVICE_MODE_INTERVAL);

    twr_led_pulse(&led, 2000);
}