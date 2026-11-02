import {temperature, humidity, co2, illuminance} from 'zigbee-herdsman-converters/lib/modernExtend';

export default {
    zigbeeModel: ['ESP32H2_CO2'],
    model: 'ESP32H2_CO2',
    vendor: 'Custom',
    description: 'ESP32-H2 SCD40 CO2 sensor',
    extend: [
        temperature(),
        humidity(),
        co2(),
        illuminance(),
    ],
    ota: true,
};