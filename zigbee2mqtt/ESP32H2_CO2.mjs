import * as exposes from 'zigbee-herdsman-converters/lib/exposes';
import * as m from 'zigbee-herdsman-converters/lib/modernExtend';
import {Zcl} from 'zigbee-herdsman';

const e = exposes.presets;
const ea = exposes.access;
const calibrationClusterName = 'esp32h2Co2Calibration';
const calibrationCluster = {
    name: calibrationClusterName,
    ID: 0xFC00,
    attributes: {
        temperatureCalibration: {name: 'temperatureCalibration', ID: 0x0001, type: Zcl.DataType.INT16, write: true, min: -50, max: 50},
        humidityCalibration: {name: 'humidityCalibration', ID: 0x0002, type: Zcl.DataType.INT16, write: true, min: -30, max: 30},
    },
    commands: {},
    commandsResponse: {},
};

const temperatureConverter = {
    cluster: 'msTemperatureMeasurement',
    type: ['attributeReport', 'readResponse'],
    convert: (model, msg, publish, options) => {
        const raw = msg.data.measuredValue;
        if (typeof raw !== 'number' || raw === 0x8000) return;

        const temperatureC = raw / 100;
        return {
            temperature_celsius: Math.round(temperatureC * 100) / 100,
            temperature_fahrenheit: Math.round((temperatureC * 9 / 5 + 32) * 100) / 100,
        };
    },
};

const humidityConverter = {
    cluster: 'msRelativeHumidity',
    type: ['attributeReport', 'readResponse'],
    convert: (model, msg, publish, options) => {
        const raw = msg.data.measuredValue;
        if (typeof raw !== 'number' || raw === 0xffff) return;

        return {humidity: Math.round(raw) / 100};
    },
};

const temperatureDisplayModeConverter = {
    cluster: 'hvacUserInterfaceCfg',
    type: ['attributeReport', 'readResponse'],
    convert: (model, msg) => {
        const raw = msg.data.tempDisplayMode;
        if (raw !== 0 && raw !== 1) return;
        const mode = raw === 1 ? 'fahrenheit' : 'celsius';
        return {temperature_display_mode: mode};
    },
};

const setTemperatureDisplayMode = {
    key: ['temperature_display_mode'],
    convertSet: async (entity, key, value) => {
        if (value !== 'celsius' && value !== 'fahrenheit') {
            throw new Error(`Unsupported temperature display mode: ${value}`);
        }
        const raw = value === 'fahrenheit' ? 1 : 0;
        await entity.write('hvacUserInterfaceCfg', {tempDisplayMode: raw});
        return {state: {temperature_display_mode: value}};
    },
    convertGet: async (entity) => {
        await entity.read('hvacUserInterfaceCfg', ['tempDisplayMode']);
    },
};

const calibrationConverter = {
    cluster: calibrationClusterName,
    type: ['attributeReport', 'readResponse'],
    convert: (model, msg) => {
        const state = {};
        if (typeof msg.data.temperatureCalibration === 'number')
            state.device_temperature_calibration = msg.data.temperatureCalibration / 10;
        if (typeof msg.data.humidityCalibration === 'number')
            state.device_humidity_calibration = msg.data.humidityCalibration;
        return Object.keys(state).length ? state : undefined;
    },
};

const setCalibration = {
    key: ['device_temperature_calibration', 'device_humidity_calibration'],
    convertSet: async (entity, key, value) => {
        const numeric = Number(value);
        if (!Number.isFinite(numeric)) throw new Error(`${key} must be numeric`);
        if (key === 'device_temperature_calibration') {
            if (numeric < -5 || numeric > 5) throw new Error('Temperature calibration must be between -5 and +5 °C');
            const raw = Math.round(numeric * 10);
            await entity.write(calibrationClusterName, {temperatureCalibration: raw});
            return {state: {device_temperature_calibration: raw / 10}};
        }
        if (numeric < -30 || numeric > 30) throw new Error('Humidity calibration must be between -30 and +30 percentage points');
        const raw = Math.round(numeric);
        await entity.write(calibrationClusterName, {humidityCalibration: raw});
        return {state: {device_humidity_calibration: raw}};
    },
    convertGet: async (entity, key) => {
        const attr = key === 'device_temperature_calibration' ? 'temperatureCalibration' : 'humidityCalibration';
        await entity.read(calibrationClusterName, [attr]);
    },
};

export default {
    zigbeeModel: ['ESP32H2_CO2'],
    model: 'ESP32H2_CO2',
    vendor: 'Custom',
    description: 'ESP32-H2 SCD40 CO2 sensor',
    fromZigbee: [temperatureConverter, humidityConverter, temperatureDisplayModeConverter, calibrationConverter],
    toZigbee: [setTemperatureDisplayMode, setCalibration],
    extend: [m.co2(), m.illuminance(), m.deviceAddCustomCluster(calibrationClusterName, calibrationCluster)],
    exposes: () => {
        return [
            e.numeric('temperature_celsius', ea.STATE)
                .withLabel('Temperature Celsius').withUnit('°C')
                .withDescription('Temperature converted to Celsius. Use this sensor when Celsius is selected.')
                // Keep the reported unit fixed; a temperature device class
                // would let Home Assistant convert it to the preferred unit.
                .withHomeAssistant({icon: 'mdi:thermometer'}),
            e.numeric('temperature_fahrenheit', ea.STATE)
                .withLabel('Temperature Fahrenheit').withUnit('°F')
                .withDescription('Temperature converted to Fahrenheit. Use this sensor when Fahrenheit is selected.')
                // Do not set the temperature device class: Home Assistant
                // would convert this fixed °F value to the preferred unit.
                .withHomeAssistant({icon: 'mdi:thermometer'}),
            e.humidity().withHomeAssistant({icon: 'mdi:water-percent'}),
            e.enum('temperature_display_mode', ea.ALL, ['celsius', 'fahrenheit'])
                .withLabel('Temperature unit')
                .withDescription('Unit used by the device display.')
                .withHomeAssistant({icon: 'mdi:temperature-celsius'}),
            e.numeric('device_temperature_calibration', ea.STATE_SET)
                .withValueMin(-5).withValueMax(5).withValueStep(0.1).withUnit('°C')
                .withCategory('config').withLabel('Temperature calibration')
                .withDescription('Absolute offset applied on the device and shown on its display.')
                .withHomeAssistant({icon: 'mdi:wrench-clock'}),
            e.numeric('device_humidity_calibration', ea.STATE_SET)
                .withValueMin(-30).withValueMax(30).withValueStep(1).withUnit('%')
                .withCategory('config').withLabel('Humidity calibration')
                .withDescription('Absolute offset in percentage points, applied on the device and shown on its display.')
                .withHomeAssistant({icon: 'mdi:wrench-clock'}),
        ];
    },
    configure: async (device, coordinatorEndpoint, definition) => {
        const configureTemperatureReporting = m.setupConfigureForReporting('msTemperatureMeasurement', 'measuredValue', {
            config: {min: '10_SECONDS', max: '1_HOUR', change: 100},
            access: ea.GET,
        });
        const configureHumidityReporting = m.setupConfigureForReporting('msRelativeHumidity', 'measuredValue', {
            config: {min: '10_SECONDS', max: '1_HOUR', change: 100},
            access: ea.GET,
        });
        await configureTemperatureReporting(device, coordinatorEndpoint, definition);
        await configureHumidityReporting(device, coordinatorEndpoint, definition);
        const endpoint = device.getEndpoint(1);
        if (endpoint) {
            await endpoint.read('hvacUserInterfaceCfg', ['tempDisplayMode']);
            await endpoint.read(calibrationClusterName, ['temperatureCalibration', 'humidityCalibration']);
        }
    },
    ota: true,
};
