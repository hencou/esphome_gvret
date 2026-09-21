# ESPHome GVRET CAN bus sniffer – wireless SavvyCAN adapter for ESP32

**esphome_gvret** is an [ESPHome](https://esphome.io) external component that turns an ESP32 into a **Wi-Fi CAN bus sniffer and logger** using the **GVRET network protocol**. It lets you monitor, log and send CAN bus frames remotely with **[SavvyCAN](https://savvycan.com)** – no USB cable, no dedicated hardware like the EVTV ESP32RET or Macchina M2 needed.

Typical use cases: reverse engineering automotive / OBD-II CAN networks, debugging CANopen and industrial CAN devices, monitoring heat pumps, boilers, solar inverters, e-bikes or any other CAN-based system from your desk over Wi-Fi.

## Features

- **GVRET over TCP/IP (Wi-Fi)** – compatible with SavvyCAN's *Network Connection* (default TCP port `23`)
- **UDP auto-discovery broadcast** (port `17222`) so SavvyCAN finds the device automatically on your LAN
- **Multiple CAN buses** – attach one or more ESPHome `canbus` instances (e.g. `esp32_can` / TWAI, MCP2515)
- **Bidirectional** – receive (sniff) *and* transmit CAN frames from SavvyCAN
- **Time synchronisation** with an optional ESPHome `time` component for accurate frame timestamps
- Runs alongside the normal ESPHome features (Home Assistant API, OTA updates, logging), so the same ESP32 can also expose CAN data as sensors
- Based on the original [GVRET firmware by Collin Kidder](https://github.com/collin80/GVRET)

## Hardware

Any ESP32 with a CAN transceiver works. The example uses the **Olimex ESP32-EVB**, which has a complete CAN bus interface (transceiver + terminal block) on board, so no extra components are required. Other options: an ESP32 dev board plus an SN65HVD230 / TJA1050 transceiver, or an MCP2515 SPI module.

## Installation

Add the component to your ESPHome YAML via `external_components`:

```yaml
external_components:
  - source: github://hencou/esphome_gvret
    components: [canbus_gvret]
```

## Example configuration

```yaml
esphome:
  name: gvret
  friendly_name: GVRET

esp32:
  board: esp32dev
  framework:
    type: arduino

external_components:
  - source: github://hencou/esphome_gvret
    components: [canbus_gvret]

api:

ota:
  - platform: esphome

logger:
  level: WARN

wifi:
  ssid: !secret wifi_ssid
  password: !secret wifi_password

# Olimex ESP32-EVB on-board CAN interface
canbus:
  - platform: esp32_can
    id: can1
    can_id: 100
    rx_pin: GPIO35
    tx_pin: GPIO5
    bit_rate: 1000KBPS

canbus_gvret:
  can:
    - canbus_id: can1
```

See [`gvret_example.yaml`](gvret_example.yaml) for the complete file.

### Configuration variables

| Option    | Required | Description                                                                 |
|-----------|----------|-----------------------------------------------------------------------------|
| `can`     | yes      | List of `canbus_id` entries; each attached CAN bus is exposed to SavvyCAN   |
| `time_id` | no       | ID of an ESPHome `time` component used to timestamp frames                  |

## Connecting SavvyCAN

1. Flash the ESP32 with ESPHome and let it join your Wi-Fi network.
2. In SavvyCAN open **Connection → Open Connection Window → Add New Device Connection**.
3. Choose **Network Connection** and enter the IP address (or hostname) of the ESP32, port `23`. Thanks to the UDP broadcast the device usually shows up automatically.
4. Click **Create New Connection** – CAN frames now stream into SavvyCAN in real time.

## Related projects

- [collin80/GVRET](https://github.com/collin80/GVRET) – original GVRET firmware (Arduino Due)
- [collin80/ESP32RET](https://github.com/collin80/ESP32RET) – GVRET for the EVTV ESP32RET board
- [collin80/SavvyCAN](https://github.com/collin80/SavvyCAN) – cross-platform CAN bus analysis tool
- [ESPHome CAN bus component](https://esphome.io/components/canbus.html)

## Keywords

ESPHome, ESP32, GVRET, SavvyCAN, CAN bus, CANbus sniffer, CAN logger, CAN analyzer, Wi-Fi CAN adapter, wireless CAN, TWAI, MCP2515, OBD-II, automotive, reverse engineering, Home Assistant, Olimex ESP32-EVB
