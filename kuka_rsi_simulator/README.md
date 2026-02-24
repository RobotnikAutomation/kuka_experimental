# KUKA RSI Simulation / Mock

This package provides a UDP mock of a KUKA RSI robot.

The mock:
- Sends `<Rob>` frames to the RSI hardware interface endpoint.
- Receives `<Sen>` corrections (`RKorr` and `AK`).
- Integrates corrections into simulated cartesian and joint state.
- Publishes raw XML on:
  - `/<node_name>/rsi/state`
  - `/<node_name>/rsi/command`

## Launch

```bash
roslaunch kuka_rsi_simulator kuka_robot_mock.launch
```

Compatible launch (legacy name):

```bash
roslaunch kuka_rsi_simulator kuka_rsi_simulator.launch
```

Useful args:
- `rsi_hw_iface_ip` (default: `127.0.0.1`)
- `rsi_hw_iface_port` (default: `49152`)
- `cycle_time` (default: `0.004`)
- `socket_timeout` (default: `1.0`)
- `log_raw_xml` (default: `false`)
