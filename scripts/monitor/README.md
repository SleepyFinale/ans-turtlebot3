# Raspberry Pi Bottleneck Monitor

This monitoring workflow has two scripts:

- `pi_bottleneck_monitor.sh`: collects runtime system and network metrics into CSV.
- `analyze_log.py`: scans the CSV and reports potential bottlenecks.
- `plot_bottleneck_metric.py`: plots any selected numeric metric over time.

## Why this is useful for ROS 2 robots

ROS 2 navigation/control can degrade when the Pi is thermally constrained, CPU saturated, memory/swap pressured, undervolted, or network-limited. These scripts help you capture that evidence during a run and summarize likely problems afterward.

## Collected metrics

- CPU usage (%), load averages.
- CPU temperature (C) and Pi throttling flags (`vcgencmd get_throttled`).
- Memory usage (%), available RAM (MB), swap usage (%).
- Root disk usage (%).
- Interface throughput (RX/TX Bps and Mbps).
- Bandwidth utilization (% of link speed).
- Packet errors/drops per second.
- Link state context (SSID/IP/connected).

## Quick start

From repository root (`/home/pinky/turtlebot3_ws`):

```bash
./scripts/monitor/pi_bottleneck_monitor.sh --iface wlan0 --interval 1.0
```

Press `Ctrl+C` to stop. The script prints the CSV path when it exits.

Then analyze (defaults to newest CSV and newest ROS `.log` in `scripts/monitor/logs`):

```bash
./scripts/monitor/analyze_log.py
```

Or analyze an explicit CSV:

```bash
./scripts/monitor/analyze_log.py scripts/monitor/logs/pi_bottleneck_YYYYmmdd_HHMMSS.csv
```

Optional JSON report:

```bash
./scripts/monitor/analyze_log.py scripts/monitor/logs/pi_bottleneck_YYYYmmdd_HHMMSS.csv --json-output scripts/monitor/logs/last_report.json
```

## Optional monitor with robot bringup

Use the wrapper below to keep your normal bringup flow, and optionally enable monitoring for that same session:

```bash
# Same behavior as plain bringup (monitor off)
./scripts/monitor/robot_program_monitor.sh

# Bringup + monitor together
./scripts/monitor/robot_program_monitor.sh --with-monitor

# Bringup + monitor + launch args
./scripts/monitor/robot_program_monitor.sh --with-monitor -- robot_name:=pinky

# Nav2 SLAM + monitor + pipeline timing capture
./scripts/monitor/robot_program_monitor.sh --with-monitor --launch-package turtlebot3_navigation2 --launch-file navigation2_slam.launch.py
```

You can also pass monitor options through the wrapper (for example `--iface eth0`, `--interval 0.5`, or `--output ...`).
When `--with-monitor` is enabled, the wrapper also saves ROS launch console output to `scripts/monitor/logs/ros_launch_*.log` (or `--ros-log-output <path>` if set).

## Collector options

```bash
./scripts/monitor/pi_bottleneck_monitor.sh --help
```

- `--iface <name>`: interface to monitor (default `wlan0`).
- `--interval <seconds>`: sample period (default `1.0`).
- `--output <path>`: explicit CSV path.
- `--link-mbps <value>`: override link speed if autodetect is wrong.

## Analyzer options

```bash
./scripts/monitor/analyze_log.py --help
```

- `--sustained-seconds`: window for sustained-threshold alerts (default `10`).
- `--json-output`: path for structured output.
- `--ros-log`: optional ROS launch log to analyze timing/pipeline issues.

Example with ROS timing analysis:

```bash
./scripts/monitor/analyze_log.py scripts/monitor/logs/pi_bottleneck_YYYYmmdd_HHMMSS.csv --ros-log scripts/monitor/logs/ros_launch_YYYYmmdd_HHMMSS.log
```

## Plot metric over time

List available numeric metrics in a CSV:

```bash
./scripts/monitor/plot_bottleneck_metric.py pi_bottleneck_YYYYmmdd_HHMMSS.csv --list-metrics
```

Create a plot (PNG):

```bash
./scripts/monitor/plot_bottleneck_metric.py pi_bottleneck_YYYYmmdd_HHMMSS.csv --metric cpu_usage_pct
```

Optional smoothing (rolling average window in samples):

```bash
./scripts/monitor/plot_bottleneck_metric.py pi_bottleneck_YYYYmmdd_HHMMSS.csv --metric cpu_usage_pct --rolling-window 5
```

## Default threshold policy

The analyzer uses sensible defaults for Raspberry Pi robot workloads:

- CPU usage warning/critical: `85% / 95%` sustained.
- CPU temp warning/critical: `75C / 82C` sustained.
- Memory usage warning/critical: `85% / 95%` sustained.
- Swap usage warning/critical: `25% / 60%` sustained.
- Disk usage warning/critical: `90% / 97%` sustained.
- Network utilization warning/critical: `80% / 95%` peak.
- Packet errors+drops warning/critical: `1/s / 5/s` peak.
- Any undervoltage or throttling flag is treated as critical.

## Typical workflow during field tests

1. Launch with wrapper and monitor enabled.
2. Run mission (teleop/nav2/autonomy).
3. Stop launch with `Ctrl+C` (wrapper stops monitor too).
4. Run analyzer on monitor CSV and the ROS launch log.
5. If warnings/critical events appear, correlate with bags and topic rates.

## Troubleshooting notes

- If `vcgencmd` is unavailable, temp falls back to thermal zone.
- If `ethtool` cannot detect link speed, default link speed is `100 Mbps` unless overridden.
- For Ethernet testing, use `--iface eth0`.
- If your Wi-Fi bursts exceed defaults, pass `--link-mbps` with realistic negotiated speed.
