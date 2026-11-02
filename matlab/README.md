# MATLAB plotting for the PX4 / ROS2 drone simulation

1. Start the simulation (`scripts/run/auto_start.bash`).
2. In another terminal, start the logger **before** the drone app:
   ```
   cd ~/px4_drone_simulation_ws && . ./install/setup.bash
   python3 <repo>/matlab/tools/telemetry_logger.py --out ~/drone_logs
   ```
3. Run the drone: `./build/drone/drone`. When it finishes, Ctrl+C the logger.
4. In MATLAB (R2019b+; `exportgraphics` needs R2020a+):
   ```matlab
   cd <repo>/matlab
   plot_drone_behavior('~/drone_logs')          % or add ,true to save PNGs
   plot_drone_behavior()                        % demo data, no logs needed
   ```
Plots: 3-D trajectory, ground track, altitude/speed, attitude, body rates,
battery, arming/flight-mode timeline. No MATLAB toolboxes required.
