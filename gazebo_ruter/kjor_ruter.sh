#!/usr/bin/env bash
# Øvre stripe vasket rute for rute (1,0 x 0,5 m): liften står i ro mens armen
# vasker hver rute, og flytter seg bare mellom rutene.
# Krever samme oppstart som kjor_raster.sh: start_gazebo.sh, deretter
# kjor_lasttest.sh slik at roboten står i startstillingen.
set -e
cd -- "$(dirname -- "${BASH_SOURCE[0]}")"
source /opt/ros/jazzy/setup.bash
source install/setup.bash
bane="$PWD/src/lift_window_sim/trajectories/${1:-patches.csv}"
if [ ! -f "$bane" ]; then
  echo "Finner ikke $bane. Lag den med:" >&2
  echo "  PYTHONPATH=src/lift_window_sim python3 -m lift_window_sim.plan_patches src/lift_window_sim" >&2
  exit 1
fi
if ros2 control -h >/dev/null 2>&1; then
  if ! timeout 20 ros2 control list_controllers | grep -q 'joint_trajectory_controller.*active'; then
    echo 'Banekontrolleren er ikke klar. Vent til Gazebo-terminalen viser TOP READY.' >&2
    exit 1
  fi
fi
resultat="base_minus3_ruter_$(date +%Y%m%d_%H%M%S).csv"
ros2 run lift_window_sim wash_demo --ros-args -p use_sim_time:=true \
  -p trajectory_csv:="$bane" -p output_csv:="$resultat"
ros2 run lift_window_sim analyze_log "$resultat" --output "${resultat%.csv}_metrics.json"
