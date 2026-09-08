#!/usr/bin/env bash
# Convenience commands for the Mac. Robotics logic lives in move_to_pose.py.
set -euo pipefail
cd "$(dirname "$0")"
command=${1:-help}
if [[ $# -gt 0 ]]; then shift; fi

if [[ "$command" == help || "$command" == --help ]]; then
  cat <<'HELP'
./robot start                   Build and start the UR5e + MoveIt + RViz
./robot pose                    Print the current tool pose
./robot move --position X Y Z --rpy-deg R P Y
                               Move to a pose (metres and degrees)
./robot move ... --plan-only    Check planning without executing
./robot test                    Run unit and simulation integration tests
./robot logs                    Follow startup and robot logs
./robot shell                   Open a ROS-ready Linux shell
./robot stop                    Stop this project's container

RViz: http://localhost:6080/vnc.html?autoconnect=true&resize=scale
HELP
  exit 0
fi

docker_cmd=(docker)
if ! docker info >/dev/null 2>&1; then
  if docker --context colima info >/dev/null 2>&1; then
    docker_cmd=(docker --context colima)
  else
    echo 'Docker engine is unavailable. Start it with: colima start --cpu 4 --memory 6 --vm-type vz' >&2
    exit 1
  fi
fi
# Use a local CLI config for these public images. The Mac's existing config may
# refer to Docker Desktop's credential helper, which requires its backend.
endpoint=${DOCKER_HOST:-$("${docker_cmd[@]}" context inspect --format '{{.Endpoints.docker.Host}}')}
mkdir -p .docker-cli
python3 - <<'PY'
import json
from pathlib import Path
plugins = [str(Path.home() / '.docker/cli-plugins'), '/opt/homebrew/lib/docker/cli-plugins']
Path('.docker-cli/config.json').write_text(json.dumps({'cliPluginsExtraDirs': plugins}))
PY
unset DOCKER_CONTEXT DOCKER_HOST
docker_cmd=(docker --config "$PWD/.docker-cli" --host "$endpoint")
compose=("${docker_cmd[@]}" compose)
exec_flags=(-T)
if [[ -t 0 && -t 1 ]]; then exec_flags=(--interactive=true); fi  # Forward Ctrl-C.
case "$command" in
  start)
    "${compose[@]}" up --build -d
    echo 'Starting ROS 2. Use ./robot logs to see progress.'
    echo 'Open http://localhost:6080/vnc.html?autoconnect=true&resize=scale'
    ;;
  pose) "${compose[@]}" exec "${exec_flags[@]}" robot bash /opt/demo/ros-env.sh ros2 run ur5e_pose_control move_to_pose --current "$@" ;;
  move) "${compose[@]}" exec "${exec_flags[@]}" robot bash /opt/demo/ros-env.sh ros2 run ur5e_pose_control move_to_pose "$@" ;;
  test) "${compose[@]}" exec -T robot bash /opt/demo/ros-env.sh python3 /workspace/tests/integration.py ;;
  logs) "${compose[@]}" logs -f --tail=100 robot ;;
  shell) "${compose[@]}" exec robot bash /opt/demo/ros-env.sh bash ;;
  stop) "${compose[@]}" down ;;
  *) echo "Unknown command: $command. Run ./robot help." >&2; exit 2 ;;
esac
