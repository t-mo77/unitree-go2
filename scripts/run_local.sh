#!/usr/bin/env bash
# 既存のROS 2と、このリポジトリの仮想環境で起動する。
set -eo pipefail
repo_dir=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)
if [[ ! -x "$repo_dir/.venv/bin/python" ]]; then
  echo '専用の .venv がありません。READMEのLinux直接実行の手順を確認してください。' >&2
  exit 1
fi
source /opt/ros/humble/setup.bash
source "$HOME/unitree_ros2/cyclonedds_ws/install/setup.bash"
export PYTHONPATH="$repo_dir:$repo_dir/nl_control${PYTHONPATH:+:$PYTHONPATH}"
cd -- "$repo_dir"
exec "$repo_dir/.venv/bin/python" -m voice_ltl.local "$@"
