#!/usr/bin/env bash
# Ubuntu 22.04 / Python 3.10。既存のROS 2を使い、アプリ依存だけ専用環境へ入れる。
set -euo pipefail
repo_dir=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)
cd -- "$repo_dir"
python3 -m venv --system-site-packages --without-pip .venv
venv_site=$("$repo_dir/.venv/bin/python" -c 'import sysconfig; print(sysconfig.get_path("purelib"))')
if [[ ! -f "$venv_site/pip/__init__.py" ]]; then
  python3 -m pip install --target "$venv_site" pip
fi
"$repo_dir/.venv/bin/python" -m pip install -r requirements-local.txt
echo '完了。bash scripts/run_local.sh fake で偽Go2を起動できます。'
