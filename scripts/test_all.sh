#!/usr/bin/env bash
set -euo pipefail
project_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
cd -- "$project_dir"
export ONLINE_TEST_RUN="${ONLINE_TEST_RUN:-$project_dir/examples/reference_online}"
export HANDOVER_TEST_RUN="${HANDOVER_TEST_RUN:-$project_dir/examples/reference_handover}"
exec "${PYTHON:-python3}" -m unittest -v test_online.py test_packet_demo.py test_handover.py test_portability.py
