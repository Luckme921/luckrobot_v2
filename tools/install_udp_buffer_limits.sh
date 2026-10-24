#!/usr/bin/env bash
# Requires explicit sudo; changes only UDP socket buffer ceilings, not networking.
set -euo pipefail
if [[ $EUID -ne 0 ]]; then
  echo 'Run: sudo bash ~/ros2_ws/tools/install_udp_buffer_limits.sh' >&2
  exit 1
fi
target=/etc/sysctl.d/90-luckrobot-udp.conf
if [[ -f "$target" ]]; then
  cp -a "$target" "$target.backup.$(date +%Y%m%d_%H%M%S)"
fi
printf '%s\n' '# Luckrobot Fast DDS image transport buffer ceilings' \
  'net.core.rmem_max = 4194304' 'net.core.wmem_max = 4194304' > "$target"
sysctl -p "$target"
