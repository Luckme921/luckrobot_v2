#!/usr/bin/env bash

# Delete only superseded LuckRobot deployment ZIPs from ~/ros2_ws.
# Diagnostics (*.tar.gz), source, build/install/log and user backups are untouched.
# Dry run: bash ~/ros2_ws/tools/cleanup_old_deployment_zips.sh
# Apply:   bash ~/ros2_ws/tools/cleanup_old_deployment_zips.sh --apply

set -euo pipefail

workspace="$(realpath "${HOME}/ros2_ws")"
if [[ "$workspace" != "${HOME}/ros2_ws" || ! -d "$workspace/src" ]]; then
  echo "拒绝清理：工作区校验失败：$workspace"
  exit 2
fi

apply=0
if [[ "${1:-}" == "--apply" ]]; then
  apply=1
elif [[ $# -gt 0 ]]; then
  echo "用法：$0 [--apply]"
  exit 2
fi

keep_zip()
{
  case "$1" in
    luckrobot_navigation_update_20260829_v2.zip|\
    luckrobot_stable_navigation_baseline_v1_20260831.zip|\
    luckrobot_observable_unknown_dfs_v10_6_20260907.zip|\
    luckrobot_rpp_heading_rollback_v10_6_1_20260907.zip|\
    luckrobot_custom_nav_plugins_v11_20260907.zip|\
    luckrobot_custom_nav_controller_fix_v11_1_20260907.zip)
      return 0 ;;
    *)
      return 1 ;;
  esac
}

count=0
bytes=0
while IFS= read -r -d '' file; do
  resolved="$(realpath "$file")"
  if [[ "$(dirname "$resolved")" != "$workspace" ]]; then
    echo "跳过工作区外文件：$resolved"
    continue
  fi
  name="$(basename "$resolved")"
  if keep_zip "$name"; then
    continue
  fi
  size="$(stat -c '%s' "$resolved")"
  count=$((count + 1))
  bytes=$((bytes + size))
  if [[ "$apply" -eq 1 ]]; then
    rm -- "$resolved"
    echo "已删除：$name"
  else
    echo "将删除：$name"
  fi
done < <(find "$workspace" -maxdepth 1 -type f -name 'luckrobot_*.zip' -print0)

human_size="$(numfmt --to=iec-i --suffix=B "$bytes")"
if [[ "$apply" -eq 1 ]]; then
  echo "完成：删除 $count 个旧部署 ZIP，释放约 $human_size。"
else
  echo "预览：共 $count 个旧部署 ZIP，约 $human_size；加 --apply 才会删除。"
fi
