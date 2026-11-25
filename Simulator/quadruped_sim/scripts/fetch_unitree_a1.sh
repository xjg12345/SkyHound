#!/usr/bin/env bash
# Download only the A1 visual assets. No SDK, ROS nodes, or upstream build scripts.
set -euo pipefail
revision=157ff950351e5f535e03c5d85a16b7d594b3dbcc
base="https://raw.githubusercontent.com/unitreerobotics/unitree_ros/$revision"
dest="${1:-${XDG_CACHE_HOME:-$HOME/.cache}/prometheus/unitree-a1/$revision}"
meshes=(trunk.dae hip.dae thigh.dae thigh_mirror.dae calf.dae)
files=("${meshes[@]}" trunk_A1.png)
complete=1
for file in LICENSE "${files[@]}"; do
  [[ -s "$dest/$file" ]] || complete=0
done
if ((complete)) && [[ -f "$dest/REVISION" && "$(<"$dest/REVISION")" == "$revision" ]]; then
  realpath "$dest"
  exit
fi
mkdir -p "$dest"
stage="$(mktemp -d "$dest/.download.XXXXXX")"
trap 'rm -rf -- "$stage"' EXIT
paths=(LICENSE "${files[@]/#/robots/a1_description/meshes/}")
pids=()
for path in "${paths[@]}"; do
  curl --fail --silent --show-error --location --retry 2 --max-time 45 \
    "$base/$path" -o "$stage/${path##*/}" &
  pids+=("$!")
done
failed=0
for pid in "${pids[@]}"; do
  wait "$pid" || failed=1
done
((failed == 0)) || exit 1
for file in "${meshes[@]}"; do
  grep -qi '<COLLADA' "$stage/$file" || exit 1
done
mv -- "$stage/LICENSE" "${files[@]/#/$stage/}" "$dest/"
printf '%s\n' "$revision" > "$dest/REVISION"
realpath "$dest"
