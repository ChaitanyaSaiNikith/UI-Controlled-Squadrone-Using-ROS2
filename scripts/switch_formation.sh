#!/usr/bin/env bash
# switch_formation.sh — Publish a formation type change.
# Usage: ./scripts/switch_formation.sh <formation>
# Valid formations: V, line, diamond, square

set -eo pipefail

VALID="V line diamond square"

if [[ -z "$1" ]]; then
    echo "Usage: $0 <formation>"
    echo "Valid formations: $VALID"
    exit 1
fi

if ! echo "$VALID" | grep -qw "$1"; then
    echo "Unknown formation '$1'. Valid: $VALID"
    exit 1
fi

echo "Switching formation to: $1"
ros2 topic pub --once /formation_type std_msgs/msg/String "{data: '$1'}"
echo "Done."
