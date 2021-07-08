#!/bin/bash -l

: "${GITHUB_WORKSPACE?GITHUB_WORKSPACE has to be set. Did you use the actions/checkout action?}"
: "${GITHUB_ACTION_PATH?GITHUB_ACTION_PATH has to be set.}"
pushd "${GITHUB_WORKSPACE}" || exit

: "${ARTIFACTS_DIR?ARTIFACTS_DIR has to be set.}"
: "${BASE_URL?BASE_URL has to be set.}"
: "${TITLE?TITLE has to be set.}"
: "${ROS_DISTRO?ROS_DISTRO has to be set.}"
: "${HUGO_DIR?HUGO_DIR has to be set.}"
: "${OUTPUT_DIR?OUTPUT_DIR has to be set.}"
: "${LCOVRC_PATH?LCOVRC_PATH has to be set.}"
: "${CCN?CCN has to be set.}"
: "${NLOC?NLOC has to be set.}"
: "${ARGUMENTS?ARGUMENTS has to be set.}"
: "${EXCLUDE?EXCLUDE has to be set.}"

ARTIFACTS_DIR=$(echo "$ARTIFACTS_DIR" | sed -e "s@/\$@@")
HUGO_DIR=$(echo $HUGO_DIR | sed -e "s@/\$@@")
OUTPUT_DIR=$(echo $OUTPUT_DIR | sed -e "s@/\$@@")

CODECHECKER_CONFIG_PATH=${CODECHECKER_CONFIG_PATH:+$GITHUB_WORKSPACE"/"$CODECHECKER_CONFIG_PATH}
CODECHECKER_CONFIG_PATH=${CODECHECKER_CONFIG_PATH:-$GITHUB_ACTION_PATH"/codechecker-config.json"}
CODECHECKER_SKIP_LIST=${CODECHECKER_SKIP_LIST:+$GITHUB_WORKSPACE"/"$CODECHECKER_SKIP_LIST}
CODECHECKER_SKIP_LIST=${CODECHECKER_SKIP_LIST:-$GITHUB_ACTION_PATH"/codechecker-skip-list.txt"}

# Set timestamp
TIMESTAMP=$(date -u '+%Y%m%d_%H%M%S')
echo "$TIMESTAMP" > "$GITHUB_WORKSPACE"/timestamp.txt

# Run CodeChecker
. /opt/ros/"$ROS_DISTRO"/setup.sh
colcon build \
  --event-handlers console_cohesion+ \
  --cmake-args -DCMAKE_BUILD_TYPE=Release \
  -DCMAKE_EXPORT_COMPILE_COMMANDS=ON

git clone https://github.com/Ericsson/codechecker.git --depth 1 "$GITHUB_ACTION_PATH"/codechecker
pushd "$GITHUB_ACTION_PATH"/codechecker || exit
make venv || true
. "${PWD}/venv/bin/activate"
BUILD_UI_DIST=NO make package || true
popd || exit

codechecker/build/CodeChecker/bin/CodeChecker analyze "$GITHUB_WORKSPACE"/build/compile_commands.json \
  --config "$CODECHECKER_CONFIG_PATH" \
  --ignore "$CODECHECKER_SKIP_LIST" \
  --output ./reports || true

codechecker/build/CodeChecker/bin/CodeChecker parse -e html ./reports \
  -o "$GITHUB_WORKSPACE"/"$ARTIFACTS_DIR"/tidy-reports/"$TIMESTAMP" \
  --trim-path-prefix "$GITHUB_WORKSPACE" || true

# Measure code quality index
mkdir -p "$ARTIFACTS_DIR"
python "$GITHUB_ACTION_PATH"/scripts/ros_metrics_reporter.py \
  --base-dir="$GITHUB_WORKSPACE" \
  --action-dir="$GITHUB_ACTION_PATH" \
  --output-dir="$ARTIFACTS_DIR" \
  --timestamp="$TIMESTAMP" \
  --lcovrc="$LCOVRC_PATH" \
  --hugo-root-dir="$HUGO_DIR" \
  --base-url="$BASE_URL" \
  --title="$TITLE" \
  --exclude="$EXCLUDE" \
  --ccn="$CCN" \
  --nloc="$NLOC" \
  --arguments="$ARGUMENTS"

# Generate pages
cd "$HUGO_DIR" && hugo -d "$OUTPUT_DIR"
echo "::set-output name=output-dir::$OUTPUT_DIR"
