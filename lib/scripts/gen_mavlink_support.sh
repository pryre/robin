#!/bin/sh

DIR=$1
MAVLINK_WEB='https://mavlink.io/en/messages/common.html#'

echo '# Supported MAVLINK Messages'
echo "[Back to index](README.md)."
echo

print_link() {
    echo "- [$@](${MAVLINK_WEB}$@)"
}

grep -R -o --no-filename -e 'MAVLINK_MSG_ID_[A-Za-z_]*' -e 'mavlink_msg_.*pack' ./src/ | \
	sed -e 's/_pack//' -e 's/mavlink_msg_//' -e 's/MAVLINK_MSG_ID_//' | \
	tr '[:lower:]' '[:upper:]' | \
	sort | \
	uniq | \
	while read in; do print_link "$in"; done
