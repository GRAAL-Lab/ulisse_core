#!/bin/bash

# Get where the script is located, so commands are working even if the script is run from somewhere
# else than the package folder.
DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"

gnome-terminal --tab -- bash -c "$DIR/run_bridge_in_gnome_tabs.sh"
