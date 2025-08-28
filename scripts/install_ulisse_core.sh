#!/bin/bash
GRN='\033[1;32m'
RED='\033[1;31m'
NC='\033[0m' # No Color

cd ~

if [ -d "graal_ws/graal_utils" ]; then
	cd graal_ws/graal_utils/scripts
	echo -e "${GRN}Found graal_ws/graal_utils${NC}. Installing deps."
	./install_update_graal_libs.sh
	
else
	echo -e "${RED}Error: no graal_ws/graal_utils folder found, please clone \"graal_utils\" first and put it inside a graal_ws folder.${NC}"
	exit
fi


# Create "ulisse_ws" folder if not existing
cd ~
if ! [ -d "ulisse_ws/src" ]; then
    mkdir -p "ulisse_ws/src"
else
	echo -e "${GRN}Found ulisse_ws/src"
fi

cd ulisse_ws/src

if ! [ -d "ulisse_core" ]; then
    echo -e "Cloning ulisse_core repo..."
    git clone git@bitbucket.org:isme_robotics/ulisse_core.git
else
	echo -e "${GRN}Found ulisse_core repo${NC}"
fi

if ! [ -d "marine_vehicle_models" ]; then
    echo -e "Cloning marine_vehicle_models repo..."
    git clone git@bitbucket.org:isme_robotics/marine_vehicle_models.git
else
	echo -e "${GRN}Found marine_vehicle_models repo${NC}"
fi

cd ~/ulisse_ws
source /opt/ros/kilted/setup.bash
colcon build --symlink-install --executor sequential

