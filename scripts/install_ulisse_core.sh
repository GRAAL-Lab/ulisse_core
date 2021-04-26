sudo apt install -y libeigen3-dev
sudo apt-get install -y libgeographic-dev
sudo apt install -y libconfig++-dev

cd ~
mdkir graal_ws
cd graal_ws

git clone git@bitbucket.org:isme_robotics/rml.git
cd rml
mkdir build
cd build
cmake ..
sudo make install
cd ../..

git clone git@bitbucket.org:isme_robotics/fsm.git
cd fsm
mkdir build
cd build
cmake ..
sudo make install
cd ../..

git clone git@bitbucket.org:isme_robotics/ctrl_toolbox.git
cd ctrl_toolbox
mkdir build
cd build
cmake ..
sudo make install
cd ../..

git clone git@bitbucket.org:isme_robotics/tpik.git
cd tpik
mkdir build
cd build
cmake ..
sudo make install
cd ../..

git clone git@bitbucket.org:isme_robotics/ikcl.git
cd ikcl
mkdir build
cd build
cmake ..
sudo make install
cd ../..

git clone https://github.com/SINTEF-Geometry/SISL.git
cd SISL
mkdir build
cd build
cmake ..
sudo make install
cd ../..

sudo apt install -y libgps-dev gpsd-clients
sudo apt install -y qtquickcontrols2-5-dev qtlocation5-dev qtpositioning5-dev qml-module-qtquick-controls2 qml-module-qt-labs-settings qml-module-qt-labs-folderlistmodel qml-module-qtlocation qml-module-qtpositioning qml-module-qtquick-extras qml-module-qtgraphicaleffects qml-module-qtquick-dialogs qml-module-qtquick-controls python3-colcon-common-extensions qml-module-qtqml-models2

cd ~
mkdir -p ros2_ws/src
cd ros2_ws/src
git clone git@bitbucket.org:isme_robotics/ulisse_core.git
cd ..

source /opt/ros/foxy/setup.bash
source ~/ros2_ws/install/setup.bash

colcon build --symlink-install --packages-select ulisse_msgs

source ~/ros2_ws/install/setup.bash

colcon build --symlink-install

