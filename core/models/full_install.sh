#!/bin/bash
# CC BY-NC
# Alex Lucas & Tom Howard, University of Sheffield
# Copyright (c) 2022

# This script instals the ROS Noetic environment with extra packages
# to support developing code for the MiRo and Turtlebot 3 robots.
# The script sets up the ROS network for work in simulation (localhost).

RED='\033[0;31m'
YELLOW='\033[1;33m'
GREEN='\033[0;32m'
NC='\033[0m'

ask() {
    local reply prompt
    prompt='y/n'
    echo -e -n "$1 ${YELLOW} [$prompt] ${NC}>> "
    read -r reply </dev/tty
    if [[ -z $reply ]]; then
        return 1;
    elif [ "$reply" == "y" ] || [ "$reply" == "Y" ]; then
        return 0;
    else
        return 1;
    fi
}

echo -e "${YELLOW}[Note] Target OS version  >>> Ubuntu 20.04.x (Focal Fossa) or Linux Mint 21.x${NC}"
echo -e "${YELLOW}[Note] Target ROS version >>> ROS Noetic Ninjemys${NC}"

#################  Part I. ROS, TB3, catkin_ws and dependencies ################
echo -e "\n${YELLOW}[Set the target OS, ROS version and the name of catkin workspace]${NC}"
name_os_version=${name_os_version:="focal"}
name_ros_version=${name_ros_version:="noetic"}
name_catkin_workspace=${name_catkin_workspace:="catkin_ws"}

echo -e "\n${YELLOW}[Update & Upgrade]${NC}"
sudo apt update -y
sudo apt upgrade -y

echo -e "\n${YELLOW}[NTP: update time]${NC}"
sudo apt install -y chrony ntpdate curl build-essential git
sudo ntpdate ntp.ubuntu.com
sleep 2

echo -e "\n${YELLOW}[Add the ROS repository]${NC}"
if [ ! -e /etc/apt/sources.list.d/ros-latest.list ]; then
  sudo sh -c "echo \"deb http://packages.ros.org/ros/ubuntu ${name_os_version} main\" > /etc/apt/sources.list.d/ros-latest.list"
fi

echo -e "\n${YELLOW}[Download the ROS keys]${NC}"
roskey=`apt-key list | grep "Open Robotics"`
if [ -z "$roskey" ]; then
  curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -
fi

echo -e "\n${YELLOW}[Check the ROS keys]${NC}"
roskey=`apt-key list | grep "Open Robotics"`
if [ -n "$roskey" ]; then
  echo -e "\n${YELLOW}[ROS key exists in the list]${NC}"
else
  echo -e "\n${RED}[Failed to receive the ROS key, aborts the installation]${NC}"
  exit 0
fi

echo -e "\n${YELLOW}[Update & Upgrade]${NC}"
sudo apt update -y
sudo apt upgrade -y

echo -e "\n${YELLOW}[Source .bashrc]${NC}"
source $HOME/.bashrc

echo -e "\n${YELLOW}[Install all the necessary ROS and TB3 packages]${NC}"
sudo apt install -y ros-noetic-desktop-full ros-noetic-rqt-* ros-noetic-gazebo-* ros-noetic-joy ros-noetic-teleop-twist-joy ros-noetic-teleop-twist-keyboard ros-noetic-laser-proc ros-noetic-rgbd-launch ros-noetic-rosserial-arduino ros-noetic-amcl ros-noetic-map-server ros-noetic-move-base ros-noetic-rqt* ros-noetic-gmapping ros-noetic-navigation python3-rosdep python3-rosinstall python3-rosinstall-generator python3-wstool build-essential ros-noetic-dynamixel-sdk ros-noetic-turtlebot3 ros-noetic-turtlebot3-simulations python3-pip python3-catkin-tools ffmpeg

echo -e "\n${YELLOW}[Initialise rosdep and update]${NC}"
sudo rm /etc/ros/rosdep/sources.list.d/20-default.list
sudo sh -c "rosdep init"
rosdep update

echo -e "\n${YELLOW}[Environment setup]${NC}"
source /opt/ros/$name_ros_version/setup.sh
source $HOME/.bashrc

echo -e "\n${YELLOW}[Create and build the TB3 catkin workspace]${NC}"
mkdir -p $HOME/$name_catkin_workspace/src
cd $HOME/$name_catkin_workspace/src
catkin_init_workspace
cd $HOME/$name_catkin_workspace
catkin build

source $HOME/.bashrc

################################# Part II. MDK #################################
if ask "[Do you want to set up MiRo Developer Kit?]"; then
  echo -e "\n${YELLOW}[Install AprilTags with Pip3]${NC}"
  sudo pip3 install apriltag

  echo -e "\n${YELLOW}[Downloading and unpacking MDK into ~/pkgs]${NC}"
  mkdir -p ~/pkgs
  cd ~/pkgs/
  wget --no-check-certificate 'https://docs.google.com/uc?export=download&id=1vNODaenljocVWalM4cOW4Kax-RB4U3nh' -O mdk_2-230105.tgz
  tar -xvzf mdk_2-230105.tgz

  echo -e "\n${YELLOW}[Remove MDK soft link]${NC}"
  rm ~/mdk
  echo -e "\n${YELLOW}[Remove MDK .bashrc entries]${NC}"
  sed -i '/# MDK/d' ~/.bashrc
  sed -i '/source ~\/mdk\/setup.bash/d' ~/.bashrc
  unset MIRO_DIR_MDK
  unset MIRO_SYSTEM

  echo -e "\n${YELLOW}[Install MDK]${NC}"
  source ~/.bashrc
  cd ~/pkgs/mdk-230105/bin/deb64
  ./install_mdk.sh

  echo -e "\n${YELLOW}[Add the 'launch_full' script]${NC}"
  wget -O ~/mdk/sim/launch_full.sh https://gist.githubusercontent.com/AlexandrLucas/703831843f9b46edc2e2032bcd08651f/raw/launch_full.sh
  chmod +x ~/mdk/sim/launch_full.sh

  echo -e "\n${YELLOW}[Cleaning and building MDK workspace]${NC}"
  source ~/.bashrc
  cd ~/mdk/catkin_ws/
  catkin clean
  catkin build
  cd ~/mdk/catkin_ws/build/miro2_msg/
  make install

  echo -e "\n${YELLOW}[Removing MDK archive]${NC}"
  rm ~/pkgs/mdk_2-230105.tgz

else
  echo -e "\n${YELLOW}[Skipping MDK...]${NC}"
fi

####################### Part III. TUoS Robotics scripts ########################
if ask "[Do you want to also set up TUoS Robot Switch scripts?]"; then

  # Remove the new MDK ~/.bashrc entries (will be done separately in a different way)
  sed -i '/# MDK/d' ~/.bashrc
  sed -i '/source ~\/mdk\/setup.bash/d' ~/.bashrc

  echo -e "\n${YELLOW}[Setting up scripts in /usr/local/bin/]${NC}"
  cd /usr/local/bin/
  sudo wget -O /usr/local/bin/robot_switch https://raw.githubusercontent.com/tom-howard/tuos_robotics/main/students/robot_switch
  sudo wget -O /usr/local/bin/robot_mode https://raw.githubusercontent.com/tom-howard/tuos_robotics/main/students/robot_mode
  sudo chmod +x *

  echo -e "\n${YELLOW}[Setting up scripts in  ~/.tuos]${NC}"
  mkdir -p ~/.tuos
  cd ~/.tuos
  wget -O ~/.tuos/bashrc_miro https://raw.githubusercontent.com/tom-howard/tuos_robotics/main/students/bashrc_miro
  wget -O ~/.tuos/bashrc_turtlebot3 https://raw.githubusercontent.com/tom-howard/tuos_robotics/main/students/bashrc_turtlebot3
  wget -O ~/.tuos/bashrc_robot_switch https://raw.githubusercontent.com/tom-howard/tuos_robotics/main/students/bashrc_robot_switch

  echo -e "\n${YELLOW}[Adding the Robotics Kit switch to ~/.bashrc]${NC}"
  wget -O /tmp/.bashrc_extras https://raw.githubusercontent.com/tom-howard/tuos_robotics/main/students/.bashrc_extras

  tmp_file=/tmp/.bashrc_extras
  while IFS= read -r line; do
    grep -qxF "$line" ~/.bashrc || echo "$line" >> ~/.bashrc
  done < "$tmp_file"

  echo -e "\n${YELLOW}[Adding bash aliases]${NC}"
  cd ~
  wget -O /tmp/.bash_aliases https://raw.githubusercontent.com/tom-howard/tuos_robotics/main/students/.bash_aliases
  touch ~/.bash_aliases
  tmp_file=/tmp/.bash_aliases
  while IFS= read -r line; do
    grep -qxF "$line" ~/.bash_aliases || echo "$line" >> ~/.bash_aliases
  done < "$tmp_file"

else
  echo -e "\n${YELLOW}[Skipping TUoS Robotics scripts...]${NC}"
fi

############################ Part IV. VcXsrv for WSL ###########################
echo -e "\n${YELLOW}[NOTE: The following is only required if you are installing on WSL]${NC}"
if ask "[Do you want to set up VcXsrv in .bashrc for running graphical applications in WSL?]"; then
  echo -e "\n${YELLOW}[Setting up VcXsrv in .bashrc]${NC}"
  wget -O /tmp/wsl_bashrc https://raw.githubusercontent.com/tom-howard/tuos_robotics/main/students/wsl_bashrc
  tmp_file=/tmp/wsl_bashrc
  while IFS= read -r line; do
    grep -qxF "$line" ~/.bashrc || echo "$line" >> ~/.bashrc
  done < "$tmp_file"

else
  echo -e "\n${YELLOW}[Skipping VcXsrv settings...]${NC}"
fi

########################## Part V. Teaching materials ##########################
if ask "[Do you want to download COM2009 and COM3528 teaching materials?]"; then
  cd $HOME/$name_catkin_workspace/src
  git clone https://github.com/tom-howard/COM2009
  catkin build

  cd ~/mdk/catkin_ws/src
  git clone https://github.com/AlexandrLucas/COM3528
  catkin build
fi

############################## Part VI. Clean up ###############################
echo -e "\n${YELLOW}[Clean-up]${NC}"
sudo apt update -y
sudo apt upgrade -y
sudo apt autoremove -y
sudo apt autoclean -y

echo -e "\n${GREEN}[COMPLETE]: Don't forget to reboot ASAP.${NC}"
exit 0
