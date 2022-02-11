sudo apt-get update

# install pip
sudo apt-get -y install python3-pip

#install jetson-stats (jtop command)
sudo pip install -U jetson-stats

#install Intel Realsense
sudo apt-key adv --keyserver keys.gnupg.net --recv-key F6E65AC044F831AC80A06380C8B3A55A6F3EFCDE || sudo apt-key adv --keyserver hkp://keyserver.ubuntu.com:80 --recv-key F6E65AC044F831AC80A06380C8B3A55A6F3EFCDE
sudo add-apt-repository "deb https://librealsense.intel.com/Debian/apt-repo bionic main" -u
sudo apt-get install -y librealsense2-utils
sudo apt-get install -y librealsense2-dev

#OpenCV install
sudo apt-get update
sudo apt-get upgrade
sudo apt-get install -y nano
sudo apt-get install -y dphys-swapfile

# enlarge SWAP
# https://qengineering.eu/install-opencv-4.5-on-jetson-nano.html
sudo nano /sbin/dphys-swapfile
sudo nano /etc/dphys-swapfile
sudo reboot now

# need 8.5Gb at least
free -m
#install
cd ~
wget https://github.com/Qengineering/Install-OpenCV-Jetson-Nano/raw/main/OpenCV-4-5-5.sh
sudo chmod 755 ./OpenCV-4-5-5.sh
./OpenCV-4-5-5.sh
rm OpenCV-4-5-5.sh
sudo /etc/init.d/dphys-swapfile stop
sudo apt-get remove --purge dphys-swapfile
sudo rm -rf ~/opencv
sudo rm -rf ~/opencv_contrib

#install H5DF
sudo apt install hdf5-tools

sudo ldconfig

#nvcc
sudo nano ~/.bashsrc

#add to the end of file
export PATH=/usr/local/cuda/bin${PATH:+:${PATH}}
export LD_LIBRARY_PATH=/usr/local/cuda/lib64\
                         ${LD_LIBRARY_PATH:+:${LD_LIBRARY_PATH}}

sudo adduser $USER dialout