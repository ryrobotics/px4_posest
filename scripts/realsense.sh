if [ $1 = 'host' ]
   git clone https://github.com/IntelRealSense/realsense-ros.git
   cd realsense-ros/
   git checkout `git tag | sort -V | grep -P "^2.\d+\.\d+" | tail -1`
elif [ $1 = 'docker' ]
then
   cd /src/catkin_ws/ \
   && catkin_make clean \
   && catkin_make -DCATKIN_ENABLE_TESTING=False -DCMAKE_BUILD_TYPE=Release \
   && catkin_make install \
   && source /src/catkin_ws/devel/setup.bash
fi