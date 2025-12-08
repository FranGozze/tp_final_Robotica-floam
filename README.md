docker cp 2011_09_30_0027.zip priceless_khayyam:/

xhost +local:docker
docker run -it \
   --env="DISPLAY=$DISPLAY" \
   --env="QT_X11_NO_MITSHM=1" \
   --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" \
   floam

mv 2011_09_30_0027.zip ~/Downloads
cd ~/Downloads
unzip ~/Downloads/