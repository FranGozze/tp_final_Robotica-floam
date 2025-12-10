docker cp 2011_09_30_0027.zip m1_floam_container:/root/Downloads
docker cp WT3_western_tunnel_north_3_no_camera_2024-06-13-15-23-55_compressed.bag m1_floam_container:/root/Downloads/

xhost +local:docker
docker run -it \
   --env="DISPLAY=$DISPLAY" \
   --env="QT_X11_NO_MITSHM=1" \
   --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" \
   --name m1_floam_container m1_floam

mv WT3_western_tunnel_north_3_no_camera_2024-06-13-15-23-55_compressed.bag ~/Downloads
cd ~/Downloads
unzip ~/Downloads/