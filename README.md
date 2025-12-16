docker build ...

xhost +local:docker
docker run -it \
   --env="DISPLAY=$DISPLAY" \
   --env="QT_X11_NO_MITSHM=1" \
   --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" \
   --name m1_floam_container m1_floam

mv WT3_western_tunnel_north_3_no_camera_2024-06-13-15-23-55_compressed.bag /root/Downloads
cd /root/Downloads
unzip /root/Downloads/

docker cp 2011_09_30_0027.zip m1_floam_container:/root/Downloads
docker cp WT3_western_tunnel_north_3_no_camera_2024-06-13-15-23-55_compressed.bag m1_floam_container:/root/Downloads/

roslauch floam floam_mapping_M1.launch
rostopic echo -p --nostr --noarr /odom/pose > hh1_slam_pose.txt

docker cp m1_floam_container:/root/Downloads/hh1_slam_pose.txt . 
A ese archivo cambiale las ',' por ' '