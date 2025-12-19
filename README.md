# Inicializando El Docker

```
docker build .
```
```
xhost +local:docker
docker run -it \
   --env="DISPLAY=$DISPLAY" \
   --env="QT_X11_NO_MITSHM=1" \
   --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" \
   --name m1_floam_container m1_floam
```
```
docker cp <Nombre_de_la_rosbag> m1_floam_container:/root/Downloads
```
```
nano /root/catkin_ms/src/floam/launch/floam_mapping_M1.launch
```

Aqui debemos reemplazar la bag en la siguiente linea por el nombre de nuestra bag

roslauch floam floam_mapping_M1.launch
rostopic echo -p --nostr --noarr /odom/pose > hh1_slam_pose.txt

docker cp m1_floam_container:/root/Downloads/hh1_slam_pose.txt . 
A ese archivo cambiale las ',' por ' '