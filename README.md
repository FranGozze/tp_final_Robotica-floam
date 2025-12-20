# Inicializando El Docker

```
docker build .
```
```
xhost +local:docker
```
```
docker run -it \
   --env="DISPLAY=$DISPLAY" \
   --env="QT_X11_NO_MITSHM=1" \
   --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" \
   --name m1_floam_container m1_floam
```
```
docker cp <Archivo_de_la_rosbag> m1_floam_container:/root/Downloads
```

En el caso de necesitar volver a abrir el docker una vez cerrado se puede utilizar el siguiente comando:

```
docker start m1_floam_container
```

Por ultimo, para poder conectarse al container basta con utilizar el siguiente comando.
```
docker exec -it m1_floam_container bash
```

# Ejecutando la rosbag
Primero deberemos modificar el path de la bag a ejecutar. Para primero deberemos modificar el archivo de lanzamiento:
```
nano root/catkin_ws/src/M1_floam/launch/floam_mapping_M1.launch
```

Aqui debemos reemplazar la bag en la siguiente linea por el nombre de nuestra bag 

```
<node pkg="rosbag" type="play" name="rosbag_play" args="&#45;&#45;clock -r 0.5 $(env HOME)/Downloads/<Archivo_de_la_rosbag>"/>
```
Luego, para ejecutar el metodo deberemos hacer:
```
roslauch floam floam_mapping_M1.launch
```

Por otra parte, si queremos grabar las distintas pose's calculadas por el metodo, basta con abrir en otra consola, conectarse nuevamente al docker, y luego de iniciar el roslaunch ejecutar a la vez:
```
rostopic echo -p --nostr --noarr /odom/pose > /root/Downloads/<Nombre_rosbag>_slam_pose.txt
```

Por ultimo, para poder tener dichos resultados en nuestro sistema, fuera del docker ejecutamos el comando:
```
docker cp m1_floam_container:/root/Downloads/<Nombre_rosbag>_slam_pose.txt Files/F-LOAM
```

# Procesando datos
## Requisitos:
-  Instalar los paquetes que se encuentran en requirements.txt


## Ejecucion:
Ejecutar el archivo 'proccess_bus_path.sh', el cual se encargara:
- Transformar los ground_truth que se encuentran en UTM al formato tum
- Formatear y alinear los archivos obtenidos por por F-LOAM
- Generar las imagenes que comparan las trayectorias
- Muestra por consola los datos de error calculados por la herramienta 'evo'

Se debe tener en cuenta de este procesos es que el ground_truth de los rosbags hh1, hh2 y hh3 que se encuentran en el dataset HK_MEMS estan dados en formato tum, por lo que no es necesario realmente los ejecutar el primero paso del script en dichas bags.

Por otra parte, cabe destacar que en caso de querer usar otros datasets, se debera modificar la variable DATASET del script, ya que este contiene los distintos datasets que su ground_truth este en formato UTM.