# Kogrob_HF
Baróthy_Berencsi_Dankó_Tancsa

## Feladat leírása

Feladatunk vonalkövetés megvalósítása turtlebottal neurális 
háló segítségével.
A neurális hálót feltanítjuk a vonalkövetésre illetve, hogy 
milyen színű vonalon milyen sebességgel közlekedjen. A 
robot által bejárt út 2D-s felülnézeti reprezentációját 
egy külön python node segítségével ábrázoljuk.

## Telepítés és indítás

Első lépésként a ROS telepítése után be kell tölteni a terminálba a ROS környezetét:

```bash
source /opt/ros/noetic/setup.bash
```

Ha rendelkezünk catkin workspace-el akkor navigáljunk bele.
Ha viszont még nem rendelkezünk catkin workspace-el akkor a következő parancsok lefuttatásával hozzuk létre és fordítsunk le egy catkin_ws mappát.

```bash
cd ~
mkdir -p catkin_ws/src
cd catkin_ws
catkin_make
```

Ezután a catkin workspace-ünket is tudjuk source-olni:

```bash
source ~/catkin_ws/devel/setup.bash
```

Figyeljünk arra, hogy azt a workspace-t source-oljuk, amibe dolgozni szeretnénk és lépjünk az src mappába.

A turtlebot3 szimulációhoz használt alapcsomagjainak GIT repoját töltsük be a workspace-ünkbe:

```bash
git clone https://github.com/ROBOTIS-GIT/turtlebot3_simulations
git clone https://github.com/ROBOTIS-GIT/turtlebot3_msgs
git clone https://github.com/MOGI-ROS/turtlebot3
```

A MOGI-ROS/turtlebot3 repojából a mogi-ros branch-et fogjuk használni, ahol a robotra a kamera már elhelyezésre és konfigurálásra került.
A branchek közötti váltás a következőképpen kell végrehajtani:

```bash
git checkout mogi-ros
git pull origin mogi-ros
```

Ezt követően le kell tölteni ezt a repo-t a következő paranccsal:

```bash
git clone https://github.com/mediforr12/Kogrob_HF
```

A szimulációhoz még be kell állítani néhány környezeti változót a robot fajtájára és az elkészített modellek elérhetőségére vonatkozóan.

```bash
export TURTLEBOT3_MODEL=burger
export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:~/catkin_ws/src/Kogrob_HF/turtlebot3_hf/gazebo_models/
```

Ezzel készen is áll a szimulációnk a futtatásra. A roslaunch parancs segítségével el is indíthatjuk a szimulációt:

```bash
roslaunch turtlebot3_hf simulation_line_follow.launch
```

Egy másik terminálablakban szintén source-oljuk a ROS környezetet és az általunk használt catkin workspace-t

```bash
source /opt/ros/noetic/setup.bash
source ~/catkin_ws/devel/setup.bash
```

Amennyiben még nem rendelkezünk python package installer-el akkor telepítsük:

```bash
sudo apt install python3-pip
```

Ha ezzel megvagyunk telepítsük fel a python script lefutásához szükséges package-ket.

```bash
python3 -m pip install tensorflow==2.9.2
```

Ebben a terminálablakban le is futtathatjuk a vonalkövetéshez készített scriptet:

```bash
rosrun turtlebot3_hf line_follower_cnn.py
```
