# Kogrob_HF
Baróthy_Berencsi_Dankó_Tancsa

## Tartalomjegyzék
1. [Feladat leírása](#leiras)  
2. [Feladat megvalósítása](#megval)  
3. [Telepítés és indítás](#run) 

## Feladat leírása <a name="leiras"></a>

Feladatunk vonalkövetés megvalósítása turtlebottal neurális 
háló segítségével.
A neurális hálót feltanítjuk a vonalkövetésre illetve, hogy 
milyen színű vonalon milyen sebességgel közlekedjen. A 
robot által bejárt út 2D-s felülnézeti reprezentációját 
egy külön python node segítségével ábrázoljuk. A modellt szerettük volna feltanítani arra az eshetőségre is, hogy egy táblát meglátva 

## Feladat megvalósítása <a name="megval"></a>

Az alkalmazott függvények, logikák és scriptek nagymértékben támaszkodnak az [óra segédletre](https://github.com/MOGI-ROS/Week-1-8-Cognitive-robotics).

A feladatunkhoz az F1-es Monza nevű pályát vettük alapul, kiegészítve egy boxutca szerűséggel, ahova a robotot egy tábla segítségével lehet kiterelni.
A pályát Blender-ben hoztuk létre egy bezier görbe segítségével. Először kialakítottuk a pálya vonalvezetését, majd ráillesztettük az úttestet jelképező modellelemet. Ezt követően elkészítettük a tábla modellt, amit a kitereléshez fogunk használni.

![image](https://github.com/mediforr12/Kogrob_HF/assets/62999984/9a979288-f47c-405b-b9cb-a3780981408c)
![image](https://github.com/mediforr12/Kogrob_HF/assets/62999984/556999b0-12ed-4e73-a83c-e56860781f1c)
![image](https://github.com/mediforr12/Kogrob_HF/assets/62999984/7154076a-a5af-4c09-be0d-35e15cb96a08)

A végeredményt collada(`.dae`) fájlformátumba mentettük, hogy a Gazebo kezelni tudja. Gazebon belül összeraktuk a teljes modellt és létrehoztuk a `.world` fájlt.

![image](https://github.com/mediforr12/Kogrob_HF/assets/62999984/0db1d453-2905-4aa0-8682-035220907b58)

A `train_network.py` tartalmazza a felépített neurális hálót, ami a robotot végigvezeti a vonalon. A háló struktúrája a kiadott segédlethez hasonlóan [LeNet-5](https://en.wikipedia.org/wiki/LeNet) jellegű. A segédlethez képest a kimenet aktivációs függvényét változtattuk meg sigmoid-ra, mert néhány helyen említik, hogy a multi-label klasszifikációs feladatoknál ez a használatos. Az elért legjobb modellsúlyok kimentéséért a `ModelCheckpoint()` callback felel.
```console
_________________________________________________________________
 Layer (type)                Output Shape              Param #   
=================================================================
 conv2d (Conv2D)             (None, 24, 24, 20)        1520      
                                                                 
 activation (Activation)     (None, 24, 24, 20)        0         
                                                                 
 max_pooling2d (MaxPooling2D  (None, 12, 12, 20)       0         
 )                                                               
                                                                 
 conv2d_1 (Conv2D)           (None, 12, 12, 50)        25050     
                                                                 
 activation_1 (Activation)   (None, 12, 12, 50)        0         
                                                                 
 max_pooling2d_1 (MaxPooling  (None, 6, 6, 50)         0         
 2D)                                                             
                                                                 
 flatten (Flatten)           (None, 1800)              0         
                                                                 
 dense (Dense)               (None, 500)               900500    
                                                                 
 activation_2 (Activation)   (None, 500)               0         
                                                                 
 dense_1 (Dense)             (None, 10)                5010      
                                                                 
 activation_3 (Activation)   (None, 10)                0         
                                                                 
=================================================================
Total params: 932,080
Trainable params: 932,080
Non-trainable params: 0
_________________________________________________________________
```
A tanításhoz használt képeket a kapott `save_training_images.py` node-al készítettük. Annak érdekében, hogy minden színt minden pozícióban helyesen felismerjen és, hogy a robot mindig pontosan navigáljon, formális és buzgó kollégám kb 1100 képet kategorizált 10 különböző mappába manuálisan. (jó nagy b@rom ..lol)

A háló feltanításának eredménye a következő képen látható:
![image](https://github.com/mediforr12/Kogrob_HF/assets/62999984/041a9c07-a273-44a3-b74b-147cf6959094)

A `line_follower_cnn.py` ros node-ot a segédlethez képest több helyen módosítottuk. A robot x-y síkban elfoglalt helyét kiolvassuk az odometryából a `Position` callback segítségével, majd az eredményeket átadtuk a `cvThread` class `drawTrajectory` függvényének. 

A robot mozgásának és a vonal szinének meghatározásához két `argmax()` függvényt használtunk egyet az irányra, egyet pedig a színre és a robot mozgásparamétereit ennek megfelően módosítottuk

``` python
if prediction_color == 0: # Red
            self.color = [1,0,0]
            color_linfaktor = 0.5
            color_angfaktor = 1.5
        elif prediction_color == 1: # Green
            self.color = [0,1,0]
            color_linfaktor = 1.2
            color_angfaktor = 1
        elif prediction_color == 2: # Blue
            self.color = [0,0,1]
            color_linfaktor = 1
            color_angfaktor = 1
        elif prediction_color == 3: # Magenta
            self.color = [1,0,1]
            color_linfaktor = 2
            color_angfaktor = 0.5
        elif prediction_color == 4: # Yellow
            self.color = [1,1,0]
            color_linfaktor = 0.8
            color_angfaktor = 1.2
        elif prediction_color == 5: # Cian
            self.color = [0,1,1]
            color_linfaktor = 0.7
            color_angfaktor = 0.8
        if prediction_dir == 0: # Forward
            self.cmd_vel.angular.z = 0*color_angfaktor
            self.cmd_vel.linear.x = 0.1*color_linfaktor
        elif prediction_dir == 1: # Right
            self.cmd_vel.angular.z = -0.2*color_angfaktor
            self.cmd_vel.linear.x = 0.05*color_linfaktor
        elif prediction_dir == 2: # Left
            self.cmd_vel.angular.z = 0.2*color_angfaktor
            self.cmd_vel.linear.x = 0.05*color_linfaktor
        elif prediction_dir == 3: # Stop
            self.cmd_vel.angular.z = 0.1*color_angfaktor
            self.cmd_vel.linear.x = 0.0*color_linfaktor
```

A kamera által látott kép mutatására a segédletben található programrészletet használtuk fel.

A modellkörnyezet felépítéséhez a `/turtlebot3_hf/launch/simulation_line_follow.launch`  lauch fájlt használjuk, amit a segédlethez képest annyival módosítottunk, hogy az Rviz elindítására vonatkkozó részt eltávolítottuk, mert arra nem volt szükségünk. 
Végeredmény a `line_follower_cnn` node elindítása után:

![image](https://github.com/mediforr12/Kogrob_HF/assets/62999984/351ad151-44fb-4c4b-a2f7-5e7c0fc9f2eb)

[**Videó**](https://www.youtube.com/watch?v=tF_Jre2SCm4)

## Telepítés és indítás <a name="run"></a>

A házifeladat megvalósítása során [ROS Noetic](http://wiki.ros.org/noetic/Installation)-et és Windows WSL2 Linux subsystemet használtunk Ubuntu-20.04 disztribúcióval.
A ROS feltelepítéséhez nagy segítségget nyújtott a [How to Install ROS Noetic on Ubuntu 20.04](https://varhowto.com/install-ros-noetic-ubuntu-20-04/) című útmutató.
Mi az ajánlott `full desktop` csomagot töltöttük le:

```bash
sudo apt install ros-noetic-desktop-full
```

Első lépésként a ROS telepítése után be kell tölteni a terminálba a ROS környezetét:

```bash
source /opt/ros/noetic/setup.bash
```

Ha rendelkezünk catkin workspace-el akkor navigáljunk bele.
Ha viszont még nem, akkor a következő parancsok lefuttatásával hozzunk létre és fordítsunk le egy catkin_ws mappát.

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

A turtlebot3 szimulációhoz használt alapcsomagjainak GIT repoját töltsük be a workspace-ünk src mappájába:

[turtlebot3_msgs](http://wiki.ros.org/turtlebot3_msgs)

[turtlebot3_simulations](http://wiki.ros.org/turtlebot3_simulations)

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


