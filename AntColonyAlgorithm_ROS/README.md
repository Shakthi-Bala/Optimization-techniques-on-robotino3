# Travelling Salesman Problem - Ant Colony - turtlebot3

Bu çalışmada küresel bir problem haline gelmiş gezgin satıcı problemine karınca kolonisi yaklaşımı kullanılarak oluşturulan algoritmadan bahsedilmektedir. Gezgin satıcı problemi; uzaklıkları bilinen noktaların her birine bir defa gidecek şekilde en az maliyetli rotayı hesaplama problemidir. Gezgin satıcı problemine kesin ve sezgisel olarak çeşitli çözüm yolları geliştirilmekle birlikte NP-zor problemi olduğu bilinmektedir. Bu çalışmada geliştirilen algoritma tsp problemine çözüm olmakla birlikte ros-turtlebot3 üzerinden çalıştırılmıştır. 

In this study, the algorithm created by using the ant colony approach to the traveling salesman problem, which has become a global problem, is mentioned. Traveling salesman problem; It is the problem of calculating the least costly route to go once to each of the points whose distances are known. Although exact and heuristic solutions to the traveling salesman problem have been developed, it is known to be an NP-hard problem. The algorithm developed in this study is a solution to the traveling salesman problem and ros-turtlebot3' has been applied.

![image](https://user-images.githubusercontent.com/78980365/130975135-53f46ac0-a3ba-4cfc-bac5-5285dafdcf2f.png)

The developed algorithm is supported by the fuzzy model and can make different calculations according to the situation conditions. Our changed terms and conditions are as follows;
 - pheromone coefficient
 - intuitive coefficient
 - set of nodes
 - heuristic value between nodes i and j
 - The pheromone value between nodes i and j.
The above-mentioned variables (optimally adjusted) are as follows.

![image](https://user-images.githubusercontent.com/78980365/130976662-1c65d072-96bc-42c7-a7a5-0754f1a1a4f0.png)


**Installing required packages**

**You can follow the steps below to install the necessary simulation packages of TurtleBot3.**

**1. Open Terminal**

**2. Go to the source folder of the workspace (cd catkin_ws/src)**

**3. Copy the codes below into the terminal.**

     git clone https://github.com/ROBOTIS-GIT/turtlebot3_msgs.git
     
     git clone https://github.com/ROBOTIS-GIT/turtlebot3.git -b noetic-devel
     
     git clone https://github.com/ROBOTIS-GIT/turtlebot3_simulations.git -b noetic-devel    

**4. Go back to the workspace (cd ~/catkin_ws) and compile (catkin_make).**

**Practice stages of launcher packages required for ROS**

>1. Run any gazebo launcher that you create yourself or that comes pre-built in turtlebot3 packages.

>2. Then, in the turtlebot3_navigation.launcher file, replace the previously scanned map map.yaml with your own map (ex: mymap.yaml) name. After the necessary preparations are made, the TSP algorithm can be applied.

>3. rosrun  ROS_Amt_tsp.py

**IMPORTANT NOTE: The first position point to be entered must always be the starting point of the robot.**
**For example: Start Point 0,0
             Target Point: It can be any location on the map.**

Youtube: https://youtu.be/LUaYieqgho8
