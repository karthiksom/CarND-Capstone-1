The goal of this project is to program the Udacity Carla vehicle so that it can detect and respond to traffic lights in real time when driving in a simulator environment and a real environment.

The connection of the different ROS nodes follows the descriptions shown below in the Block diagram.
<img src="https://github.com/karthiksom/CarND-Capstone-1/blob/master/imgs/Overview.JPG" width="100%">
Following are the additional Subscriber-Publisher transactions.

| Subscriber          | Publisher                     | Signal Name    | Purpose                                       |
|---------------------|-------------------------------|----------------|-----------------------------------------------|
| DBW Node            | Traffic light detection node  | tld_enabled    | Sends the Traffic light detected              |
| DBW Node            | Waypoint Updater              | red_light_near | Checks if its with in the near 100 waypoints  |


Most of the code was taken from the Udacity lectures. The code above for adding additional Subscriber and Publisher was done separately.

###Setup used:

1. Udacity workspace:
   Some issues faced with respect to communication delay between different  modules. 
2. Windows 7 Host OS- Ubuntu 16.04 using Virtual Box setup.
   This setup showed some improvements compared to udacity workspace. But still there were some communication delay between different modules. 
   
### Issues Faced

1. Car oscillating between lanes and not following the waypoints.
      - Optimizing the PID controller helped to solve this problem to a certain extent
      - Reducing the frequency rate of waypoint updater to 30 Hz. This matches the frequency of Waypoint follower node.
2. Car not stopping at the Red light signal.
      - This was solved by adding tld_enabled signal to the Traffic Detector block. The block once it detects the red signal, updates this line to TRUE. This is then received by DBW Node and send to Twist controller for applying breaks.
3. Car Stopping at highway well before the Traffic signal detected in the waypoints.
      - This problem should only visible in Simulator as the Get signal state function reads it from a file. Once Traffic Detector block updates the tld_enabled signal to TRUE, the break is applied
      - Solution: The Waypoint updater block detects if the red signal detected is with in the next 100 waypoints. If so it updates the red_light_near signal to DBW Node. The DBW Node then compares if both tld_enabled and red_light_near signal are TRUE. If so sends the information to Twist controller for applying breaks.
      
 Following issues still needs a fix:
 
 1. Car stops well before the Red light signal and not just before the signal line.
        - I tried to solve the problem by having a check if the red light is below or equal to 40 waypoints, then update the red_light_near signal. This introduced some indecision in the simulator environment. When it is too close to traffic lights, the decision to apply breaks and stop was flipping with moving forward decision. So reverted back this decision
 
 2. Car oscillations are controlled completely. 
 
 Following were not tested
 
 1. The code was not tested on the second track. So traffic classification problem was not implemented.
 
 
      

