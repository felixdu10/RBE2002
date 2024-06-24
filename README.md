# RBE2002
Final Project video:

https://youtu.be/RgbbjsHG65M



RBE2002 Demo robot capabilities:

•	Two robots are going up to the ramp, the B robot will stop at a specific point as a counterweight and send a message “READY” to MQTT. 

•	Robot A can receive the message and go to the top. Otherwise, the ramp is going to change from a slope to flat, robot A will not be able to go up. [As shown in the figure below (2,4)]

•	Robot C is going to go to (5,2) and face at the April tag on beacon button and push that button. 

•	Then there will be a April tag shown on (2,4), robot A is going to detect the tag and send the tag information to MQTT. 

•	Robot D will receive the tag information and go to (2,1), open the escape door and be into the escape door. 

•	Robot B is going to do a U turn at the ramp and go back in to the escape room as well.

•	We also changed our algorithm so that the robot would look for the ramp, (if the ramp is at a different place) and go up the ramp. Robot D could go to any other coordinates as we put in MQTT broker.




![image](https://github.com/felixdu10/RBE2002/assets/146391322/9a18857e-e37a-4623-bd35-aa282ef04be0)
