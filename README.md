# RobotLab

This project was dubbed "the robot race". We were given a simulation of a robot in a room.
This robot had a realistic physical attributes so it would topple over if it turned while going to fast. 
The challange was that the programmer was given a path that the robot had to follow as quickly as possible.
The path was unknown until examination.

Our robot utilized a technique called "Pure pursuit" where it navigated by calculating the optimal curve to 
the next desired point. (Often the furthest visible point on the track). Pure pursuit does not follow the path as 
exactly but this was not the point: it sacrifices accuracy for speed since it is turning as much as possible without
toppling over, giving the robot near top-speed at all times.
