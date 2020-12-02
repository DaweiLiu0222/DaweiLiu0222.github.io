This is a feedforward and P controller.


Initial condition: 
%         Chassis                  Joint            Wheel
Config = [pi/6,-0.6,0,           0,-1,-2,0,0,        0,0,0,0]; 

Block: 
initial (x,y,theta) = (0.5,0,0)
Final   (x,y,theta) = (0,-0.5,-pi/2)

And Kp = 1.