This is a feedforward and PI controller.


Initial condition: 
%         Chassis                  Joint            Wheel
Config = [pi/6,-0.5,0,           0,0,0,0,0,        0,0,0,0]; 

Block: 
initial (x,y,theta) = (1,0,0)
Final   (x,y,theta) = (0,-1,-pi/2)

And Kp = 1; Ki = 10