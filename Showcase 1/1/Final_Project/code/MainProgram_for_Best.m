clc;clear all
T_sei = [[0 ,0, 1, 0]; [ 0,  1, 0,  0]; [-1, 0, 0,   0.5]; [0, 0, 0, 1]];
T_sci = [[1 ,0, 0, 1]; [ 0,  1, 0,  0]; [ 0, 0, 1, 0.025]; [0, 0, 0, 1]];
T_scf = [[0 ,1, 0, 0]; [-1,  0, 0, -1]; [ 0, 0, 1, 0.025]; [0, 0, 0, 1]];
T_ces = [[-0.707 ,0, 0.707, 0]; [ 0,  1, 0,  0]; [-0.707, 0, -0.707,  0.1]; [0, 0, 0, 1]];
T_ceg = [[-0.707 ,0, 0.707, 0]; [ 0,  1, 0,  0]; [-0.707, 0, -0.707,  0.005]; [0, 0, 0, 1]];
k = 1;
[Output] = TrajectoryGenerator(T_sei,T_sci,T_scf,T_ceg,T_ces,k);

%Fixed offset of the chassis frame {b} to the base frame of the arm{0}
Tb0=[1,0,0,0.1662;0,1,0,0;0,0,1,0.0026;0,0,0,1];

%The end-effector frame {e} relative to the arm base frame {0} 
M0e=[1,0,0,0.033
     0,1,0,0;
     0,0,1,0.6546;
     0,0,0,1];
Blist = [[0;  0; 1;         0; 0.033;   0], ...
         [0; -1; 0;   -0.5076;     0;   0], ...
         [0; -1; 0;   -0.3526;     0;   0], ...
         [0; -1; 0;   -0.2176;     0;   0], ...
         [0;  0; 1;         0;     0;   0]];
     
% Creating list for chassis/arm/wheel/grasper_state

gslist = Output(:,13);

% Initial condition
SofO = size(Output);
Nofline = SofO(1,1)-1;
w = 0.15;
l = 0.235;
r = 0.0475;
H0 = (1/r)*[[-l-w, 1, -1];[l+w, 1, 1];[l+w, 1, -1];[-l-w, 1, 1]];
F = pinv(H0);
F6 = [[0,0,0,0];[0,0,0,0];F;[0,0,0,0]];
dt = 0.01;
Kp = 1;
Ki = 0;
MaxAS = 12.3;
% Random inital configuration
%         Chassis               Joint            Wheel
Config = [pi/6,-0.6,0,           0,-1,-2,0,0,        0,0,0,0];    

Tsb = [cos(Config(1)), -sin(Config(1)), 0,    Config(2);
       sin(Config(1)),  cos(Config(1)), 0,    Config(3);
                    0,               0, 1,       0.0963;
                    0,               0, 0,            1];
theta = Config(4:8)';
T0e = FKinBody(M0e,Blist,theta);
X = Tsb*Tb0*T0e;

% Creating zero actual configuration matrix and first row
Actual = zeros(Nofline,13);
Actual(1,1:12) = Config;
Actual(1,13) = 0;
XerrMat = zeros(Nofline,6);
VMat = zeros(Nofline,6);
for i = 1 : Nofline
    % The pseudoinverse of the mobile manipulator Jacobian Je(?)
    TseJ = X;
    phiJ = Config(1);
    xJ = Config(2);
    yJ = Config(3);
    TsbJ = [[cos(phiJ),-sin(phiJ), 0, xJ];
            [sin(phiJ), cos(phiJ), 0, yJ];
            [0,0,1,0.0963];
            [0,0,0,1]];
        
    T0eJ = FKinBody(M0e,Blist,Config(4:8)');
    TebJ = inv(T0eJ)*inv(Tb0);
    JbJ = Adjoint(TebJ)*F6;

    thetalistJ = Config(4:8)';
    JaJ = JacobianBody(Blist, thetalistJ);
    Je = [JbJ JaJ];
    
    % ith xd
    Xd = [[Output(i,1:3),Output(i,10)];
          [Output(i,4:6),Output(i,11)];
          [Output(i,7:9),Output(i,12)];
          [      0, 0, 0,           1]];
    % ith xd_next
    Xd_nt = [[Output(i+1,1:3),Output(i+1,10)];
             [Output(i+1,4:6),Output(i+1,11)];
             [Output(i+1,7:9),Output(i+1,12)];
             [        0, 0, 0,             1]];
   
    % ith FC(X_ith,Xd_ith,Xd_next_ith) to generate u and thetadot
    [V,Adxixd_Vd,Vd,Xerr,Incre_err] = FeedbackControl(X,Xd,Xd_nt,Kp,Ki,dt);
    VMat(i,:) = V';
    XerrMat(i,:) = Xerr';
    % Commanded wheel and arm joint speeds
     ajs = pinv(Je)*V;
     AJS = ajs';   

    Speed = [AJS(5:9), AJS(1:4)]; % Speed = [thetadot,u]
    
    % ith (configuration and speed) to generate (i+1)th configuration
    [Config] = NextState(Config,Speed,dt,MaxAS);
    
    % Record (i+1)th configuration in Actual matrix
    Actual(i+1,1:12) = Config;
    Actual(i+1,13) = gslist(i+1);
    
    % Find actual (i+1)th Tse beasd on (i+1)th configuration
    Tsb_ac = [[cos(Config(1)), -sin(Config(1)), 0, Config(2)];
              [sin(Config(1)),  cos(Config(1)), 0, Config(3)];
              [             0,               0, 1,    0.0963];
              [             0,               0, 0,         1]];
    T0e_ac = FKinBody(M0e,Blist,(Config(1,4:8))');
    X = Tsb_ac*Tb0*T0e_ac;
    
end
csvwrite('Best.csv', Actual);
csvwrite('Xerr_Best.csv', XerrMat);

xerr1 = XerrMat(:,1)';
xerr2 = XerrMat(:,2)';
xerr3 = XerrMat(:,3)';
xerr4 = XerrMat(:,4)';
xerr5 = XerrMat(:,5)';
xerr6 = XerrMat(:,6)';
t = 0.01 * linspace(1,2125,2125);

plot(t,xerr1)
hold on
plot(t,xerr2)
hold on
plot(t,xerr3)
hold on
plot(t,xerr4)
hold on
plot(t,xerr5)
hold on
plot(t,xerr6)
hold off
title('Plot of Xerr')
xlabel('Time(s)')
ylabel('Xerr')
legend('Xerr1','Xerr2','Xerr3','Xerr4','Xerr5','Xerr6','Location','southeast' )

    
    
    
    
    
    
    
    
    
    
