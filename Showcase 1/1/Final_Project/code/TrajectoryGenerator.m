function [Output] =   TrajectoryGenerator(T_sei,T_sci,T_scf,T_ceg,T_ces,k)
% Example Input:

% clc;clear all
% T_sei = [[0 ,0, 1, 0]; [ 0,  1, 0,  0]; [-1, 0, 0,   0.5]; [0, 0, 0, 1]];
% T_sci = [[1 ,0, 0, 1]; [ 0,  1, 0,  0]; [ 0, 0, 1, 0.025]; [0, 0, 0, 1]];
% T_scf = [[0 ,1, 0, 0]; [-1,  0, 0, -1]; [ 0, 0, 1, 0.025]; [0, 0, 0, 1]];
% T_ces = [[-0.707 ,0, 0.707, 0]; [ 0,  1, 0,  0]; [-0.707, 0, -0.707,  0.1]; [0, 0, 0, 1]];
% T_ceg = [[-0.707 ,0, 0.707, 0]; [ 0,  1, 0,  0]; [-0.707, 0, -0.707,  0.005]; [0, 0, 0, 1]];
% k = 1;
% [Output] = TrajectoryGenerator(T_sei,T_sci,T_scf,T_ceg,T_ces,k);

% Output:
% 
% 2126 x 13 Matrix (reference trajectory)
%

method = 3;
% Segement 1: Initial to standoff
Xstart1 = T_sei;
Xend1 = T_sci*T_ces;
Tf1 = 8;
N1 = Tf1*k/0.01;
Output1 = zeros(N1,13);
traj1 = ScrewTrajectory(Xstart1, Xend1, Tf1, N1, method);
for i = 1 : N1
Output1(i,:) = [traj1{1,i}(1,1:3), traj1{1,i}(2,1:3), traj1{1,i}(3,1:3), traj1{1,i}(1,4), traj1{1,i}(2,4), traj1{1,i}(3,4), 0];
i = i + 1;
end

% Segement 2: Standoff to grasp
Xstart2 = Xend1;
Translation1 = [1,0,0,0 ;0,1,0,0 ;0,0,1,T_sci(3,4)-Xstart2(3,4);0,0,0,1];
Xend2 = Translation1*Xstart2;
Tf2 = 1;
N2 = Tf2*k/0.01;
Output2 = zeros(N2,13);
traj2 = ScrewTrajectory(Xstart2, Xend2, Tf2, N2, method);
for i = 1 : N2
Output2(i,:) = [traj2{1,i}(1,1:3), traj2{1,i}(2,1:3), traj2{1,i}(3,1:3), traj2{1,i}(1,4), traj2{1,i}(2,4), traj2{1,i}(3,4), 0];
i = i + 1;
end

% Segement 3: Grasper close
N3 = 63;
Output3 = zeros(N3,13);
for i = 1 : N3
Output3(i,:) = [Xend2(1,1:3),Xend2(2,1:3),Xend2(3,1:3),Xend2(1,4),Xend2(2,4),Xend2(3,4),1];
i = i + 1;
end

% Segment 4: Back up
Xstart4 = Xend2;
Translation2 = [1,0,0,0;0,1,0,0;0,0,1,-(T_sci(3,4)-Xstart2(3,4));0,0,0,1];
Xend4 = Translation2*Xstart4;
Tf4 = 1;
N4 = Tf4*k/0.01;
Output4 = zeros(N4,13);
traj4 = ScrewTrajectory(Xstart4, Xend4, Tf4, N4, method);
for i = 1 : N4
Output4(i,:) = [traj4{1,i}(1,1:3), traj4{1,i}(2,1:3), traj4{1,i}(3,1:3), traj4{1,i}(1,4), traj4{1,i}(2,4), traj4{1,i}(3,4), 1];
i = i + 1;
end

% Segment 5: To the Final standoff
Xstart5 = Xend4;
Xend5 = T_scf*T_ces;
Tf5 = 8;
N5 = Tf5*k/0.01;
Output5 = zeros(N5,13);
traj5 = ScrewTrajectory(Xstart5, Xend5, Tf5, N5, method);
for i = 1 : N5
Output5(i,:) = [traj5{1,i}(1,1:3), traj5{1,i}(2,1:3), traj5{1,i}(3,1:3), traj5{1,i}(1,4), traj5{1,i}(2,4), traj5{1,i}(3,4), 1];
i = i + 1;
end

% Segement 6: Final standoff to final cube position
Xstart6 = Xend5;
Translation3 = [1,0,0,0;0,1,0,0;0,0,1,T_scf(3,4)-Xstart6(3,4);0,0,0,1];
Xend6 = Translation3*Xstart6;
Tf6 = 1;
N6 = Tf6*k/0.01;
Output6 = zeros(N6,13);
traj6 = ScrewTrajectory(Xstart6, Xend6, Tf6, N6, method);
for i = 1 : N6
Output6(i,:) = [traj6{1,i}(1,1:3), traj6{1,i}(2,1:3), traj6{1,i}(3,1:3), traj6{1,i}(1,4), traj6{1,i}(2,4), traj6{1,i}(3,4), 1];
i = i + 1;
end

% Segement 7: Grasper open
N7 = 63;
Output7 = zeros(N7,13);
for i = 1 : N7
Output7(i,:) = [Xend6(1,1:3),Xend6(2,1:3),Xend6(3,1:3),Xend6(1,4),Xend6(2,4),Xend6(3,4),0];
i = i + 1;
end

% Segment 8: Back up
Xstart8 = Xend6;
Translation4 = [1,0,0,0;0,1,0,0;0,0,1,-(T_scf(3,4)-Xstart6(3,4));0,0,0,1];
Xend8 = Translation4*Xstart8;
Tf8 = 1;
N8 = Tf8*k/0.01;
Output8 = zeros(N8,13);
traj8 = ScrewTrajectory(Xstart8, Xend8, Tf8, N8, method);
for i = 1 : N8
Output8(i,:) = [traj8{1,i}(1,1:3), traj8{1,i}(2,1:3), traj8{1,i}(3,1:3), traj8{1,i}(1,4), traj8{1,i}(2,4), traj8{1,i}(3,4), 0];
i = i + 1;
end

Output = [Output1;Output2;Output3;Output4;Output5;Output6;Output7;Output8];
end 







