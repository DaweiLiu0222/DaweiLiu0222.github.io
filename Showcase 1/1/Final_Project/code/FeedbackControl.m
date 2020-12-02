function [V,Adxixd_Vd,Vd,Xerr,Incre_err] = FeedbackControl(X,Xd,Xd_next,Kp,Ki,t)
% Example Input:
% clc;clear all
% Xd = [[0,0,1,0.5];[0,1,0,0];[-1,0,0,0.5];[0,0,0,1]];
% Xd_next = [[0,0,1,0.6];[0,1,0,0];[-1,0,0,0.3];[0,0,0,1]];
% X = [[0.17,0,0.985,0.387];[0,1,0,0];[-0.985,0,0.17,0.57];[0,0,0,1]];
% t = 0.01;
% Kp = 0;
% Ki = 0;
% robot_config = [0,0,0,0,0,0.2,-1.6,0];
% [V,Adxixd_Vd,Vd,Xerr,Incre_err]= FeedbackControl(X,Xd,Xd_next,Kp,Ki,t)
%
% Output:
% 
% V =
% 
%          0
%          0
%          0
%    21.4187
%          0
%     6.4556
% 
% Adxixd_Vd =
% 
%          0
%          0
%          0
%    21.4187
%          0
%     6.4556
% 
% Vd =
% 
%          0
%          0
%          0
%    20.0000
%          0
%    10.0000
% 
% Xerr =
% 
%          0
%     0.1710
%          0
%     0.0795
%          0
%     0.1068
% 
% Incre_err =
% 
%          0
%     0.0017
%          0
%     0.0008
%          0
%     0.0011


% Feedforward reference twist: Vd
IDDN = inv(Xd)*Xd_next;
ML = MatrixLog6(IDDN)/t;
Vd = se3ToVec(ML);

% The commanded end-effector twist V expressed in the end-effector frame {e}

IXD = inv(X)*Xd;
Adxixd_Vd = Adjoint(IXD)*Vd;

% The error twist Xerr and the increment of the error
Xerr = se3ToVec(MatrixLog6(IXD));
Incre_err = Xerr*t;

% Twist
V = Adxixd_Vd + Kp*Xerr + Ki*Incre_err;
end

