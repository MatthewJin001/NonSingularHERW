clc;clear;

addpath('..\method')

load 'data.mat'
[RX,tX,RY,tY]=con_point(RAi,tAi,qij,rj,K);

Y_th(1:3,4)=Y_th(1:3,4)*1000;
X_th(1:3,4)=X_th(1:3,4)*1000;
X_est=[RX,tX;0,0,0,1];
Y_est=[RY,tY;0,0,0,1];

disp('The true value of  TX:')
disp(X_th)
disp('The estimated value of  TX:')
disp(X_est)
disp('The true value of  TY:')
disp(Y_th)
disp('The estimated value of  TY:')
disp(Y_est)
