clc;clear;

load 'data.mat'
[RX,tX,RY,tY]=pro_point(RAi,tAi,qij,rj,K);


X_est=[RX,tX;0,0,0,1];
Y_th(1:3,4)=Y_th(1:3,4)*1000;
X_th(1:3,4)=X_th(1:3,4)*1000;
X_redef=[X_th(1:3,1:3),X_th(1:3,4)-Y_th(3,4)*X_th(1:3,3);0,0,0,1];

Y_est=[RY,[tY;0];0,0,0,1];
Y_redef=[Y_th(1:3,1:3),[Y_th(1:2,4);0];0,0,0,1];

disp('The true value of original TX:')
disp(X_th)
disp('The true value of redefined TX:')
disp(X_redef)
disp('The estimated value of redefined TX:')
disp(X_est)
disp('The true value of original TY:')
disp(Y_th)
disp('The true value of redefined TY:')
disp(Y_redef)
disp('The estimated value of redefined TY:')
disp(Y_est)