function [pointRXpro,pointtXpro,pointRYpro,pointtYpro]=pro_point(eRibn,etibn,qijn,pattern,K)

n=size(qijn,2);
m=size(qijn,3);

RA=eRibn;
tA=etibn;
TA=cat(1,cat(2,RA,reshape(tA,3,1,n)),repmat([0,0,0,1],[1,1,n]));

% Camera Pose Estimation. Ref： APPENDIX.A in the paper.

[cRpList,ctpList] = extrinsicEst(qijn,pattern,K);
RB=cRpList;
tB=ctpList;
TB=cat(1,cat(2,RB,reshape(tB,3,1,n)),repmat([0,0,0,1],[1,1,n]));


sRA=zeros(2,2,n);
for i=1:n
    sRA(:,:,i)= RA(1:2,1:2,i);
end

I23=[eye(2),zeros(2,1)];

fx=K(1,1);
fy=K(2,2);

r=pattern;

% Reasonable Initialization. Ref： Section V.B in the paper.

x=zeros(9+3+2,1);
C=zeros(3,14,n);

CTC=zeros(14,14);
CTt=zeros(14,1);

for i=1:n
    C(:,:,i)=[kron(dim(tB(:,i))',eye(3)), -I23'*sRA(:,:,i)];
    CTC=CTC+C(:,:,i)'*C(:,:,i);
    CTt=CTt+C(:,:,i)'*tA(:,i);
end

x=inv(CTC)*CTt;

[U,S,V]=svd(reshape(x(1:9),3,3));
RXini=(sign(det(U)*det(V'))*U*V')';
stXini=-RXini*x(10:12);
stYini=x(13:14);

RYini=RA(:,:,1)'*RXini'*RB(:,:,1);

%%
%para  for iteration
iter_max=25;
step_th=1e-6;
k_num=1;
step=1000;

RX_k=RXini;
RY_k=RYini;

stX_k=stXini;
stY_k=stYini;

RX_k_list=dcm2rodv(RXini);
RY_k_list=dcm2rodv(RYini);
stX_k_list=stXini;
stY_k_list=stYini;

while (step_th<step && k_num<iter_max)

    k_num=k_num+1;

    sJ=zeros(2,11);
    sg=zeros(2,1);

    sbeta=zeros(3,1);
    sgamma=zeros(3,1);

    sJTJ=zeros(11,11);
    sJTg=zeros(11,1);



    for i=1:n
        for  j=1:m
            sgamma=RX_k*RA(:,:,i)*RY_k*r(:,j)+RX_k*I23'*sRA(:,:,i)*stY_k+RX_k*tA(:,i);
            sbeta=sgamma+stX_k;

            pgpb=1/sbeta(3)*[fx,0,-fx*sbeta(1)/sbeta(3);...
                0,fy,-fy*sbeta(2)/sbeta(3);];
            sJ=pgpb*[-skew(sgamma),eye(3),-RX_k*RA(:,:,i)*skew(RY_k*r(:,j)),RX_k*I23'*sRA(:,:,i)];
            sg=homo(K*sbeta);

            sJTJ=sJTJ+sJ'*sJ;
            sJTg=sJTg+sJ'*(qijn(:,i,j)-sg);
        end
    end

    delta=inv(sJTJ)*sJTg;
    delta_RX=delta(1:3);
    delta_stX=delta(4:6);

    delta_RY=delta(7:9);
    delta_stY=delta(10:11);

    %更新
    RX_k=expm(skew(delta_RX))*RX_k;
    RY_k=expm(skew(delta_RY))*RY_k;
    stX_k=stX_k+delta_stX;
    stY_k=stY_k+delta_stY;

    RX_k_list=[RX_k_list,dcm2rodv(RX_k)];
    RY_k_list=[RY_k_list,dcm2rodv(RY_k)];
    stX_k_list=[stX_k_list,stX_k];
    stY_k_list=[stY_k_list,stY_k];

    step=norm(delta_RX);

end


pointRXpro=rod2dcmv(RX_k_list(:,end));
pointtXpro=stX_k_list(:,end)*1000;
pointRYpro=rod2dcmv(RY_k_list(:,end));
pointtYpro=stY_k_list(:,end)*1000;

end


function [cRpList,ctpList] = extrinsicEst(qij,pattern,K)
cRpList=zeros(3,3,size(qij,2));
ctpList=zeros(3,size(qij,2));
for i=1:size(qij,2)
    imagePoints=squeeze(qij(:,i,:));  %2xm
    [cRpList(:,:,i),ctpList(:,i)] = estPNP(imagePoints,pattern,K);
end
end

function [cRp,ctp] = estPNP(imagePoints,worldPoints,K)
m=size(imagePoints,2);
fx=K(1,1);
fy=K(2,2);
cx=K(1,3);
cy=K(2,3);

Ahomo=zeros(2*m,9);
for j=1:m
    Ahomo(2*j-1:2*j,:)=[kron([worldPoints(1:2,j)',1],[1,0,-(imagePoints(1,j)/fx-cx/fx)]);...
        kron([worldPoints(1:2,j)',1],[0,1,-(imagePoints(2,j)/fy-cy/fy)])];
end

[~,~,V]=svd(Ahomo);
alpha=V(:,end);

Xh=[alpha(1),alpha(4),alpha(7);...
    alpha(2),alpha(5),alpha(8);...
    alpha(3),alpha(6),alpha(9)];

lamda1=1/norm(Xh(:,1));
lamda2=1/norm(Xh(:,2));
lamda=(lamda1+lamda2)/2;
if alpha(9)*lamda<0
    lamda=-lamda;
end
Xh=Xh*lamda;
Rh=[Xh(:,1),Xh(:,2),cross(Xh(:,1),Xh(:,2))];

tini=Xh(:,3);

[U,~,V]=svd(Rh);
if det(U*V')>0
    Rini=U*V';
else
    Rini=-U*V';
end

para.imagePoints=imagePoints;
para.worldPoints=worldPoints;
para.K=K;
para.x0=[dcm2rod(Rini)';tini];
options = optimoptions('lsqnonlin','Display','off','StepTolerance',1e-4,'MaxIterations',100,'Algorithm','levenberg-marquardt','ScaleProblem','jacobian');
[x,~,~,~,~] =lsqnonlin(@(x)loss_lm(x,para),para.x0,[],[],options);
cRp=rod2dcm([x(1),x(2),x(3)]);
ctp=[x(4);x(5);x(6);];
end

function [s] = homo(v)
s=v(1:end-1,1)/v(end);
end

function s = expo(v)
phi=v(1:3);rho=v(4:6);
rh=norm(rho,2);
L=cos(rh)*eye(3)+((1-cos(rh))/rh/rh)*rho*rho'+(sin(rh)/rh)*skewsys(rho);
l=(sin(rh)/rh)*phi+((rh-sin(rh))/rh/rh/rh)*rho*rho'*phi+((1-cos(rh))/rh/rh)*skewsys(rho)*phi;
s=[L,l;0,0,0,1];
end

function s = skewsys(v)
s=[0 -v(3) v(2);v(3) 0 -v(1);-v(2) v(1) 0];
end

function [cost]=loss_lm(x,para)
imageP=para.imagePoints;
Points=para.worldPoints;
inK=para.K;
m=size(imageP,2);
R_op=rod2dcm([x(1),x(2),x(3)]);
t_op=[x(4);x(5);x(6);];
cost=zeros(2*m,1);
for j=1:m
    p_temp=R_op*Points(:,j)+t_op;
    cost((j*2-1):(j*2),1)=imageP(:,j)-homo(inK*p_temp);
end
end


function [out] = rod2dcmv(in)
out=rod2dcm(in');
end


function [S] = vex(V)
S = [-V(2,3);V(1,3);-V(1,2);];
end

function [s] = dim(v)
s=[v;1];
end

function [out] = dcm2rodv(in)
out=dcm2rod(in)';
end

function S = skew(V)
S = [0    -V(3)    V(2);
    V(3)      0    -V(1);
    -V(2)    V(1)      0];
end


