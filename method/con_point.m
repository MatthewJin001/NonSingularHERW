function [pointRXpro,pointtXpro,pointRYpro,pointtYpro]=con_point(eRibn,etibn,qijn,pattern,K)

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


fx=K(1,1);
fy=K(2,2);

r=pattern;



%%
% Reasonable Initialization. Ref： Section V.B in the paper.

x=zeros(9+3+3,1);
C=zeros(3,15,n);

CTC=zeros(15,15);
CTt=zeros(15,1);

for i=1:n
    C(:,:,i)=[kron(dim(tB(:,i))',eye(3)), -RA(:,:,i)];
    CTC=CTC+C(:,:,i)'*C(:,:,i);
    CTt=CTt+C(:,:,i)'*tA(:,i);
end

x=inv(CTC)*CTt;

[U,S,V]=svd(reshape(x(1:9),3,3));
RXini=(sign(det(U)*det(V'))*U*V')';
tXini=-RXini*x(10:12);
tYini=x(13:15);
RYini=RA(:,:,1)'*RXini'*RB(:,:,1);


iter_max=25;
step_th=1e-6;
k_num=1;
step=1000;

RX_k=RXini;
RY_k=RYini;

tX_k=tXini;
tY_k=tYini;

RX_k_list=dcm2rodv(RXini);
RY_k_list=dcm2rodv(RYini);
tX_k_list=tXini;
tY_k_list=tYini;



while (step_th<step && k_num<iter_max)

    k_num=k_num+1;

    J=zeros(2,12);
    g=zeros(2,1);
    
    beta=zeros(3,1);
    gamma=zeros(3,1);

    JTJ=zeros(12,12);
    JTg=zeros(12,1);

    for i=1:n
        for  j=1:m
        gamma=RX_k*RA(:,:,i)*RY_k*r(:,j)+RX_k*RA(:,:,i)*tY_k+RX_k*tA(:,i);
        beta=gamma+tX_k;

        pgpb=1/beta(3)*[fx,0,-fx*beta(1)/beta(3);...
                         0,fy,-fy*beta(2)/beta(3);];
        J=pgpb*[-skew(gamma),eye(3),-RX_k*RA(:,:,i)*skew(RY_k*r(:,j)),RX_k*RA(:,:,i)];
        g=homo(K*beta);
         
        JTJ=JTJ+J'*J;
        JTg=JTg+J'*(qijn(:,i,j)-g);
        end
    end

    delta=inv(JTJ)*JTg;
    delta_RX=delta(1:3);
    delta_tX=delta(4:6);

    delta_RY=delta(7:9);
    delta_tY=delta(10:12);

    %更新
    RX_k=expm(skew(delta_RX))*RX_k;
    RY_k=expm(skew(delta_RY))*RY_k;
    tX_k=tX_k+delta_tX;
    tY_k=tY_k+delta_tY;

    RX_k_list=[RX_k_list,dcm2rodv(RX_k)];
    RY_k_list=[RY_k_list,dcm2rodv(RY_k)];
    tX_k_list=[tX_k_list,tX_k];
    tY_k_list=[tY_k_list,tY_k];



    step=norm(delta_RX);

end


pointRXpro=rod2dcmv(RX_k_list(:,end));
pointtXpro=tX_k_list(:,end)*1000;
pointRYpro=rod2dcmv(RY_k_list(:,end));
pointtYpro=tY_k_list(:,end)*1000;


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


