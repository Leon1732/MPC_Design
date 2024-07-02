clear
clc

bz = [0.5]; az=[1 -0.8];
ts=0.1; %this is a discrete-time model, so sampling time is arbitary
Gz=tf(bz,az,ts)

Gz = ss(Gz);
Ap=Gz.A; Bp=Gz.B; Cp=Gz.C; Dp=Gz.D;
nx=size(Ap,1); m=size(Bp,2); p=size(Cp,1);

br = [0.7]; ar=[1 -0.6];

% Eq(6) in L2
A = [Ap zeros(nx,m);
    Cp eye(m,m)];
B = [Bp; zeros(m,m)];
C = [Cp eye(p,m)];
n = size(A,1);

%The tuning parameters
N1=1; N2=2; Nu=1; Lambda=0;
Phi = [];
G = [];
for i = 1:N2
    Phi = [Phi; C*A^i];
end
for i = 0:N2-1
    G = [G; C*A^i*B];
end

%% Test
tsim = 60;
SetPt = [0 ones(1,tsim/4) ones(1,tsim/4) ones(1,tsim/2)];
dist = [0 zeros(1,tsim/2) -0.4*ones(1,tsim/2)];
for k=1:tsim

 if (mod(k,10)==0), k, end

 % measure the plant state and the set-point
 wk = SetPt(k);
 if k > 1
 xold = xold+dxk; uold = uk; yold = yk;
 dxk = x-xold;
 else
 dxk = zeros(nx,1); xold = dxk; yold = zeros(m,1); uold = yold;
 end
 yk = Cp*(xold+dxk) + dist(k);

% The state vector is [dxk; y(k-1)]
 x_mpc = [dxk; yold];
 dU = MPC1(wk,x_mpc,Phi,G,N1,N2,Nu,Lambda,m);

 % Apply control u(k) to the plant
 du = dU(1);
 uk = uold + du;
 umax = 3;
 umin = 0;
 if uk > umax
     uk = umax;
 elseif uk <= umin
     uk = umin;
 end
 x = Ap*(xold+dxk) + Bp*uk;

 % Save the plant y, u and set-point for plotting later
 y_plot(k) = yk;
 u_plot(k) = uk;
 w_plot(k) = wk;
end

subplot(211),stairs(w_plot),hold on,plot(y_plot),ylabel('Y')
subplot(212),stairs(u_plot),ylabel('U')