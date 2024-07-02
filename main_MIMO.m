clear
clc
num11 = [1, 0]; den11 = [1, -0.5];
num12 = [0.8]; den12 = [1, -1]; 
num21 = [0.5]; den21 = [1, -1]; 
num22 = [1, 0.5]; den22 = [1, -1.2, -0.5]; 

% z-transform
ts = 0.1
sys11 = tf(num11, den11, ts);
sys12 = tf(num12, den12, ts);
sys21 = tf(num21, den21, ts);
sys22 = tf(num22, den22, ts);

% MIMO system
sys = [sys11, sys12; sys21, sys22]

[Ap, Bp, Cp, Dp] = ssdata(sys);
nx=size(Ap,1); m=size(Bp,2); p=size(Cp,1);

A = [Ap zeros(nx,m);
    Cp eye(m,m)];
B = [Bp; zeros(m,m)];
C = [Cp eye(p,m)];
n = size(A,1);
% Thus the state vector is
% [xk; u(k-1)]
N1=1; N2=5; Nu=3; Lambda=0.1;
Phi = [C*A; C*A^2; C*A^3; C*A^4; C*A^5];
G = [C*B zeros(m,m) zeros(m,m);
    C*A*B C*B zeros(m,m);
    C*A^2*B C*A*B C*B;
    C*A^3*B C*A^2*B C*A*B;
    C*A^4*B C*A^3*B C*A^2*B];

%% Test
tsim = 60;
SetPt = [0 ones(1,tsim/4) ones(1,tsim/4) ones(1,tsim/2);
         0 zeros(1,tsim/4) ones(1,tsim/4) ones(1,tsim/2)];
dist = [0 zeros(1,tsim/2) -0.2*ones(1,tsim/2);
        0 zeros(1,tsim/2) -0.2*ones(1,tsim/2)];
for k=1:tsim

 if (mod(k,10)==0), k, end

 % measure the plant state and the set-point
 wk = SetPt(:,k);
 if k > 1
 xold = xold+dxk; uold = uk; yold = yk;
 dxk = x-xold;
 else
 dxk = zeros(nx,1); xold = dxk; yold = zeros(m,1); uold = yold;
 end
 yk = Cp*(xold+dxk) + dist(:,k);

 % Compute the MPC control signal
 %
 % For unconstrained MPC, we have closed-form
 % solution as follows:

 % Alternatively, use CVX which can solve
 % MPC with constraints, and other more general
 % formulations

 x_mpc = [dxk; yold];
 dU = MPC2(wk,x_mpc,Phi,G,N1,N2,Nu,Lambda,m);

 % Apply control u(k) to the plant
 du = dU(1:m);
 uk = uold + du;
 x = Ap*(xold+dxk) + Bp*uk;

 % Save the plant y, u and set-point for plotting later
 y_plot1(k) = yk(1,:);
 y_plot2(k) = yk(2,:);
 u_plot1(k) = uk(1,:);
 u_plot2(k) = uk(2,:);
 w_plot1(k) = wk(1,:);
 w_plot2(k) = wk(2,:);
end

subplot(2,2,1),stairs(w_plot1),hold on,plot(y_plot1),ylabel('Y1')
subplot(2,2,2),stairs(u_plot1),ylabel('U1')
subplot(2,2,3),stairs(w_plot2),hold on,plot(y_plot2),ylabel('Y2')
subplot(2,2,4),stairs(u_plot2),ylabel('U2')