function [dU] = MPC(wk,xk,Phi,G,N1,N2,Nu,Lambda,m)
 cvx_begin quiet
 variable dU(m*Nu)
 Y = Phi*xk + G*dU;
 W = repmat(wk,size(Y,1)/m,1);
 OBJ = (W-Y)'*(W-Y) + Lambda*dU'*dU;
 minimize(OBJ)
 % The following lines add constrains on U
 dUMIN = -5*ones(size(dU,1),1);
 dUMAX = 5*ones(size(dU,1),1);
 subject to
 dUMIN <= dU <= dUMAX;
 cvx_end
end