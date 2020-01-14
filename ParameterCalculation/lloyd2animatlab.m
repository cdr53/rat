function F = lloyd2animatlab(x,Tdot,Al,T,Ldot,L,Lr)

% Tdot = [62.6061  315.7766   75.8723   12.4583];
% Al = [0.7931    0.7766    0.8174    0.8872];
% T = [2.5566    8.0898   10.7605   12.3074];
% Ldot = [-0.0110    0.0022    0.0278    0.0188];
% L = [0.0114    0.0113    0.0116    0.0123];
% Lr = 0.0150;

% F(1) = (ks/b)*(kp*(L(1)-Lr)+b*Ldot(1)+(1+kp/ks)*T(1)+Am*Al(1))- Tdot(1);
% F(2) = (ks/b)*(kp*(L(2)-Lr)+b*Ldot(2)+(1+kp/ks)*T(2)+Am*Al(2))- Tdot(2);
% F(3) = (ks/b)*(kp*(L(3)-Lr)+b*Ldot(3)+(1+kp/ks)*T(3)+Am*Al(3))- Tdot(3);
% F(4) = (ks/b)*(kp*(L(4)-Lr)+b*Ldot(4)+(1+kp/ks)*T(4)+Am*Al(4))- Tdot(4);

%x = x.^2;

% F(1) = Tdot(1)-(x(1)./x(2)).*(x(3).*max(0,(L(1)-Lr))+x(2).*Ldot(1)-(1+x(3)./x(1)).*T(1)+x(4).*Al(1));
% F(2) = Tdot(2)-(x(1)./x(2)).*(x(3).*max(0,(L(2)-Lr))+x(2).*Ldot(2)-(1+x(3)./x(1)).*T(2)+x(4).*Al(2));
% F(3) = Tdot(3)-(x(1)./x(2)).*(x(3).*max(0,(L(3)-Lr))+x(2).*Ldot(3)-(1+x(3)./x(1)).*T(3)+x(4).*Al(3));
% F(4) = Tdot(4)-(x(1)./x(2)).*(x(3).*max(0,(L(4)-Lr))+x(2).*Ldot(4)-(1+x(3)./x(1)).*T(4)+x(4).*Al(4));

Feqn = @(ks,b,kp,Am) Tdot-(ks./b).*(kp.*max(0,L-Lr)+b.*Ldot-(1+kp./ks).*T+Am.*Al);
Tdot_eqn = @(ks,b,kp,Am) (ks./b).*(kp.*max(0,L-Lr)+b.*Ldot-(1+kp./ks).*T+Am.*Al);

ks = x(1);
b = x(2);
kp = x(3);
Am = x(4);

F = Feqn(ks,b,kp,Am);
end