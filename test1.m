musc = obj.musc_obj{12};
steep = musc.steepness;
xoff = musc.x_off;
fmax = musc.max_force;
Lr = musc.RestingLength;

inF = -5*fmax:.01:(5*fmax);

V = @(A1,A2,A3,A) A1-(1/A3).*log((A2-A)./A);

Vb = -60e-3;
G = 100e-9;

Vvec = real(V(xoff,fmax,steep,inF));

I = (Vvec-Vb).*G;