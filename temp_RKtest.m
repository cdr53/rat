h = 1e-3;  % set the step size
t = 0:h:10;  % set the interval of t

% act = zeros(1,length(t));
% act(1) = 0;   % set the intial value for y

% lm = zeros(1,length(t));
% lm(1) = 0;   % set the intial value for y

qdot = zeros(1,length(t));
qdot(1) = 0;   % set the intial value for y

n = length(t)-1;
%act_dot = @(t,act)(___________); %insert function to be solved
% lm_dot_big = @(t,lm,b,lr,kp,Am) (1/b)*((1-4*(L-Lr).^2/(Lr^2))*Am-kp*max((L-Lr),0));
%qdot_dot = @(q,fm) ();

for i = 1:n
    for j = 1:numMusc
        k1 = qdot_dot(q(i),fm(i));
        k2 = qdot_dot(t(i)+.5*h,qdot(i)+.5*k1*h);
        k3 = qdot_dot(t(i)+.5*h,qdot(i)+.5*k2*h);
        k4 = qdot_dot(t(i)+h,qdot(i)+k3*h);
        qdot(i+1) = qdot(i)+((k1+2*k2+2*k3+k4)/6)*h;
    end
end

function outQ = qdot_dot(obj,q)
    Tens = @(L,Am,Ks,Kp,Lr) (Ks/(Ks+Kp))*(max(Kp*(L-Lr),0)+(1-4*(L-Lr).^2/(Lr^2))*Am);
    outLens = muscle_lengths_on_demand(obj,q);
    numMusc = length(obj.musc_obj);
    fm = zeros(numMusc,1);
    Am = fm;
    for ii = 1:numMusc
        muscle = obj.musc_obj{ii};
        Lr = muscle.RestingLength; Kp = muscle.Kpe; Ks = muscle.Kse;
        fm(ii) = Tens(outLens(ii),Am(ii),Ks,Kp,Lr);
    end
    M = compute_mass_matrix(obj,q);
    G = compute_gravity_vector(obj,q);
    R = leg_moment_arms(obj,q)'./1000;
    outQ = inv(M)*(G'+R*fm);
end