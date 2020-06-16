

baseFrame = obj.joint_obj{1}.sim_position_profile(1,:)';
w = [obj.joint_obj{1}.uuw_joint(:,1) obj.joint_obj{2}.uuw_joint(:,1) obj.joint_obj{3}.uuw_joint(:,1)];
q = [obj.joint_obj{1}.sim_position_profile(1,:)' obj.joint_obj{2}.sim_position_profile(1,:)' obj.joint_obj{3}.sim_position_profile(1,:)']-baseFrame;
twists = [-cross(w,q);w];

% COMs relative to hip position
hipJointRotMat = obj.joint_obj{1}.joint_rotmat_profile;
kneeJointRotMat = obj.joint_obj{2}.joint_rotmat_profile;
ankleJointRotMat = obj.joint_obj{3}.joint_rotmat_profile;

pelvisCR = obj.CR_bodies(:,:,1);
femurCR = obj.CR_bodies(:,:,2);
tibiaCR = obj.CR_bodies(:,:,3);
footCR = obj.CR_bodies(:,:,4);

%pelvisPos = obj.pos_bodies(:,1);
femurPos = obj.pos_bodies(:,2);
tibiaPos = obj.pos_bodies(:,3);
footPos = obj.pos_bodies(:,4);

femurCOM = obj.body_obj{2}.com';
tibiaCOM = obj.body_obj{3}.com';
footCOM = obj.body_obj{4}.com';

hippos = obj.joint_obj{1}.init_pos_w;

Rwp = [[pelvisCR;[0,0,0]],[0;0;0;1]];
Rpf = [[femurCR;[0,0,0]],[obj.body_obj{2}.position'*1000;1]];
Rft = [[tibiaCR;[0,0,0]],[obj.body_obj{3}.position'*1000;1]];
Rtp = [[footCR;[0,0,0]],[obj.body_obj{4}.position'*1000;1]];

femurCOM_w = Rwp*Rpf*[femurCOM*1000;1];
tibiaCOM_w = Rwp*Rpf*Rft*[tibiaCOM*1000;1];
footCOM_w = Rwp*Rpf*Rft*Rtp*[footCOM*1000;1];

% Composition of transform frames for the center of motion in each link (MLS p.172)
% Format: top left is rotation frame relative to global
% Format: right is position of COM in global coordinates
% In the paper, they are able to align the tool frames with the body frame so top left is I. We can't do that here.
gsl0 = [[pelvisCR;[0,0,0]],[0;0;0;1]];
gsl1 = [[pelvisCR*femurCR;[0 0 0]],femurCOM_w];
gsl2 = [[pelvisCR*femurCR*tibiaCR;[0 0 0]],tibiaCOM_w];
gsl3 = [[pelvisCR*femurCR*tibiaCR*footCR;[0 0 0]],footCOM_w];

%%
gsl1 = [eye(3) a;zeros(1,3) 1];
gsl2 = [eye(3) b;zeros(1,3) 1];
gsl3 = [eye(3) c;zeros(1,3) 1];

femurMass = obj.body_obj{2}.mass/1000;
tibiaMass = obj.body_obj{3}.mass/1000;
footMass = obj.body_obj{4}.mass/1000;

% Femur MOI
    r = 1.7751e-3; L = 38.6521e-3; % measured from the simulation
    Ix = (1/12)*femurMass*L^2;
    Iy = Ix;
    Iz = (1/2)*femurMass*r^2;
    MOI(:,1) = [Ix;Iy;Iz];
% Tibia MOI
    r = 1.3745e-3; L = 41.46e-3; % measured from the simulation
    Ix = sqrt(((1/12)*tibiaMass*L^2)^2+((1/2)*tibiaMass*r^2));
    Iy = Ix;
    Iz = (1/12)*tibiaMass*L^2;
    MOI(:,2) = [Ix;Iy;Iz];
% Foot MOI
    h = 2.846e-3; w = 9.486e-3; d = 20.19e-3; % measured from the simulation
    Ix = (1/12)*footMass*(d^2+h^2);
    Iy = (1/12)*footMass*(d^2+w^2);
    Iz = (1/12)*footMass*(w^2+h^2);
    MOI(:,3) = [Ix;Iy;Iz];
    

gimat(:,:,1) = [femurMass*eye(3) zeros(3,3);zeros(3,3) diag(MOI(:,1))];
gimat(:,:,2) = [tibiaMass*eye(3) zeros(3,3);zeros(3,3) diag(MOI(:,2))];
gimat(:,:,3) = [footMass*eye(3) zeros(3,3);zeros(3,3) diag(MOI(:,3))];

    function Rout = rotMat(ang,dim)
        ang = ang*(pi/180);
        if strfind(dim,'-')
            dim = dim(dim~='-');
            augMat = [1 -1 -1;-1 1 -1;-1 -1 1];
        else
            augMat = ones(3,3);
        end
        if strcmp(dim,'X') || strcmp(dim,'x')
            Rout = [1 0 0; 0 cos(ang) -sin(ang);0 sin(ang) cos(ang)];
        elseif strcmp(dim,'Y') || strcmp(dim,'y')
            Rout = [cos(ang) 0 sin(ang); 0 1 0;-sin(ang) 0 cos(ang)];
        elseif strcmp(dim,'Z') || strcmp(dim,'z')
            Rout = [cos(ang) -sin(ang) 0 ; sin(ang) cos(ang) 0;0 0 1];
        else
            warning('No dimension term supplied as a second input. Please specify ''x'',''y'', or ''z''')
            return
        end
       Rout = augMat.*Rout; 
    end