function temp_MLSopenchain(obj)

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

%Now to actually move the attachments through the motion. This loop builds a post-motion cell array for each body
for i = 1
    a = pelvisCR*(femurPos+hipJointRotMat(:,:,i)*femurCR*femurCOM)-hippos;
    b = pelvisCR*(femurPos+hipJointRotMat(:,:,i)*femurCR*...
        (tibiaPos+kneeJointRotMat(:,:,i)*tibiaCR*tibiaCOM))-hippos;
    c = pelvisCR*(femurPos+hipJointRotMat(:,:,i)*femurCR*...
        (tibiaPos+kneeJointRotMat(:,:,i)*tibiaCR*...
        (footPos+ankleJointRotMat(:,:,i)*footCR*footCOM)))-hippos;
end

gsl1 = [eye(3) a;zeros(1,3) 1];
gsl2 = [eye(3) b;zeros(1,3) 1];
gsl3 = [eye(3) c;zeros(1,3) 1];

moi_holder = .01*eye(3);

femurMass = obj.body_obj{2}.mass/1000;
tibiaMass = obj.body_obj{3}.mass/1000;
footMass = obj.body_obj{4}.mass/1000;

gimat(:,:,1) = [femurMass*eye(3) zeros(3,3);zeros(3,3) moi_holder];
gimat(:,:,2) = [tibiaMass*eye(3) zeros(3,3);zeros(3,3) moi_holder];
gimat(:,:,3) = [footMass*eye(3) zeros(3,3);zeros(3,3) moi_holder];


keyboard
end