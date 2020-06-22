function [Mout,G,C] = MLS_maker(obj)
    Adj_func = @(eX) [[eX(1:3,1:3) wedge(eX(1:3,4))*eX(1:3,1:3)];[zeros(3,3) eX(1:3,1:3)]];
    exp_func = @(R,w,v,th) [[R (eye(3)-R)*(cross(w,v))+w*w'*v*th];[zeros(1,3) 1]];

    gWH = [[obj.CR_bodies(:,:,1) [0;0;0]];[zeros(1,3) 1]];
    gHF = [[obj.CR_bodies(:,:,2) obj.body_obj{2}.position'];[zeros(1,3) 1]];
    gFT = [[obj.CR_bodies(:,:,3) obj.body_obj{3}.position'];[zeros(1,3) 1]];
    gTX = [[obj.CR_bodies(:,:,4) obj.body_obj{4}.position'];[zeros(1,3) 1]];

    femCOMw = gWH*gHF*[obj.body_obj{2}.com';1];
    tibCOMw = gWH*gHF*gFT*[obj.body_obj{3}.com';1];
    fotCOMw = gWH*gHF*gFT*gTX*[obj.body_obj{4}.com';1];
    COMw = [femCOMw(1:3),tibCOMw(1:3),fotCOMw(1:3)];

    gslfem = [[obj.Cabs_bodies(:,:,2) femCOMw(1:3)];[zeros(1,3) 1]];
    gsltib = [[obj.Cabs_bodies(:,:,3) tibCOMw(1:3)];[zeros(1,3) 1]];
    gslfot = [[obj.Cabs_bodies(:,:,4) fotCOMw(1:3)];[zeros(1,3) 1]];

    m1 = obj.body_obj{2}.mass/1000;
    m2 = obj.body_obj{3}.mass/1000;
    m3 = obj.body_obj{4}.mass/1000;

    % Computing the moments of inertia
    MOI = zeros(3,3,3);
    % Femur 
    r1 = 1.7751e-3; L1 = 38.6521e-3; MOI(1,1,1) = (1/12)*m1*L1^2; MOI(2,2,1) = (1/12)*m1*L1^2; MOI(3,3,1) = (1/2)*m1*r1^2;
    % Tibia
    r2 = 1.3745e-3; L2 = 41.46e-3; MOI(1,1,2) = sqrt(((1/12)*m2*L2^2)^2+((1/2)*m2*r2^2)^2); MOI(2,2,2) = sqrt(((1/12)*m2*L2^2)^2+((1/2)*m2*r2^2)^2); MOI(3,3,2) = (1/12)*m2*L2^2;
    % Tibia
    h = 2.846e-3; w = 9.486e-3; d = 20.19e-3; MOI(1,1,3) = (1/12)*m3*(d^2+h^2); MOI(2,2,3) = (1/12)*m3*(d^2+w^2); MOI(3,3,3) = (1/12)*m3*(h^2+w^2);

    gifem = [[eye(3).*[m1;m1;m1] zeros(3,3)];[zeros(3,3) eye(3).*[MOI(1,1,1);MOI(2,2,1);MOI(3,3,1)]]];
    gitib = [[eye(3).*[m2;m2;m2] zeros(3,3)];[zeros(3,3) eye(3).*[MOI(1,1,2);MOI(2,2,2);MOI(3,3,2)]]];
    gifot = [[eye(3).*[m3;m3;m3] zeros(3,3)];[zeros(3,3) eye(3).*[MOI(1,1,3);MOI(2,2,3);MOI(3,3,3)]]];

    Mprime(:,:,1) = inv(Adj_func(gslfem))'*gifem*inv(Adj_func(gslfem));
    Mprime(:,:,2) = inv(Adj_func(gsltib))'*gitib*inv(Adj_func(gsltib));
    Mprime(:,:,3) = inv(Adj_func(gslfot))'*gifot*inv(Adj_func(gslfot));

    % Make moment arm term
%     curvars = whos;
%     if ~any(contains({curvars.name},'moment_output'))
        for jointNum=1:3
            moment_output(:,:,jointNum) = compute_joint_moment_arms(obj,jointNum,1);
        end
        momentArmsHip = moment_output(:,:,1);
        momentArmsKnee = moment_output(:,:,2);
        momentArmsAnkle = moment_output(:,:,3);
%     end

    samplingInds = 2:length(obj.theta_motion);
    %samplingInds = 2:floor(length(obj.theta_motion)/10):length(obj.theta_motion);

    for timeInd = samplingInds
        for jj = 1:3
            w0 = obj.joint_obj{jj}.uuw_joint(:,timeInd);
            q = obj.joint_obj{jj}.sim_position_profile(timeInd,:)';
            Jac(:,jj) = [-cross(w0,q);w0];
        end
        for ii = 1:3
            switch ii
                case 1
                    e1 = exp_func(rotMat(obj.theta_motion(timeInd,1),Jac(4:6,1)./norm(Jac(4:6,1))),Jac(4:6,1),Jac(1:3,1),obj.theta_motion(timeInd,1));
                    temp0 = e1*gslfem;
                case 2
                    e2 = exp_func(rotMat(obj.theta_motion(timeInd,2),Jac(4:6,2)./norm(Jac(4:6,2))),Jac(4:6,2),Jac(1:3,2),obj.theta_motion(timeInd,2));
                    temp0 = e1*e2*gsltib;
                case 3
                    e3 = exp_func(rotMat(obj.theta_motion(timeInd,3),Jac(4:6,3)./norm(Jac(4:6,3))),Jac(4:6,3),Jac(1:3,3),obj.theta_motion(timeInd,3));
                    temp0 = e1*e2*e3*gslfot;
            end
            h(ii,timeInd) = dot(temp0(1:3,4),[0;1;0]);
        end
        
        J1 = [inv(Adj_func(e1*gslfem))*Jac(:,1) zeros(6,1) zeros(6,1)];
        J2 = [inv(Adj_func(e1*e2*gsltib))*Jac(:,1) inv(Adj_func(e2*gsltib))*Jac(:,2) zeros(6,1)];
        J3 = [inv(Adj_func(e1*e2*e3*gslfot))*Jac(:,1) inv(Adj_func(e2*e3*gslfot))*Jac(:,2) inv(Adj_func(e3*gslfot))*Jac(:,3)];

        Mout(:,:,timeInd) = J1'*gifem*J1+J2'*gitib*J2+J3'*gifot*J3;
        
        % Create the adjoint transformation matrix according to MLS 1994 p. 176 (4.27)
        for hh = 1:3
            for kk = 1:3
                if hh>kk
                    expMat = eye(4);
                    for tt = kk+1:hh
                        temp1  = exp_func(rotMat(obj.theta_motion(timeInd,tt),Jac(4:6,tt)),Jac(4:6,tt),Jac(1:3,tt),obj.theta_motion(timeInd,tt));
                        expMat = expMat*temp1; 
                    end
                    Acell{hh,kk} = inv(Adj_func(expMat));
                elseif hh==kk
                    Acell{hh,kk} = eye(6);
                elseif hh<kk
                    Acell{hh,kk} = zeros(6,6);
                end
            end
        end

        % Make Coriolis Term
        tDot = (obj.theta_motion(2,:)-obj.theta_motion(2-1,:))./obj.dt_motion;
        C(:,:,timeInd) = make_coriolis_term(Acell,Jac,Mprime,tDot);
        nummuscles = 1:38;
        p = [momentArmsHip(1:nummuscles,timeInd),momentArmsKnee(1:nummuscles,timeInd),momentArmsAnkle(1:nummuscles,timeInd)]';
        R = p/1000;
    end
    G = compute_body_torques(obj);
    
    function C = make_coriolis_term(A,Jac,M,theta_dot)
        % Make the Coriolis term according to MLS 1994 p. 176 (4.30)
        % Input: A cell(3,3): The adjoint Transformation according to MLS 1994 p. 176 (4.27)
        % Input: Jac double(6,3): spatial manipulator Jacobian. Each column is the joint twist
        % Input: M double(6,6): generalized inertia matrix following form MLS 1994 p.176 (4.28)
        % Input: theta_dot double(3,1): velocity of joint angle input
        C = zeros(3,3);
        for i = 1:3
            for j = 1:3
                temp = 0;
                for k = 1:3
                    dMdT_1 = 0;
                    for l = max(i,j):3
                        dMdT_1 = dMdT_1+bracket(A{k,i}*Jac(:,i),Jac(:,k))'*A{l,k}'*M(:,:,l)*A{l,j}*Jac(:,j)+Jac(:,i)'*A{l,i}'*M(:,:,l)*A{l,k}*bracket(A{k,j}*Jac(:,j),Jac(:,k));
                    end
                    dMdT_2 = 0;
                    for l = max(i,k):3
                        dMdT_2 = dMdT_2+bracket(A{j,i}*Jac(:,i),Jac(:,j))'*A{l,j}'*M(:,:,l)*A{l,k}*Jac(:,k)+Jac(:,i)'*A{l,i}'*M(:,:,l)*A{l,j}*bracket(A{j,k}*Jac(:,k),Jac(:,j));
                    end
                    dMdT_3 = 0;
                    for l = max(k,j):3
                        dMdT_3 = dMdT_3+bracket(A{i,k}*Jac(:,k),Jac(:,i))'*A{l,i}'*M(:,:,l)*A{l,j}*Jac(:,j)+Jac(:,k)'*A{l,k}'*M(:,:,l)*A{l,i}*bracket(A{i,j}*Jac(:,j),Jac(:,i));
                    end
                    temp = temp+(1/2)*(dMdT_1+dMdT_2-dMdT_3)*theta_dot(k);
                end
                C(i,j) = temp;
            end
        end
    end

    function Rout = rotMat(ang,w)
        if ang > 2*pi
            % Rudimentary test to determine if input is in degrees. Assumes that supplied angles will be given smaller than a full circle
            ang = ang*(pi/180);
        end
        Rout = [cos(ang)+w(1)^2*(1-cos(ang)) w(1)*w(2)*(1-cos(ang))-w(3)*sin(ang) w(1)*w(3)*(1-cos(ang))+w(2)*sin(ang); ...
                w(2)*w(1)*(1-cos(ang))+w(3)*sin(ang) cos(ang)+w(2)^2*(1-cos(ang)) w(2)*w(3)*(1-cos(ang))-w(1)*sin(ang);...
                w(3)*w(1)*(1-cos(ang))-w(2)*sin(ang) w(3)*w(2)*(1-cos(ang))+w(1)*sin(ang) cos(ang)+w(3)^2*(1-cos(ang))]; 
    end
end