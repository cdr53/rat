classdef FullLeg < matlab.mixin.SetGet
   properties
        proj_file
        original_text
        organism_name = 'Organism_1';
        joint_types; %type of joints in the leg, as read from the .asim file
        num_bodies; %number of bodies in the leg, as given by the user's list
        bodies; %names of the segments in the leg
        body_obj
        joints; %names of the joints in the leg
        joint_obj; %cell array of joint objects (for each joint in the leg)
        musc_inds;
        musc_obj = [];
        to_plot; %boolean of whether or not to plot graphs of the leg.
        vec_len; %length of the axis vector drawn in the kinematic maps.
        Cabs_joints; %Absolute orientation of the joints
        C_joints; %Rotation of the joints as per theta
        CR_bodies; %Relative rotation of the bodies with respect to the proximal one
        CR_joints; %Relative rotation of the joints with respect to the local frame
        C_pd;
        u_joint;
        cum_C_joints; %Cumulative rotation (product) of proximal joints
        Cabs_bodies; %Absolute rotation of the bodies
        pos_bodies; %position of the distal body in the proximal frame
        pos_joints; %location of the distal joint in the proximal frame
        pos_bodies_w;
        pos_bodies_w_profile;
        pos_joints_w;
        pos_attachments; %location of muscle attachments in their own frame
        pos_attachments_w;
        euler_angs_joints;
        euler_angs_bodies;
        theta_range; %min and max rotation for each joint
        velocity_range;
        p_st; %Position of the body relative to the foot on the ground.
        leg_attach_pt;
        foot_pos_residual;
        control_current; %Stimuli to joint position controllers necessary for each "config."
        body_twists; %Body frame transformations desired from a standing still posture. These correspond to "config."
        body_thetas;
        toe_pos_known = 0; %Boolean about whether the toe position is known or not.
        foot_vec; %vector that describes the toe wrt the most distal joint. Important for calculating forces from the foot.
        ankle_factor; %multiply distal max torque by 3 if it operates like an ankle.
        sensecasenum
        sensemuscname
    
        torque_motion;
        theta_motion;
        theta_motion_time;
        theta_dot_motion;
        dt_motion;
        
        gsyn_hip_knee_out;
        elo_hip_knee_out;
        ehi_hip_knee_out;
        
        sampling_vector;
        passive_tension;
   end
   methods
        function obj = FullLeg(sim_file,joints,bodies,joint_limits,joint_profile,passive_tension)
        %% Process the Simulation File
            obj.joint_types = cell(1,length(joints));
            obj.num_bodies = 0;
            i = 1;
            while i <= length(bodies) && ~isempty(bodies{i})
                obj.num_bodies = obj.num_bodies + 1;
                i = i + 1;
            end
            obj.theta_range = joint_limits;
            
%             %Read the directory provided by the user.
%             if ismac
%                 dir_file_division = max(strfind(proj_file,'/'));
%             else
%                 dir_file_division = max(strfind(proj_file,'\'));
%             end
%                 proj_folder = proj_file(1:dir_file_division-1);
%                 sim_files = dir(proj_folder);
%                 obj.proj_file = proj_file;
%             
%             %How many files are in that folder?
%                 len = length(sim_files);
%                 sim_file_str = [];
%             
%             %Pull out the simulation file
%                 for i=1:len
%                     name_str = sim_files(i).name;        
%                     if length(name_str) > 14 && strcmp(name_str(end-14:end),'Standalone.asim')
%                         %We've found the file we need
%                         if ismac
%                             sim_file_str = [proj_folder,'/',name_str];
%                         else
%                             sim_file_str = [proj_folder,'\',name_str];
%                         end
%                         break
%                     end
%                 end
            
            %Load the text of the simulation file
                try obj.original_text = importdata(sim_file);
                catch
                    disp('No simulation file exists. Open AnimatLab and export a standalone simulation.')
                    keyboard
                end
        %% Initialize Body and Joint Rotations for Inertial and Local Frame
            %Orientation of the joint axis, in the inertial frame
            obj.Cabs_joints = zeros(3,3,obj.num_bodies);
            obj.Cabs_joints(:,:,1) = eye(3);

            %Rotation of the joint axis, in the local frame
            obj.C_joints = zeros(3,3,obj.num_bodies);
            obj.C_joints(:,:,1) = eye(3);

            %Rotation of each body relative to the previous
            obj.CR_bodies = zeros(3,3,obj.num_bodies); 
            obj.CR_joints = zeros(3,3,obj.num_bodies);

            %Rotation of each body in the inertial frame
            obj.Cabs_bodies = zeros(3,3,obj.num_bodies);

            %Cumulative body rotation
            cum_body_rot = eye(3);
        %% Initialize Body and Joint Positions in the Local Reference Frames
            %Position of the joints and bodies, in their local frames.
            obj.pos_joints = zeros(3,obj.num_bodies);
            obj.pos_bodies = zeros(3,obj.num_bodies);
            obj.pos_attachments = cell(obj.num_bodies,1); %one for each joint
            euler_angs_bodies = zeros(3,obj.num_bodies);
            obj.euler_angs_joints = zeros(3,obj.num_bodies);
            
            %Initialize other object values
            for ii = 1:length(bodies)
                obj.body_obj{ii,1} = Body(bodies{ii});
            end
            obj.bodies = bodies;
            
            joints = joints(~cellfun(@isempty,joints));
            obj.joints = joints;
            for ii = 1:length(joints)
                obj.joint_obj{ii,1} = JointSyn(joints{ii});
            end         
        %% Fill in Body and Joint Details
            i = 1;
            ot = obj.original_text;
%             for ii = 1:(obj.num_bodies-1)
%                 obj.joint_obj{ii,1} = JointSyn();
%             end
            
            while i <= obj.num_bodies
                %Find the body of interest
                if ~isempty(obj.bodies{i})
                    next_body_ind = find(contains(obj.original_text,['<Name>',obj.bodies{i},'</Name>']),1,'first');
                    chain_lower_limit = next_body_ind;

                    if isempty(chain_lower_limit)
                        disp('An improper body or joint was entered, or perhaps out of order.')
                    end

                    body_params = {'<ID>';'<Position ';'<Rotation ';'<Density>';'<Mass>';'<COM ';'<MeshFile>';'<ConvexMeshFile>';'<Scale '};
                    
                    for ii = 1:length(body_params)
                        pTemp = lower(erase(body_params{ii},{'<','>',' '}));
                        pInd = find(contains(ot(chain_lower_limit:end),body_params{ii}),1,'first') + chain_lower_limit - 1;
                        quoteLocs = strfind(ot{pInd},'"');
                        if ~isempty(quoteLocs)
                            sTemp = ot{pInd};
                            qTemp = reshape(quoteLocs,[2 3])';
                            parDbl = zeros(1,length(quoteLocs)/2);
                            for jj = 1:size(qTemp,1)
                                parDbl(jj) = str2double(sTemp(qTemp(jj,1)+1:qTemp(jj,2)-1));
                            end
                            obj.body_obj{i}.(pTemp) = parDbl;
                        else
                            sTemp = extractBetween(string(ot{pInd}),'>','<');
                            if isnan(str2double(sTemp))
                                obj.body_obj{i}.(pTemp) = char(sTemp);
                            else
                                obj.body_obj{i}.(pTemp) = str2double(sTemp);
                            end
                        end
                    end
                    
                    %Find the string that lists its rotation
                    body_rot_ind = find(contains(ot(chain_lower_limit:end),'<Rotation'),1,'first') + chain_lower_limit - 1;
                    body_rot_str = ot(body_rot_ind);

                    %Read that body's rotation angles and construct a rotation matrix.
%                     quote_locs = cell2mat(strfind(body_rot_str,'"'));
%                     for j=1:length(quote_locs)/2
%                         euler_angs_bodies(j,i) = str2double(body_rot_str{1}(quote_locs(2*j-1)+1:quote_locs(2*j)-1));
%                     end
                    euler_angs_bodies(:,i) = obj.body_obj{i}.rotation';
                    %obj.euler_angs_bodies(:,i) = euler_angs_bodies(:,i);
                    %Compute the rotation matrix for each axis.
                    rotx_b = [1,0,0;0,cos(euler_angs_bodies(1,i)),-sin(euler_angs_bodies(1,i));0,sin(euler_angs_bodies(1,i)),cos(euler_angs_bodies(1,i))];
                    roty_b = [cos(euler_angs_bodies(2,i)),0,sin(euler_angs_bodies(2,i));0,1,0;-sin(euler_angs_bodies(2,i)),0,cos(euler_angs_bodies(2,i))];
                    rotz_b = [cos(euler_angs_bodies(3,i)),-sin(euler_angs_bodies(3,i)),0;sin(euler_angs_bodies(3,i)),cos(euler_angs_bodies(3,i)),0;0,0,1];

                    %Record the relative rotation of this frame with regard to the
                    %previous, as well as the absolute rotation with respect to the ground.
                    obj.CR_bodies(:,:,i) = rotx_b*roty_b*rotz_b;
                    cum_body_rot = cum_body_rot * obj.CR_bodies(:,:,i);
                    obj.Cabs_bodies(:,:,i) = cum_body_rot;

%                     %Find the position of the body
%                     body_pos_found = contains(ot(chain_lower_limit:end),'<Position');
%                     body_pos_ind = find(body_pos_found,1,'first') + chain_lower_limit - 1;
%                     body_pos_str = ot(body_pos_ind);
% 
%                     quote_locs = cell2mat(strfind(body_pos_str,'"'));
%                     for j=1:length(quote_locs)/2
%                         obj.pos_bodies(j,i) = str2double(body_pos_str{1}(quote_locs(2*j-1)+1:quote_locs(2*j)-1));
%                     end

                    obj.pos_bodies(:,i) = obj.body_obj{i}.position';
                           
                    % Now for the joint
                    if i > 1
                        %Move up the minimum line value now that that body is done.
                        joint_found = contains(ot,['<Name>',obj.joints{i-1},'</Name>']);
                        %obj.joint_obj{i-1}.name = obj.joints{i};
                        %next_joint_ind = find(~cellfun(@isempty,joint_found),1,'first');
                        %chain_lower_limit = next_joint_ind;
                        obj.joint_obj{i-1}.index = find(joint_found,1,'first');
                        chain_lower_limit = obj.joint_obj{i-1}.index;

                        %Find the orientation of that joint
                        joint_rot_found = contains(ot(chain_lower_limit:end),'<Rotation');
                        joint_rot_ind = find(joint_rot_found,1,'first') + chain_lower_limit - 1;
                        joint_rot_str = ot(joint_rot_ind);

                        quote_locs = cell2mat(strfind(joint_rot_str,'"'));
                        for j=1:length(quote_locs)/2
                            %obj.euler_angs_joints(j,i) = str2double(joint_rot_str{1}(quote_locs(2*j-1)+1:quote_locs(2*j)-1));
                            obj.joint_obj{i-1}.euler_angs(j,1) = str2double(joint_rot_str{1}(quote_locs(2*j-1)+1:quote_locs(2*j)-1));
                        end
                        
                        %Find the limits of that joint
                        joint_lims_found = contains(ot(chain_lower_limit:end),'<LimitPos>');
                        joint_lims_ind = find(joint_lims_found,1,'first') + chain_lower_limit - 1;
                        joint_lims_str1 = cell2mat(ot(joint_lims_ind));
                        joint_lims_str2 = cell2mat(ot(joint_lims_ind+8));

                        obj.joint_obj{i-1}.limits(1,1) = str2double(joint_lims_str2(11:length(joint_lims_str2)-11));
                        obj.joint_obj{i-1}.limits(2,1) = str2double(joint_lims_str1(11:length(joint_lims_str1)-11));
                        
                        %Compute the rotation of the joint's axis within the local frame.
                        rotx_j = [1,0,0;0,cos(obj.euler_angs_joints(1,i)),-sin(obj.euler_angs_joints(1,i));0,sin(obj.euler_angs_joints(1,i)),cos(obj.euler_angs_joints(1,i))];
                        roty_j = [cos(obj.euler_angs_joints(2,i)),0,sin(obj.euler_angs_joints(2,i));0,1,0;-sin(obj.euler_angs_joints(2,i)),0,cos(obj.euler_angs_joints(2,i))];
                        rotz_j = [cos(obj.euler_angs_joints(3,i)),-sin(obj.euler_angs_joints(3,i)),0;sin(obj.euler_angs_joints(3,i)),cos(obj.euler_angs_joints(3,i)),0;0,0,1];

                        %Quick side track: find what kind of joint this is. If it is
                        %prismatic, it does not rotate anything. If it is a hinge, then we
                        %need to account for that rotation.
                        joint_type_found = contains(ot(chain_lower_limit:end),'<PartType');
                        joint_type_ind = find(joint_type_found,1,'first') + chain_lower_limit - 1;
                        joint_type_str = ot(joint_type_ind);

                        %pull out the joint type
                        type_begin = strfind(joint_type_str,'.');
                        type_end = strfind(joint_type_str,'<');
                        %obj.joint_types{i} = joint_type_str{:}(type_begin{:}(end)+1:type_end{:}(end)-1);
                        obj.joint_obj{i-1}.type = joint_type_str{:}(type_begin{:}(end)+1:type_end{:}(end)-1);

                        %Calculate the orientation of the axis in space, Cabs_joints, and
                        %the rotation of the joint, C_joints
                        obj.Cabs_joints(:,:,i) = obj.Cabs_bodies(:,:,i)*rotx_j*roty_j*rotz_j;
                        obj.CR_joints(:,:,i) = rotx_j*roty_j*rotz_j;
                        obj.C_joints(:,:,i) = eye(3);

                        %Find the location of the joint within the frame
                        joint_pos_found = contains(ot(chain_lower_limit:end),'<Position');
                        joint_pos_ind = find(joint_pos_found,1,'first') + chain_lower_limit - 1;
                        joint_pos_str = ot(joint_pos_ind);

                        quote_locs = cell2mat(strfind(joint_pos_str,'"'));
                        for j=1:length(quote_locs)/2
                            %obj.pos_joints(j,i) = str2double(joint_pos_str{1}(quote_locs(2*j-1)+1:quote_locs(2*j)-1));
                            obj.joint_obj{i-1}.init_pos(j,1) = str2double(joint_pos_str{1}(quote_locs(2*j-1)+1:quote_locs(2*j)-1));
                        end

                        %Move up the minimum line value now that that body is done.
                        try
                            next_joint_ind = find(contains(ot,['<Name>',obj.bodies{i+1},'</Name>']),1,'first');
                            chain_lower_limit = next_joint_ind;
                        catch
                            %disp('End of joint chain reached.')
                        end
                    end
                end
                i = i + 1;
            end
            
            %%Store the world position for each body and joint
            obj.pos_bodies_w = zeros(3,obj.num_bodies);
            obj.pos_joints_w = zeros(3,obj.num_bodies-1);

            for i=2:obj.num_bodies
                %pos_bodies should be counted from 2 to end. Each column is that body's
                %position relative to the first body (which is 0s for the first).
                %Second body's position is given in the frame of the first.

                obj.pos_bodies_w(:,i) = obj.pos_bodies_w(:,i-1) + obj.Cabs_bodies(:,:,i-1)*obj.pos_bodies(:,i);
                %obj.pos_joints_w(:,i-1) = obj.pos_bodies_w(:,i) + obj.CR_bodies(:,:,i)*obj.pos_joints(:,i);
                obj.joint_obj{i-1}.init_pos_w = obj.pos_bodies_w(:,i) + obj.Cabs_bodies(:,:,i)*obj.joint_obj{i-1}.init_pos;
            end
            
            initToeW = [-.014793 -.084157 .020988];
            for i = 2:obj.num_bodies
                if i == obj.num_bodies
                    obj.body_obj{i}.length = norm(initToeW-obj.joint_obj{i-1}.init_pos_w);
                else
                    obj.body_obj{i}.length = norm(obj.joint_obj{i}.init_pos_w-obj.joint_obj{i-1}.init_pos_w);
                end
            end
        %% Find Muscles, Attachments, and Their Associated Bodies/Joints
            tstart = tic;
            musc_found = strfind(obj.original_text,'<Type>LinearHillMuscle</Type>');
            musc_inds = find(~cellfun(@isempty,musc_found));
            musc_name_inds = musc_inds-2;

            for i = 1:length(musc_inds)
                obj.musc_obj{i,1} = Muscle();
            end
            
            name_true = 1;
            for i=1:length(obj.joint_obj)-1
                if ~strcmp(obj.joints{i}(1:2),obj.joints{i+1}(1:2))
                    name_true = 0;
                end
            end
            
            if name_true
                leg_name = obj.joints{1}(1:2);
            else
                leg_name = input('What is the prefix of objects belonging to this leg?\nA cell array of prefixes may be entered. ');
            end
            
            if ischar(leg_name)
                musc_for_this_leg = strfind(obj.original_text(musc_name_inds),['<Name>',leg_name]);
            elseif iscell(leg_name)
                musc_for_this_leg = cell(length(musc_name_inds),1);
                for i=1:length(leg_name)
                    temp_musc_for_this_leg = strfind(obj.original_text(musc_name_inds),['<Name>',leg_name{i}]);
                    for j=1:length(temp_musc_for_this_leg)
                        if ~isempty(temp_musc_for_this_leg{j})
                            musc_for_this_leg{j} = 1;
                        end
                    end
                end
            end
            musc_for_this_leg_inds = cellfun(@isempty,musc_for_this_leg) == 0;
            
            %These are the indices of the names of muscles.
            obj.musc_inds = musc_name_inds(musc_for_this_leg_inds);
            attach_to_find = cell(length(musc_inds),1);
            
            %Now that we know where the muscles are saved, we need to
            %extract all of the attachment IDs, find their names (to figure
            %out which body they belong to), and save their locations to
            %create joint objects.
            for i=1:length(musc_inds)
                attachment_start = contains(obj.original_text(musc_inds(i):end),'<Attachments>');
                attachment_start_ind = find(attachment_start,1,'first') + musc_inds(i) - 1;
                attachment_end = contains(obj.original_text(musc_inds(i):end),'</Attachments>');
                attachment_end_ind = find(attachment_end,1,'first') + musc_inds(i) - 1;

                attach_to_find{i} = obj.original_text(attachment_start_ind + 1:attachment_end_ind - 1);
                %remove the word "attach" from the strings
                for j=1:length(attach_to_find{i})
                    attach_to_find{i}{j} = strrep(attach_to_find{i}{j},'Attach','');
                end                
            end
            
            attach_names = cell(size(attach_to_find));
            attach_names_str = cell(size(attach_to_find));
            attach_locs = cell(size(attach_to_find));
            %find indices of the names of these attachments.
            for i=1:length(musc_inds)
                for j=1:length(attach_to_find{i})
                    %save the names
                    id_loc = contains(obj.original_text,attach_to_find{i}{j});
                    id_ind = find(id_loc,1,'first');
                    attach_names{i}{j} = id_ind - 1;
                    attach_names_str{i}{j} = obj.original_text{attach_names{i}{j}};
                     
                    %Find the position of that attachment. This mapping is
                    %identical to the names.
                    %NEW
                    attach_pos_found = contains(obj.original_text(id_ind:end),'<Position');
                    attach_pos_ind = find(attach_pos_found,1,'first') + id_ind - 1;
                    attach_pos_str = obj.original_text(attach_pos_ind);

                    quote_locs = cell2mat(strfind(attach_pos_str,'"'));
                    for k=1:length(quote_locs)/2
                        attach_locs{i}{j}(k,1) = str2double(attach_pos_str{1}(quote_locs(2*k-1)+1:quote_locs(2*k)-1));
                    end
                end
            end
            %We will need to remove redundant attachments. If multiple muscles use the same
            %attachment, it will show up in our list twice. Therefore we
            %will start a list used_indices, and every time we add an
            %attachment's position to our record, we add the index of its
            %name to this list. Attachments whose indices already appear on
            %the list will be ignored.
            used_indices = [];
            
            %Tally up the total number of attachments on each body so that
            %we can set the size for obj.pos_attachments
            %Cells don't allow you to increment the size
            pelvis_count = 0;
            femur_count = 0;
            tibia_count = 0;
            foot_count = 0;
            for j=1:length(attach_names)
                for k=1:length(attach_names{j})
                    if isempty(find(used_indices == attach_names{j}{k},1,'first'))
                        temp_name_str = char(obj.original_text(attach_names{j}{k}));
                        if contains(temp_name_str,' Pelvis ')
                            pelvis_count = pelvis_count + 1;
                        elseif contains(temp_name_str,' Femur ')
                            femur_count = femur_count + 1;
                        elseif contains(temp_name_str,' Tibia ')
                            %spaces around Tibia important bc muscle
                            %'Tibialis Anterior' contains Tibia
                            tibia_count = tibia_count + 1;
                        elseif contains(temp_name_str,' Foot ')
                            foot_count = foot_count + 1;
                        else
                            keyboard
                            error('Attachment not labeled with a body part')
                        end
                        used_indices = [used_indices,attach_names{j}{k}];
                    end
                end
            end
            used_indices = used_indices';
            saved_indices = used_indices';
            %Set the size of obj.pos_attachments for each body
            obj.pos_attachments{2} = cell(pelvis_count,3);
            obj.pos_attachments{3} = cell(femur_count,3);
            obj.pos_attachments{4} = cell(tibia_count,3);
            obj.pos_attachments{5} = cell(foot_count,3);
            
%             %For sensitivity analysis of moment arms, move the attachments points of selected muscles
%             obj.sensemuscname = 'TA';
%             obj.sensecasenum = input('Enter case number: ');
%             switch obj.sensemuscname
%                 case 'BFA'
%                     muscnum = 33;
%                     switch obj.sensecasenum
%                         case 1
%                             attach_locs{muscnum,1}{1,3} = attach_locs{muscnum,1}{1,3};
%                         case 2
%                             attach_locs{muscnum,1}{1,3} = attach_locs{muscnum,1}{1,3}+[.001;0;0];
%                         case 3
%                             attach_locs{muscnum,1}{1,3} = attach_locs{muscnum,1}{1,3}-[.001;0;0];
%                         case 4
%                             attach_locs{muscnum,1}{1,4} = attach_locs{muscnum,1}{1,4}+[0;.001;0];
%                         case 5           
%                             attach_locs{muscnum,1}{1,4} = attach_locs{muscnum,1}{1,4}-[0;.001;0];
%                     end
%                 case 'Pect'
%                     muscnum = 5;
%                     switch obj.sensecasenum
%                         case 1
%                             attach_locs{muscnum,1}{1,1} = attach_locs{muscnum,1}{1,1};
%                         case 2
%                             attach_locs{muscnum,1}{1,1} = attach_locs{muscnum,1}{1,1}+[.001;0;0];
%                         case 3
%                             attach_locs{muscnum,1}{1,1} = attach_locs{muscnum,1}{1,1}-[.001;0;0];
%                         case 4
%                             attach_locs{muscnum,1}{1,2} = attach_locs{muscnum,1}{1,2}+[0;0;.001];
%                         case 5           
%                             attach_locs{muscnum,1}{1,2} = attach_locs{muscnum,1}{1,2}-[0;0;.001];
%                     end
%                 case 'SM'
%                     muscnum = 31;
%                     switch obj.sensecasenum
%                         case 1
%                             attach_locs{muscnum,1}{1,1} = attach_locs{muscnum,1}{1,1};
%                         case 2
%                             attach_locs{muscnum,1}{1,1} = attach_locs{muscnum,1}{1,1}+[0;0;.001];
%                         case 3
%                             attach_locs{muscnum,1}{1,1} = attach_locs{muscnum,1}{1,1}-[0;0;.001];
%                         case 4
%                             attach_locs{muscnum,1}{1,2} = attach_locs{muscnum,1}{1,2}+[0;-.001*sin(30*(pi/180));-.001*cos(30*(pi/180))];
%                         case 5           
%                             attach_locs{muscnum,1}{1,2} = attach_locs{muscnum,1}{1,2}-[0;-.001*sin(30*(pi/180));-.001*cos(30*(pi/180))];
%                     end
%                 case 'VI'
%                     muscnum = 18;
%                     switch obj.sensecasenum
%                         case 1
%                             attach_locs{muscnum,1}{1,1} = attach_locs{muscnum,1}{1,1};
%                         case 2
%                             attach_locs{muscnum,1}{1,2} = attach_locs{muscnum,1}{1,2}+[0;0;.001];
%                         case 3
%                             attach_locs{muscnum,1}{1,2} = attach_locs{muscnum,1}{1,2}-[0;0;.001];
%                         case 4
%                             attach_locs{muscnum,1}{1,3} = attach_locs{muscnum,1}{1,3}+[0;.001;0];
%                         case 5           
%                             attach_locs{muscnum,1}{1,3} = attach_locs{muscnum,1}{1,3}-[0;.001;0];
%                     end
%                 case 'MG'
%                     muscnum = 17;
%                     switch obj.sensecasenum
%                         case 1
%                             attach_locs{muscnum,1}{1,1} = attach_locs{muscnum,1}{1,1};
%                         case 2
%                             attach_locs{muscnum,1}{1,2} = attach_locs{muscnum,1}{1,2}+[0;.001;0];
%                         case 3
%                             attach_locs{muscnum,1}{1,2} = attach_locs{muscnum,1}{1,2}-[0;.001;0];
%                         case 4
%                             attach_locs{muscnum,1}{1,3} = attach_locs{muscnum,1}{1,3}+[0;.001;0];
%                         case 5           
%                             attach_locs{muscnum,1}{1,3} = attach_locs{muscnum,1}{1,3}-[0;.001;0];
%                     end
%                 case 'TA'
%                     muscnum = 22;
%                     switch obj.sensecasenum
%                         case 1
%                             attach_locs{muscnum,1}{1,1} = attach_locs{muscnum,1}{1,1};
%                         case 2
%                             attach_locs{muscnum,1}{1,3} = attach_locs{muscnum,1}{1,3}+[0;.001;0];
%                         case 3
%                             attach_locs{muscnum,1}{1,3} = attach_locs{muscnum,1}{1,3}-[0;.001;0];
%                         case 4
%                             attach_locs{muscnum,1}{1,4} = attach_locs{muscnum,1}{1,4}+[.001;0;0];
%                         case 5           
%                             attach_locs{muscnum,1}{1,4} = attach_locs{muscnum,1}{1,4}-[.001;0;0];
%                     end
%             end
            
            %Store information about attachment points for each body in
            %obj.pos_attachments. Resultant cell array holds position
            %information for XYZ and name strings of every unique
            %attachment point on each body
            for j=1:length(attach_names)
                obj.musc_obj{j}.muscle_index = musc_name_inds(j);
                for k=1:length(attach_names{j})
                    if isequal(size(attach_locs{j}{k}),[3,1])
                        temp_name_str = char(obj.original_text(attach_names{j}{k}));
                        temp_name_str = strrep(temp_name_str,'<Name>','');
                        temp_name_str = strrep(temp_name_str,'</Name>','');

                        if contains(temp_name_str,' Pelvis ')
                            body = 2;
                        elseif contains(temp_name_str,' Femur ')
                            body = 3;
                        elseif contains(temp_name_str,' Tibia ')
                            body = 4;
                        elseif contains(temp_name_str,' Foot ')
                            body = 5;
                        else
                            keyboard
                            error('Attachment not labeled with a body part')
                        end
                        %Following if statement necessary to save
                        %information only for non-redundant attachment
                        %points
                        obj.musc_obj{j,1}.pos_attachments{k,1} = attach_locs{j}{k};
                        num_attach = strcat(num2str(size(obj.musc_obj{j,1}.pos_attachments{k,1},2)),') ');
                        obj.musc_obj{j,1}.pos_attachments{k,2} = [num_attach,obj.original_text{attach_names{j}{k}}(7:end-7),'\n'];
                        obj.musc_obj{j,1}.pos_attachments{k,3} = body-1;
                        obj.musc_obj{j,1}.pos_attachments{k,4} = [];
                        if ~isempty(find(used_indices == attach_names{j}{k},1,'first'))
                            used_indices(find(used_indices == attach_names{j}{k},1,'first')) = 0;
                            jj = sum(~cellfun(@isempty,obj.pos_attachments{body,1}(:,1)))+1;
                            obj.pos_attachments{body,1}{jj,1} = [obj.pos_attachments{body,1}{jj,1},attach_locs{j}{k}];
                            num_attach = strcat(num2str(size(obj.pos_attachments{body,1}{jj,1},2)),') ');
                            obj.pos_attachments{body,1}{jj,2} = [obj.pos_attachments{body,1}{jj,2},num_attach,obj.original_text{attach_names{j}{k}}(7:end-7),'\n'];
                        else
                            %skip this reused attachment
                        end
                    end  
                end
            end
            
            %%Store the world position for each attachment
            obj.pos_attachments_w = obj.pos_attachments;

            for i=2:obj.num_bodies+1
                for j = 1:size(obj.pos_attachments{i,1},1)
                    obj.pos_attachments_w{i,1}{j,1} = obj.pos_bodies_w(:,i-1) + obj.Cabs_bodies(:,:,i-1)*obj.pos_attachments_w{i,1}{j,1};
                end
            end
            
            telapsed = toc(tstart);
            %disp(['Attachments Stored.',' (',num2str(telapsed),'s)'])
%             for i=1:length(obj.bodies)-1
%                 %Hinge joint axis in the previous body's frame
%                 obj.u_joint{i,1} = obj.CR_bodies(:,:,i+1)*obj.three_axis_rotation(obj.euler_angs_joints(:,i+1))*[-1;0;0];
%             end
            
            %obj.u_joint = obj.CR_bodies(:,:,2)*obj.three_axis_rotation(obj.joint_obj{2,1}.euler_angs)*[-1;0;0];            
        %% Store joint properties
            tstart = tic;
            %%Neutral position. Enable if you want to have a contant profile where all joints are at 90 degrees
                if 0
                    joint_profile(:,2) = -.1396;
                    joint_profile(:,3) = 0;
                    joint_profile(:,4) = -.4538;
                end
            [jointMotion,jointMotionDot] = store_joint_rotmat_profile(obj,joint_profile);
            store_torque_and_kinematic_data(obj,jointMotion,jointMotionDot,jointMotion(10,1)-jointMotion(9,1));
            store_sampling_vector(obj);
            store_jointbodyw_position_profiles(obj,jointMotion);
            store_joint_params(obj);
            telapsed = toc(tstart);
            %disp(['Joint Properties Stored.',' (',num2str(telapsed),'s)'])
            clear tstart telapsed
        %% Store muscle properties  
            tstart = tic;
            store_animatlab_params(obj);
            store_johnson_params(obj);
            store_muscle_profiles(obj);
            store_input_muscle_passive_tension(obj,passive_tension);
            telapsed = toc(tstart);
            %disp(['Muscle Properties Stored.',' (',num2str(telapsed),'s)'])
            clear tstart telapsed
        end
        %% Function: Store the joint rotation matrices in the joint objects
        function [jointMotion,jointMotionDot] = store_joint_rotmat_profile(obj,jointMotion)     
            %load('joint_data_fullLeg')
            %load('joint_angles_maxmin.mat')
%              jointMotion = [];
%              jointMotionDot = [];
            obj.dt_motion = jointMotion(10,1)-jointMotion(9,1);
            t = jointMotion(:,1);
%             t = t(t<1);
            jointMotionDot = [jointMotion(1:length(t)-1,1),diff(jointMotion(1:length(t),2:4))/obj.dt_motion];
            jointMotion = jointMotion(1:length(t),:);
            %load('joint_angles_alexsim.mat','jointMotion','jointMotionDot')
%             load('joint_angles_realtime.mat','jointMotion','jointMotionDot')
%             load('joint_angles_realtime_20190314.mat','jointMotion','jointMotionDot')
            %load('joint_data_fullLeg_trimmed')
            %% For Moving a single joint at a time
            onejointonly = 0;
            if onejointonly
                load('joint_angles_maxmin.mat')
                fullrangewalking = 1;
                standardwalking = 0;
                hiponly = 0;
                kneeonly = 0;
                ankleonly = 1;
                if hiponly+kneeonly+ankleonly > 1
                    error('Only one active at a time')
                end
                if standardwalking
                    if hiponly == 1
                        joint_profile = jointMotion(200:end,1:4);
                        joint_profile(:,3) = max(joint_profile(:,3))-min(joint_profile(:,3));
                        joint_profile(:,4) = max(joint_profile(:,4))-min(joint_profile(:,4));
                        jointMotionDot = jointMotionDot(200:end,1:4);
                        jointMotionDot(:,3) = max(jointMotionDot(:,3))-min(jointMotionDot(:,3));
                        jointMotionDot(:,4) = max(jointMotionDot(:,4))-min(jointMotionDot(:,4));
                    elseif kneeonly == 1
                        joint_profile = jointMotion(200:end,1:4);
                        joint_profile(:,2) = max(joint_profile(:,2))-min(joint_profile(:,2));
                        joint_profile(:,4) = max(joint_profile(:,4))-min(joint_profile(:,4));
                        jointMotionDot = jointMotionDot(200:end,1:4);
                        jointMotionDot(:,2) = max(jointMotionDot(:,2))-min(jointMotionDot(:,2));
                        jointMotionDot(:,4) = max(jointMotionDot(:,4))-min(jointMotionDot(:,4));
                    elseif ankleonly == 1
                        joint_profile = jointMotion(200:end,1:4);
                        joint_profile(:,2) = max(joint_profile(:,2))-min(joint_profile(:,2));
                        joint_profile(:,3) = max(joint_profile(:,3))-min(joint_profile(:,3));
                        jointMotionDot = jointMotionDot(200:end,1:4);
                        jointMotionDot(:,2) = max(jointMotionDot(:,2))-min(jointMotionDot(:,2));
                        jointMotionDot(:,3) = max(jointMotionDot(:,3))-min(jointMotionDot(:,3));
                    end
                elseif fullrangewalking
                    limits = (pi/180)*[30.00 -50.00;...
                        -63.42 36.58;...
                        29.95 -70.05];
                    ranges = limits(:,1)-limits(:,2);
                    offsets = ranges/2-limits(:,1);
                    joint_profile = zeros(size(jointMotion));
                    joint_profile(:,1) = jointMotion(:,1);
                    joint_profile(:,2) = joint_profile(:,2)+(ranges(1)/2*sin(.1*2*pi*jointMotion(:,1))-offsets(1));
                    joint_profile(:,3) = joint_profile(:,3)+(ranges(2)/2*sin(.1*2*pi*jointMotion(:,1))-offsets(2));
                    joint_profile(:,4) = joint_profile(:,4)+(ranges(3)/2*sin(.1*2*pi*jointMotion(:,1))-offsets(3));
                    jointMotionDot = jointMotionDot(200:end,1:4);
                    if hiponly == 1
                        jointMotionDot = jointMotionDot(200:end,1:4);
%                         joint_profile(:,3) = max(joint_profile(:,3))-min(joint_profile(:,3));
%                         joint_profile(:,4) = max(joint_profile(:,4))-min(joint_profile(:,4));
                        %Set the knee and ankle to the C/J zero angle as defined in Animatlab
                        joint_profile(:,3) = (pi/180)*81.578;
                        joint_profile(:,4) = (pi/180)*-20.053;
                        jointMotionDot(:,3) = max(jointMotionDot(:,3))-min(jointMotionDot(:,3));
                        jointMotionDot(:,4) = max(jointMotionDot(:,4))-min(jointMotionDot(:,4));
                    elseif kneeonly == 1
                        jointMotionDot = jointMotionDot(200:end,1:4);
%                         joint_profile(:,2) = max(joint_profile(:,2))-min(joint_profile(:,2));
%                         joint_profile(:,4) = max(joint_profile(:,4))-min(joint_profile(:,4));
                        joint_profile(:,2) = 0;
                        joint_profile(:,4) =(pi/180)*81.578;
                        jointMotionDot(:,2) = max(jointMotionDot(:,2))-min(jointMotionDot(:,2));
                        jointMotionDot(:,4) = max(jointMotionDot(:,4))-min(jointMotionDot(:,4));
                    elseif ankleonly == 1
                        jointMotionDot = jointMotionDot(200:end,1:4);
%                         joint_profile(:,2) = max(joint_profile(:,2))-min(joint_profile(:,2));
%                         joint_profile(:,3) = max(joint_profile(:,3))-min(joint_profile(:,3));
                        joint_profile(:,2) = 0;
                        joint_profile(:,3) = (pi/180)*81.578;
                        jointMotionDot(:,2) = max(jointMotionDot(:,2))-min(jointMotionDot(:,2));
                        jointMotionDot(:,3) = max(jointMotionDot(:,3))-min(jointMotionDot(:,3));
                    end
                end
            end
            %% Continuing on once joint profile has been established
            for i = 1:obj.num_bodies-1
                %uu_joint(:,i) = obj.CR_bodies(:,:,i+1)*obj.three_axis_rotation(obj.euler_angs_joints(:,i+1))*[-1;0;0];
                %These are defined in the distal body's coordinate system (femur for the hip, etc).
                %If you want uu_joint in world coords, you need to pre-multiply by each previous body's obj.CR_bodies(:,:,i)
                %uu_joint == Ext/Flx
                %uu_joint2 == ExR/InR
                %uu_joint3 == Add/Abd
                axismatrix = -obj.CR_bodies(:,:,i+1)*obj.three_axis_rotation(obj.joint_obj{i}.euler_angs);
                     obj.joint_obj{i}.uu_joint  = axismatrix(:,1);
                     %Need these?
                    obj.joint_obj{i}.uu_joint2 = axismatrix(:,2);
                    obj.joint_obj{i}.uu_joint3 = axismatrix(:,3);
                    obj.joint_obj{i}.init_rot = jointMotion(1,i+1);
            end
            %This is the same thing as the above loops. The euler angs bodies term is the same as CR_bodies. This is how Alex solved for the u_joint axes
            %u_joint = obj.three_axis_rotation(obj.euler_angs_bodies(:,2))*obj.three_axis_rotation(obj.joint_obj{1}.euler_angs)*[-1;0;0];
            a = zeros(3,3,size(jointMotion,1)); 
            b = zeros(3,3,size(jointMotion,1)); 
            c = zeros(3,3,size(jointMotion,1)); 
            hip_uu2 = zeros(3,size(jointMotion,1));
            hip_uu3 = hip_uu2;
            knee_uu2 = hip_uu2+obj.joint_obj{2}.uu_joint2;
            knee_uu3 = hip_uu2+obj.joint_obj{2}.uu_joint3;
            ankle_uu2 = hip_uu2+obj.joint_obj{3}.uu_joint2;
            ankle_uu3 = hip_uu2+obj.joint_obj{3}.uu_joint3;
            
            parfor i=1:size(jointMotion,1)
                a(:,:,i) = axis_angle_rotation(obj,jointMotion(i,2),obj.joint_obj{1}.uu_joint);
                b(:,:,i) = axis_angle_rotation(obj,jointMotion(i,3),obj.joint_obj{2}.uu_joint);
                c(:,:,i) = axis_angle_rotation(obj,jointMotion(i,4),obj.joint_obj{3}.uu_joint);
                knee  = -1*b(:,:,i)*obj.CR_bodies(:,:,3)*obj.three_axis_rotation(obj.joint_obj{2}.euler_angs);
                ankle = -1*obj.CR_bodies(:,:,4)*obj.three_axis_rotation(obj.joint_obj{3}.euler_angs);
                knee_uu2(:,i)  = knee(:,2);
                knee_uu3(:,i)  = knee(:,3);
                ankle_uu2(:,i) = ankle(:,2);
                ankle_uu3(:,i) = ankle(:,3);
            end
%             hip = -1*a(:,:,1)*obj.CR_bodies(:,:,2)*obj.three_axis_rotation(obj.joint_obj{1}.euler_angs);
            hip = a(:,:,1)*obj.CR_bodies(:,:,2)*obj.three_axis_rotation(obj.joint_obj{1}.euler_angs);
            hip_uu2 = hip_uu2-hip(:,2);
            hip_uu3 = hip_uu3-hip(:,3);
            
            obj.joint_obj{1}.joint_rotmat_profile = a;
            obj.joint_obj{2}.joint_rotmat_profile = b;
            obj.joint_obj{3}.joint_rotmat_profile = c;
            
                obj.joint_obj{1}.uu_joint2  = hip_uu2;
                obj.joint_obj{1}.uu_joint3  = hip_uu3;
                obj.joint_obj{1}.uuw_joint = zeros(3,size(jointMotion,1))+obj.CR_bodies(:,:,1)*obj.joint_obj{1}.uu_joint;
                obj.joint_obj{1}.uuw_joint2 = obj.CR_bodies(:,:,1)*hip_uu2;
                obj.joint_obj{1}.uuw_joint3 = obj.CR_bodies(:,:,1)*hip_uu3;
            kneeworld = obj.CR_bodies(:,:,1)*obj.CR_bodies(:,:,2);
                obj.joint_obj{2}.uu_joint2  = knee_uu2;
                obj.joint_obj{2}.uu_joint3  = knee_uu3;
                obj.joint_obj{2}.uuw_joint  = zeros(3,size(jointMotion,1))+kneeworld*obj.joint_obj{2}.uu_joint;
                obj.joint_obj{2}.uuw_joint2 = kneeworld*knee_uu2;
                obj.joint_obj{2}.uuw_joint3 = kneeworld*knee_uu3;
            ankleworld = obj.CR_bodies(:,:,1)*obj.CR_bodies(:,:,2)*obj.CR_bodies(:,:,3);
                obj.joint_obj{3}.uu_joint2  = ankle_uu2;
                obj.joint_obj{3}.uu_joint3  = ankle_uu3;
                obj.joint_obj{3}.uuw_joint  = zeros(3,size(jointMotion,1))+ankleworld*obj.joint_obj{3}.uu_joint;
                obj.joint_obj{3}.uuw_joint2 = ankleworld*ankle_uu2;
                obj.joint_obj{3}.uuw_joint3 = ankleworld*ankle_uu3;
        end
        %% Function: Store the Torque and Kinematic Data from Torque, Theta, ThetaDot input
        function store_torque_and_kinematic_data(obj,theta,theta_dot,dt)
            %Saves torque, angle, and velocity data from desired motions
            %obj.torque_motion = torque;
            %obj.theta_motion = theta;
            %obj.theta_dot_motion = theta_dot;
            
            %Torque data
            % Torque for only stance in units of [Nmm]
                    %load('TorqueinNmm.mat');
            % Simulated torque for stance and swing in units of [Nm]
            % Generated by ConnectData.m in the Rat/Main Sim and Ani compare files
            % TorqueAll refers to torque for all joints for both stance and swing
            % TorqueinNm is -1*measured stance torque. The swing portion of torque is generated by a Simulink model
            if ismac
                load('/Volumes/GoogleDrive/My Drive/Rat/Main Sim and Ani compare files/AllTraining20180824.mat','TorqueAll');
            else
                load([pwd,'\Data\AllTraining20180824.mat'],'TorqueAll');
            end
            torque = TorqueAll;
            obj.dt_motion = dt;
            for i=1:obj.num_bodies-1
                obj.joint_obj{i}.rec_angle_profile = theta(:,i+1);
                obj.joint_obj{i}.rec_angle_time = theta(:,1);
                obj.joint_obj{i}.rec_angleDot_profile = theta_dot(:,i+1);
                obj.joint_obj{i}.rec_torque_profile = torque(:,i);
            end
            obj.theta_motion = theta(:,2:4);
            obj.theta_motion_time = theta(:,1);
            obj.theta_dot_motion = theta_dot(:,2:4);
            obj.dt_motion = dt;
            obj.torque_motion = TorqueAll;
        end
        %% Function: Store Joint Position Profiles in Joint Objects
        function store_jointbodyw_position_profiles(obj,joint_profile)
            sizer    = zeros(size(joint_profile,1),3);
            kneepos  = sizer;
            anklepos = sizer;
            footpos  = sizer;
            tibiapos = sizer;
            
            hippos = sizer+obj.joint_obj{1}.init_pos_w';
            femurpos = sizer+(obj.CR_bodies(:,:,1)*obj.pos_bodies(:,2))';
            parfor i=1:size(joint_profile,1)
                a = axis_angle_rotation(obj,joint_profile(i,2),obj.joint_obj{1}.uu_joint);
                b = axis_angle_rotation(obj,joint_profile(i,3),obj.joint_obj{2}.uu_joint);
                c = axis_angle_rotation(obj,joint_profile(i,4),obj.joint_obj{3}.uu_joint);
                
                tibiapos(i,:) = obj.CR_bodies(:,:,1)*obj.pos_bodies(:,2)+obj.CR_bodies(:,:,1)*a*obj.CR_bodies(:,:,2)*obj.pos_bodies(:,3);
                kneerel = obj.CR_bodies(:,:,1)*a*obj.CR_bodies(:,:,2)*b*obj.CR_bodies(:,:,3);
                kneepos(i,:) = tibiapos(i,:)+(kneerel*obj.joint_obj{2}.init_pos)';
%                 e(i,:) = (obj.CR_bodies(:,:,1)*(obj.pos_bodies(:,2)+a*obj.CR_bodies(:,:,2)*(obj.pos_bodies(:,3)+b*obj.CR_bodies(:,:,3)*obj.joint_obj{2}.init_pos)))';
                footpos(i,:) = tibiapos(i,:)+(kneerel*obj.pos_bodies(:,4))';
                anklerel = kneerel*c*obj.CR_bodies(:,:,4);
                anklepos(i,:) = footpos(i,:)+ (anklerel*obj.joint_obj{3}.init_pos)';
%                f(i,:) = (obj.CR_bodies(:,:,1)*(obj.pos_bodies(:,2)+a*obj.CR_bodies(:,:,2)*(obj.pos_bodies(:,3)+b*obj.CR_bodies(:,:,3)*(obj.pos_bodies(:,4)+c*obj.CR_bodies(:,:,4)*obj.joint_obj{3}.init_pos))))';
                
                %kneepos =  kneepos(i,:)*1000;
                %jointangles = joint_profile(i,2:4)';
            end
            holder = cell(4,1);
            holder{1,1} = sizer;
            holder{2,1} = femurpos;
            holder{3,1} = tibiapos;
            holder{4,1} = footpos;
            obj.pos_bodies_w_profile = holder;
            obj.joint_obj{1}.sim_position_profile = hippos;
            obj.joint_obj{2}.sim_position_profile = kneepos;
            %obj.joint_obj{2}.sim_position_profile = kneepos+.001*ones(length(kneepos),1);
            obj.joint_obj{3}.sim_position_profile = anklepos;
        end
        %% Function: Store Animatlab Parameters Simulation File in the Muscle Objects
        function store_animatlab_params( obj )

            num_musc = length(obj.musc_inds);
            
%             load('AnimatlabProperties.mat');
            load([pwd,'\Data\AnimatlabProperties.mat']);
            

    %             property_names = repmat(Properties{4,1}(:,1),num_musc,1);
            property_names = Properties{4,1}(:,1);
            numProps = length(property_names);
            property_values = zeros(num_musc*numProps,1);
            property_inds = zeros(num_musc*numProps,3);

            %for each property
            for i=1:num_musc

                %Define its line as the "lower limit." All properties will be found
                %after it in the file.
                lower_limit = obj.musc_inds(i);
                for j=1:numProps
                    if strcmp(property_names{j},'damping')
                        %We should find muscle damping, which is actually called "B,"
                        %just like the muscle activation's curve. Therefore we need to
                        %be a little more specific. We want the second B that shows up
                        %for a particular muscle.
                        prop_to_find = '<B>';
                        which_to_find = 2;
                    else
                        %Now find the desired property, formatted like the file.
                        prop_to_find = ['<',property_names{j},'>'];
                        which_to_find = 1;
                    end

                    %Find that string in the file, the first time after the lower limit
                    %(where the named object is found). 
                    %prop_found = strfind(obj.original_text(lower_limit:end),prop_to_find);
                    prop_found = contains(obj.original_text(lower_limit:end),prop_to_find);

                    %Find the index at which this occurs, and save this for quick
                    %reference later. Remember that we were only examining
                    %original_text after the lower_limit, so we need to add that back
                    %on. -1 makes the indexing work properly.
                    %temp = find(~cellfun(@isempty,prop_found)) + lower_limit - 1;
                    temp = find(prop_found) + lower_limit - 1;
                    property_inds(numProps*(i-1)+j,1) = temp(which_to_find);

                    %Find the final index of the row to keep before the number begins
                    %Number we're looking for is formatted like
                    %'<A>-0.04<A>'
                    %Index of > before the number we want
                    property_inds(numProps*(i-1)+j,2) = length(prop_to_find);

                    %Find the first index of the row to keep after the number begins
                    %Index of < after number we want
                    property_inds(numProps*(i-1)+j,3) = cell2mat(strfind(obj.original_text(property_inds(numProps*(i-1)+j,1)),'</'));
                end
            end

            %For each property that we're changing, read in the value from
            %the current simulation
            for i=1:size(property_inds,1) %Each property
                property_values(i) = str2double(obj.original_text{property_inds(i,1)}(property_inds(i,2)+1:property_inds(i,3)-1));
            end

            %Make a case-insensitive list of all the muscles on this leg
            musc_names = obj.original_text(obj.musc_inds);
            for i=1:num_musc
                obj.musc_obj{i,1}.muscle_name = lower(musc_names{i}(7:end-7));
                obj.musc_obj{i,1}.muscle_index = obj.musc_inds(i);
                %Store the muscle properties in individual muscle objects
                obj.musc_obj{i}.x_off = property_values(numProps*i-9);
                obj.musc_obj{i}.ST_max = property_values(numProps*i-8);
                obj.musc_obj{i}.steepness = property_values(numProps*i-7);
                obj.musc_obj{i}.y_off = property_values(numProps*i-6);
                obj.musc_obj{i}.RestingLength = property_values(numProps*i-5);
                obj.musc_obj{i}.l_width = property_values(numProps*i-4);
                obj.musc_obj{i}.Kse = property_values(numProps*i-3);
                obj.musc_obj{i}.Kpe = property_values(numProps*i-2);
                obj.musc_obj{i}.damping = property_values(numProps*i-1);
                obj.musc_obj{i}.max_force = property_values(numProps*i);
            end
        end
        %% Function: Store Joint Parameters from Animatlab Sim File
        function store_joint_params(obj)
            numJoints = length(obj.joint_obj);
            ot = obj.original_text;
            for ii = 1:numJoints
                joint = obj.joint_obj{ii};
                jInd = obj.joint_obj{ii}.index;
                fricCoeff = find(contains(ot(jInd:end),'<Coefficient>'),1,'first')+jInd-1;
                joint.fricCoeff = double(extractBetween(string(ot{fricCoeff}),'>','</'));
                obj.joint_obj{ii} = joint;
            end
        end
        %% Function: Store Muscle Parameters from Johnson 2011 Paper in the Muscle Objects
        function store_johnson_params(obj)
            params = [];
            num_musc = length(obj.musc_inds);
            load([pwd,'\Data\Johnson2011MuscleParameters.mat'],'params');
            %We don't want to waste time going through the entire list if we've already found the muscle we're looking for.
            %Make a list of the muscle and switch off the ones we've already found
            muscle_list = ones(num_musc,1);
            for i=2:size(params,1)
                for j=1:num_musc
                    %Make sure that the Excel muscle names are in the same format as the Animatlab ones
                    name_no_space = params{i,1}{1}(~isspace(params{i,1}{1}));
                    nameAnimatlab = obj.musc_obj{j}.muscle_name;
                    if muscle_list(j,1) == 1
                        if contains(obj.musc_obj{j}.muscle_name,name_no_space)
                            %Switch this muscle off so we skip it next time
                            muscle_list(j,1) = 0;
                            %Muscle mass in [mg]
                            obj.musc_obj{j}.mass = params{i,2};
                            %Optimal muscle fiber length in [mm]
                            obj.musc_obj{j}.opt_fiber_length = params{i,3};
                            %Pinnation angle in [deg]
                            obj.musc_obj{j}.pennation_angle = params{i,4};
                            %Maximum isometric force in [g]
                            obj.musc_obj{j}.Po = params{i,5};
                            %Maximum fiber shortening velocity in [mm/s]
                            obj.musc_obj{j}.vmax_fiber = params{i,6};
                            %Tendon slack length in [mm]
                            obj.musc_obj{j}.tendon_sl = params{i,7};
                            %Optimal muscle length from Eng 2008 (cm)
                            obj.musc_obj{j}.opt_muscle_length = params{i,8};
                            %Lf/Lm, fiber length to muscle length ratio
                            obj.musc_obj{j}.lf_lm = params{i,9};
                            break
                        end
                    end
                end
            end
        end
        %% Function: Store Muscle Profiles, Attachment Points, and Muscle Length Profiles in Muscle Objects
        function store_muscle_profiles(obj)
            %Uses the joint rotation matrices at each timestep to calculate the muscle length and velocity. Stores this as a profile in the muscle object.
            num_steps = size(obj.joint_obj{1}.joint_rotmat_profile,3);
           
            attachments_body = cell(4,1);
            attdirect = attachments_body;
            attachments_post = cell(4,1,num_steps);
            
            %Increments the pointer for each body so that new attachment entries don't overwrite previous ones
            %Also used to count how many attachments each body has
            pcount = 1;
            fcount = 1;
            tcount = 1;
            ftcount = 1;
            
            %This loop builds two cell arrays: one that holds the attachment positions for each body and one that functions as a directory for storing the
            %attachment positions in the correct muscles. The directoy holds (for each body) the muscle number in the top row and the attachment number in the
            %bottom row.
            for i = 1:size(obj.musc_obj,1)
                for j = 1:size(obj.musc_obj{i}.pos_attachments,1)
                    switch obj.musc_obj{i}.pos_attachments{j,3}
                        case 1
                            attachments_body{1}(:,pcount) = obj.musc_obj{i}.pos_attachments{j,1};
                            attdirect{1}(1,pcount) = i;
                            attdirect{1}(2,pcount) = j;
                            pcount = pcount + 1;
                        case 2
                            attachments_body{2}(:,fcount) = obj.musc_obj{i}.pos_attachments{j,1};
                            attdirect{2}(1,fcount) = i;
                            attdirect{2}(2,fcount) = j;
                            fcount = fcount + 1;
                        case 3
                            attachments_body{3}(:,tcount) = obj.musc_obj{i}.pos_attachments{j,1};
                            attdirect{3}(1,tcount) = i;
                            attdirect{3}(2,tcount) = j;
                            tcount = tcount + 1;
                        case 4
                            attachments_body{4}(:,ftcount) = obj.musc_obj{i}.pos_attachments{j,1};
                            attdirect{4}(1,ftcount) = i;
                            attdirect{4}(2,ftcount) = j;
                            ftcount = ftcount + 1;
                    end
                end
            end
            
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
            
            %Now to actually move the attachments through the motion. This loop builds a post-motion cell array for each body
           parfor i = 1:num_steps
                a = pelvisCR*attachments_body{1};
                b = pelvisCR*(femurPos+hipJointRotMat(:,:,i)*femurCR*attachments_body{2});
                c = pelvisCR*(femurPos+hipJointRotMat(:,:,i)*femurCR*...
                    (tibiaPos+kneeJointRotMat(:,:,i)*tibiaCR*attachments_body{3}));
                d = pelvisCR*(femurPos+hipJointRotMat(:,:,i)*femurCR*...
                    (tibiaPos+kneeJointRotMat(:,:,i)*tibiaCR*...
                    (footPos+ankleJointRotMat(:,:,i)*footCR*attachments_body{4})));
                attachments_post(:,:,i) = [{a};{b};{c};{d}];
            end
            
            %This loop creates an array of the attachment's motion and then stores it in the correct muscle object from the directory
            for j = 1:size(attachments_post,1)
                num_muscles = size(attachments_post{j},2);
                bb = [attachments_post{j,1,:}]';
                r = size(bb);
                for k = 1:size(attachments_post{j},2)
                    xx = k:num_muscles:r;
                    temp = bb(xx,:)';
                    musclenumber = attdirect{j}(1,k);
                    attnumber = attdirect{j}(2,k);
                    obj.musc_obj{musclenumber}.pos_attachments{attnumber,4} = temp';
                end
            end
            
            %This loop calculates the muscle length from the attachment points and stores the length profile (and velocity) in the muscle object
            for i = 1:length(obj.musc_obj)
                %Put this muscle's attachments into a single matrix
                attachmentmatrix = cell2mat(obj.musc_obj{i}.pos_attachments(:,4));
                attatts = zeros(num_steps,size(obj.musc_obj{i}.pos_attachments,1)-1);
                for k = 1:size(obj.musc_obj{i}.pos_attachments,1)-1
                    %This just iterates through each attachment pair, subtracting the distal position from the proximal position.
                    %Since attachment positions are in one long matrix, we have to set moving pointers for the beginning and end of each attachment profile
                    temp = attachmentmatrix((k*num_steps+1):((k+1)*num_steps),:)-attachmentmatrix(((k-1)*num_steps+1):k*num_steps,:);
                    %Once we have the difference between the two attachments, take the vector norm along the second dimension (each row)
                    attatts(:,k) = vecnorm(temp,2,2);
                end
                %Each column of attatts corresponds to the length of a different segment. Summing each segment gives us the full muscle length.
                muscle_length = sum(attatts,2);
                obj.musc_obj{i}.muscle_length_profile = muscle_length;
                obj.musc_obj{i}.l_min = min(muscle_length);
                obj.musc_obj{i}.l_max = max(muscle_length);
                obj.musc_obj{i}.muscle_velocity_profile = diff(muscle_length)/obj.dt_motion;
            end   
        end
        %% Function: Store Input Muscle Passive Tension
        function store_input_muscle_passive_tension(obj,passive_tension)
            pt = passive_tension;
            musc_names = cell(size(obj.musc_obj,1),1);
            for ii = 1:size(obj.musc_obj,1)
                musc_names{ii,1} = obj.musc_obj{ii}.muscle_name;
            end
            for ii = 1:size(passive_tension,1)
                ptName = pt{ii,1};
                obj.musc_obj{contains(musc_names,ptName)}.passive_tension = pt{ii,2};
            end
        end
        %% Function: Store the sampling vector
        function store_sampling_vector(obj)
            %% Optimization and simulations can take ages if not downsampled. This function creates a sampling vector which parses down analysis to a single step
            [beg,ennd,~] = find_step_indices(obj);
            div = 500;
            obj.sampling_vector = floor(linspace(beg,ennd,div));
            
%             alt = linspace(1,length(obj.theta_motion),length(obj.theta_motion));
%             obj.sampling_vector = alt;
        end
        %% Function: Find the Global Point Position Profile for a Local Point (input) in a Body Coordinate Frame (input)
        function [point_profile] = move_point_through_walking(obj,bodynum,localpoint)
            %Uses the joint rotation matrices at each timestep to calculate the muscle length and velocity. Stores this as a profile in the muscle object.
            num_steps = size(obj.joint_obj{1}.joint_rotmat_profile,3);
            if size(localpoint,2) == 3
                localpoint = localpoint';
            end
            
            if log(norm(localpoint)/norm(obj.pos_bodies(:,2))) > 2
                localpoint = localpoint./1000;
            end
            
            point_profile = zeros(3,num_steps);
            
            switch bodynum
                case 1
                    for i = 1:num_steps
                        point_profile(:,i) = obj.CR_bodies(:,:,1)*localpoint;
                    end
                case 2
                    for i = 1:num_steps
                        point_profile(:,i) = obj.CR_bodies(:,:,1)*(obj.pos_bodies(:,2)+obj.joint_obj{1}.joint_rotmat_profile(:,:,i)*obj.CR_bodies(:,:,2)*localpoint);
                    end
                case 3
                    for i = 1:num_steps
                        point_profile(:,i) = obj.CR_bodies(:,:,1)*(obj.pos_bodies(:,2)+obj.joint_obj{1}.joint_rotmat_profile(:,:,i)*obj.CR_bodies(:,:,2)*...
                            (obj.pos_bodies(:,3)+obj.joint_obj{2}.joint_rotmat_profile(:,:,i)*obj.CR_bodies(:,:,3)*localpoint));
                    end
                case 4
                    for i = 1:num_steps
                        point_profile(:,i) = obj.CR_bodies(:,:,1)*(obj.pos_bodies(:,2)+obj.joint_obj{1}.joint_rotmat_profile(:,:,i)*obj.CR_bodies(:,:,2)*...
                            (obj.pos_bodies(:,3)+obj.joint_obj{2}.joint_rotmat_profile(:,:,i)*obj.CR_bodies(:,:,3)*...
                            (obj.pos_bodies(:,4)+obj.joint_obj{3}.joint_rotmat_profile(:,:,i)*obj.CR_bodies(:,:,4)*localpoint)));
                    end
            end 
        end
        %% Function: Write Muscle Parameters to Animatlab
        function [parameters] = write_parameters_to_animatlab(obj)
            num_muscles = size(obj.musc_obj,1);
            parameters = cell(num_muscles,1);
            project_file = importdata(obj.proj_file);
            muscle_addresses = contains(project_file,'<Type>LinearHillMuscle</Type>');
            muscle_indices = find(muscle_addresses)-2;
            %Some paramters terms have placeholders because we want to write that parameter to our Matlab objects but not overwrite them in the simulation file
            parameter_terms = {'NamePlaceholder';...
                               '<B Value';...
                               '<Lwidth Value';...
                               'VmaxPlaceHolder';...
                               '<Kse Value';...
                               '<Kpe Value';...
                               '<B Value';...
                               '<D Value';...
                               '<LowerLimitScale Value';...
                               '<UpperLimitScale Value';...
                               '<RestingLength'};
            for i=1:num_muscles
                parameters{1,1} = 'Muscle name';
                parameters{i+1,1} = obj.musc_obj{i}.muscle_name;
                parameters{1,2} = 'Maximum force';
                parameters{i+1,2} = obj.musc_obj{i}.Po*(9.8/1000);
                parameters{1,3} = 'L_width (cm)';
                parameters{i+1,3} = calculate_muscle_width(obj,obj.musc_obj{i})*100;
                parameters{1,4} = 'V_max Muscle';
                parameters{i+1,4} = (1/obj.musc_obj{i}.lf_lm)*obj.musc_obj{i}.vmax_fiber;
                parameters{1,5} = 'Kse';
                if obj.musc_obj{i}.tendon_sl == 0
                    %This is a trendline found from comparing tendon slack length to muscle mass. There was a R^2 = .7089 correlation between the parameters.  
                    parameters{i+1,5} = 78.278*obj.musc_obj{i}.mass+6952.7;
                else
                    parameters{i+1,5} = 2.5*parameters{i+1,2}/(0.067*0.001*obj.musc_obj{i}.tendon_sl);
                end
                parameters{1,6} = 'Kpe';
                parameters{i+1,6} = (.3*parameters{i+1,5}*parameters{i+1,2})/(parameters{i+1,5}*(obj.musc_obj{i}.l_max-obj.musc_obj{i}.l_min)-.3*parameters{i+1,2});
                if parameters{i+1,6} < 0
                    disp('Something is wrong, the calculated Kpe value is negative.')
                    keyboard
                end
                %The damping constant is the maximum muscle force in N divded by the maximum velocity of the muscle in m/s
                parameters{1,7} = 'B';
                parameters{i+1,7} = (parameters{i+1,2}/parameters{i+1,4})*1000;
                parameters{1,8} = 'Yoffset (mN)';
                parameters{i+1,8} = -2.3*parameters{i+1,2};
                parameters{1,9} = 'l_min (cm)';
                parameters{i+1,9} = min(obj.musc_obj{i}.muscle_length_profile)*100;
                parameters{1,10} = 'l_max (cm)';
                parameters{i+1,10} = max(obj.musc_obj{i}.muscle_length_profile)*100;
                parameters{1,11} = 'l_rest (cm)';
                parameters{i+1,11} = parameters{i+1,10};
                for j=1:size(parameters,2)
                    lower_limit = muscle_indices(i);
                    scale = 1;
                    if j ~= 1 && j ~= 4 
                        if j == 7
                            %For some dumb reason, the creator fo Animatlab has two parameters named 'B'. In order to put damping in the right place, we have to
                            %skip over the first B.
                            prop_addresses = contains(project_file(lower_limit:end),parameter_terms{j});
                            lower_limit = find(prop_addresses,1,'first')+lower_limit;
                        end
                        if j == 9 || j == 10
                            %We need to do a similar skip over w Lmin and Lmax since they're used for two different things.
                            prop_addresses = contains(project_file(lower_limit:end),parameter_terms{j});
                            lower_limit = find(prop_addresses,1,'first')+lower_limit;
                        end
                    prop_addresses = contains(project_file(lower_limit:end),parameter_terms{j});
                    prop_index = find(prop_addresses,1,'first')+lower_limit-1;
                    %Find the line with the parameter
                    line_of_interest = project_file{prop_index};
                    if contains(line_of_interest,'None') == 1
                    else
                        if contains(line_of_interest,'milli') == 1
                            scale = 1000;
                        elseif contains(line_of_interest,'centi') == 1
                            scale = 100;
                        end
                    end
                    quotelocs = strfind(line_of_interest,'"');
                    modified_line = strcat(line_of_interest(1:quotelocs(1)),num2str(parameters{i+1,j}),line_of_interest(quotelocs(2):quotelocs(end-1)),num2str(parameters{i+1,j}/scale),line_of_interest(quotelocs(end):end));
                    %Replace the line with the modified parameter
                    project_file{prop_index} = modified_line;
                    end
                end
            end
            carry_on = input('You are about to overwrite the Animatlab project file you''re using with new parameters.\n This could permanently ruin your project file if there are errors.\n If this is what you want to do, type YES. Otherwise, the program will not overwrite.\n','s');
            if strcmp(carry_on,'YES')
                filename = 'G:\My Drive\Rat\Optimizer\CalculatedPropertiesForAnimatlab.xlsx';
                xlswrite(filename,parameters);
                %file_path = strcat(obj.proj_file(1:end-6),'_2',obj.proj_file(end-5:end));
                file_path = strcat(obj.proj_file);
                fileID = fopen(file_path,'w');
                formatSpec = '%s\n';
                nrows = size(project_file);
                for row = 1:nrows
                    fprintf(fileID,formatSpec,project_file{row,:});
                end
                fclose(fileID);
            end
        end
        %% Function: Calculate the rotation matrix for an arbitrary set of three angles
        function C_mat = three_axis_rotation(obj,angs)
            %Take in a euler angle triplet, and return the associated
            %rotation matrix.
            c1 = cos(angs(1));
            s1 = sin(angs(1));
            c2 = cos(angs(2));
            s2 = sin(angs(2));
            c3 = cos(angs(3));
            s3 = sin(angs(3));
            %XYZ            
            C_mat = [c2*c3,-c2*s3,s2;...
                     c1*s3+c3*s1*s2,c1*c3-s1*s2*s3,-c2*s1;...
                     s1*s3-c1*c3*s2,c3*s1+c1*s2*s3,c1*c2];
        end
        %% Function: Calculate Rotation Matrix for an Input Rotation Angle
        function C = axis_angle_rotation(obj,angle,joint_axis)
            c = cos(angle);
            s = sin(angle);

            a1 = joint_axis(1);
            a2 = joint_axis(2);
            a3 = joint_axis(3);
            
            C = [c+a1^2*(1-c), a1*a2*(1-c)-a3*s, a1*a3*(1-c)+a2*s;...
                 a1*a2*(1-c)+a3*s, c+a2^2*(1-c), a2*a3*(1-c)-a1*s;...
                 a1*a3*(1-c)-a2*s, a2*a3*(1-c)+a1*s, c+a3^2*(1-c)];
        end
        %% Function: FIND find_step_indices: STEP INDICES Find the bounding indices for a single step
        function [beg,ennd,mid] = find_step_indices(obj)
            %Finds the bounding indices for the second step in walking. This will be used to find muscle moment arms and passive tension
            %Over a step since individual joint motion can't be isolated. Using the second step prevents issues with initialization of the first step (t==0)
            
%             jointprofile = obj.theta_motion;
%             if size(unique(jointprofile(:,1)),1) == 1
%                 if size(unique(jointprofile(:,2)),1) == 1
%                     muscleprofile = jointprofile(:,3);
%                     [~,maxlocs] = findpeaks(jointprofile(:,3));
%                     [~,minlocs] = findpeaks(-jointprofile(:,3));
%                 else
%                     muscleprofile = jointprofile(:,2);
%                     [~,maxlocs] = findpeaks(jointprofile(:,2));
%                     [~,minlocs] = findpeaks(-jointprofile(:,2));
%                 end
%             else
%                 muscleprofile = jointprofile(:,1);
%                 [~,maxlocs] = findpeaks(jointprofile(:,1));
%                 [~,minlocs] = findpeaks(-jointprofile(:,1));
%             end
%             
%             a3 = unique([maxlocs;minlocs]);
%             a4 = muscleprofile(a3);
%             for i = 1:length(a4)-1
%                 %signer(i,1) = sign(a4(i+1,1)-a4(i,1));
%                 range(i,1) = abs(a4(i+1,1)-a4(i,1));
%             end
%             [~,dd] = max(range);
%             dd = dd(1);
%             beg = a3(dd);
%             ennd = a3(dd+1);
%             mid = floor((beg+ennd)/2);
            
            jointprofile = obj.theta_motion;
            trimmedjp = jointprofile(floor(length(jointprofile)*.1):floor(length(jointprofile)*.9),:);
            if mean(range(trimmedjp)) < 1e-3 || mean(std(trimmedjp)) < .01
                % If jointprofile is constant
                beg = floor(length(jointprofile)*(1/3));
                ennd = floor(length(jointprofile)*(2/3));
                mid = floor((beg+ennd)/2);
                return
            end
            
            if size(unique(jointprofile(:,1)),1) == 1
                if size(unique(jointprofile(:,2)),1) == 1
                    maxvals = findpeaks(jointprofile(:,3));
                    [minvals,minlocs]= findpeaks(-jointprofile(:,3));
                else
                    maxvals = findpeaks(jointprofile(:,2));
                    [minvals,minlocs]= findpeaks(-jointprofile(:,2));
                end
            else
                maxvals = findpeaks(jointprofile(:,1));
                [minvals,minlocs]= findpeaks(-jointprofile(:,1));
            end
            if size(maxvals,1) > 2
%                 [minvals,minlocs]= findpeaks(-jointprofile(:,1));
                minvals = -minvals;
                if size(minvals,1) == size(maxvals,1)
                    beg = minlocs(1);
                    ennd = minlocs(2);
                else
                    beg = minlocs(2);
                    ennd = minlocs(3);
                end
                mid = floor((beg+ennd)/2);
            elseif size(maxvals,1) == 1 && size(minvals,1) == 1
                beg = 1;
                ennd = size(jointprofile,1);
                mid = floor((beg+ennd)/2);
            else
                plot(jointprofile)
                disp('Are you using a simulation that has at least three steps? Check that the boundaries are going to be defined correctly for a single step')
                return
            end
        end
        %% Function: MUSCLE MOMENT ARM Compute a single muscle's moment arm over an entire walking cycle
        function [moment_arm_profile] = compute_muscle_moment_arm(obj,muscle,axis_used,joint,plotval)
            
            if muscle.pos_attachments{1,3} > joint || muscle.pos_attachments{end,3} < joint
                disp('This muscle may not bridge the joint specified')
                keyboard
                moment_arm_profile = -1;
                return
            end
            
            if nargin < 4 || ~ismember(plotval,[0 1])
                plotval = 0;
            end

            scale = 1000;
            
            if joint >= 1 && joint <= 3
            else
                joint = input('Joint specified incorrectly. hip = 1, knee = 2, ankle = 3');
            end

            [beg,ennd,~] = find_step_indices(obj);
            %Higher div, the longer the program takes.
            div = 500;
            plotX = floor(linspace(beg,ennd,div));
            xx = linspace(1,length(obj.theta_motion),length(obj.theta_motion));
            xx = obj.sampling_vector;

            axis = ['Ext/Flx';'Abd/Add';'ExR/InR'];
            %Defining values for plot axis limits
            if plotval == 1 
                jointmat = [obj.joint_obj{1}.sim_position_profile(beg:ennd,:);...
                                obj.joint_obj{2}.sim_position_profile(beg:ennd,:);...
                                obj.joint_obj{3}.sim_position_profile(beg:ennd,:)];
                toemat = obj.musc_obj{20,1}.pos_attachments{5,4};
                jointaxmat = (1/100).*[obj.joint_obj{joint}.uuw_joint(:,xx)';...
                    obj.joint_obj{joint}.uuw_joint2(:,xx)';...
                    obj.joint_obj{joint}.uuw_joint3(:,xx)'];
                
                muscmax = max(cell2mat(muscle.pos_attachments(:,4)));
                muscmin = min(cell2mat(muscle.pos_attachments(:,4)));
                jointmax = max(jointmat);
                jointmin = min(jointmat);
                toemax = max(toemat);
                toemin = min(toemat);
                jointaxmax = max(jointaxmat);
                jointaxmin = min(jointaxmat);

                posmax = scale*max([jointmax;...
                                     muscmax;...
                                     toemax;...
                                     jointaxmax]);
                posmin = scale*min([jointmin;...
                                    muscmin;...
                                    toemin;...
                                    jointaxmin]);
                limits =  [posmin' posmax'];
                xlims = limits(1,:);
                ylims = limits(2,:);
                zlims = limits(3,:);
                %Can change limscale to give more or less viewing space
                limscale = 1;
            end 
                moment_arm_profile = zeros(size(xx,2),5);
                moment_arm_profile(:,1) = obj.joint_obj{joint}.rec_angle_time(xx);
                moment_arm_profile(:,2) = obj.joint_obj{joint}.rec_angle_profile(xx)*(180/pi);

            for j = axis_used
                %f1 = figure(1);
                %for i=1:length(jointprofile)
                count2 = 0;
                whichsegmentisfree = diff(cell2mat(muscle.pos_attachments(:,3)));
                if size(find(whichsegmentisfree>0),1) > 1
                    atts = cell2mat(muscle.pos_attachments(:,3));
                    holder  = atts-joint;
                    seg2use = find(holder==1,1,'first')-1;
                    if whichsegmentisfree(seg2use,1) == 0
                        keyboard
                    end
                else
                    [~,seg2use] = max(whichsegmentisfree);
                end
                for i=xx
                    count2 = count2 + 1;
                    % Joint axis in global coordinates
                    jointv = (scale/100).*[obj.joint_obj{joint}.uuw_joint(:,i),obj.joint_obj{joint}.uuw_joint2(:,i),obj.joint_obj{joint}.uuw_joint3(:,i)];
                    pointrel = zeros(2,3);
                    pointprojection = pointrel;
                    for k = 1:2
                        pointrel(k,:) = muscle.pos_attachments{seg2use+k-1,4}(i,:)' - obj.joint_obj{joint}.sim_position_profile(i,:)';
                        pointprojection(k,:) = scale*(pointrel(k,:) - (dot(pointrel(k,:),jointv(:,j)')/norm(jointv(:,j)')^2)*jointv(:,j)')+scale*obj.joint_obj{joint}.sim_position_profile(i,:);
                    end
                    %Matrix of muscle projections perpendicular to the joint axis
                        fflatmat = pointprojection(2,:)-pointprojection(1,:);
                    %Storing the moment arms in a matrix
                        moment_arm_long = cross(fflatmat,jointv(:,j));
                        scaledjoint = scale*obj.joint_obj{joint}.sim_position_profile(i,:);
                        PA2 = [pointprojection(1,:);scaledjoint];
                        PB2 = [pointprojection(2,:);scaledjoint+moment_arm_long];
                        %lineINtersect3D gives the scaled vector of the moment arms for each segment
                        moment_arm = lineIntersect3D(obj,PA2,PB2);
                        %Storing the moment arms in a matrix
                        %A modifier to determine whether the moment arm is positive or negative
                        sig_momentarm = moment_arm-scaledjoint;
                        sig_muscleprojection = (pointprojection(1,:)-pointprojection(2,:));
                        signal2 = sign(dot(cross(sig_momentarm,sig_muscleprojection),jointv(:,j)'));
                        %moment_arm_length2(i-beg+1,j+1) = momentlen2;
                        moment_arm_profile(count2,j+2) = signal2*norm(sig_momentarm);
                    %% For plotting the muscle and joint in 3D as the muscle moves
                    %Unnecessary to plot every time step. This sets it to plot only after a certain number of steps
                    if mod(count2,floor(length(xx)/24))==0
                        plotter = 1;
                    else
                        plotter = 0;
                    end
                    if plotval == 1 && plotter
                        if i==floor(length(xx)/24)
                            figure('name','legplot','Position',[-955   130   960   985]);
                        end
                        %The muscle vector
                        for k = 1:size(muscle.pos_attachments,1)
                            musclevecco(k,:) = scale*muscle.pos_attachments{k,4}(i,:);
                        end
                        jointvecco1 = [scaledjoint;scaledjoint+jointv(:,1)'];
                        jointvecco2 = [scaledjoint;scaledjoint+jointv(:,2)'];
                        jointvecco3 = [scaledjoint;scaledjoint+jointv(:,3)'];
                        femur = scale*[obj.joint_obj{1}.sim_position_profile(i,1),obj.joint_obj{1}.sim_position_profile(i,2),obj.joint_obj{1}.sim_position_profile(i,3);...
                            obj.joint_obj{2}.sim_position_profile(i,1),obj.joint_obj{2}.sim_position_profile(i,2),obj.joint_obj{2}.sim_position_profile(i,3)];
                        tibia = scale*[obj.joint_obj{2}.sim_position_profile(i,1),obj.joint_obj{2}.sim_position_profile(i,2),obj.joint_obj{2}.sim_position_profile(i,3);...
                            obj.joint_obj{3}.sim_position_profile(i,1),obj.joint_obj{3}.sim_position_profile(i,2),obj.joint_obj{3}.sim_position_profile(i,3)];
                        foot = scale*[obj.joint_obj{3}.sim_position_profile(i,1),obj.joint_obj{3}.sim_position_profile(i,2),obj.joint_obj{3}.sim_position_profile(i,3);...
                            obj.musc_obj{20, 1}.pos_attachments{5,4}(i,:)];
                                %flongco = [fflat(:,i-beg+1)'+scale*muscle.pos_attachments{1,4}(i,:);flong(:,i-beg+1)'+fflat(:,i-beg+1)'+scale*muscle.pos_attachments{1,4}(i,:)];
                                %fflatco = [scale*muscle.pos_attachments{1,4}(i,:);fflat(:,i-beg+1)'+scale*muscle.pos_attachments{1,4}(i,:)];
                                %mprojection = [originprojection';insprojection'];
                        %Plot the muscle
                        gcf;
                        plot3(musclevecco(:,1),musclevecco(:,2),musclevecco(:,3),'r', 'LineWidth', 2)
                        %Thicken the muscle
%                         set(findobj(gca, 'Type', 'Line', 'Linestyle', '-'), 'LineWidth', 2);
                        hold on
                        %Plot the moment arm average
                        singlesegmentmomentarm = [scaledjoint;moment_arm];
                        plot3(singlesegmentmomentarm(:,1),singlesegmentmomentarm(:,2),singlesegmentmomentarm(:,3),'m','LineWidth',2)                  
                        
                        %Plot the muscle origin
                        plot3(scale*muscle.pos_attachments{1,4}(i,1),scale*muscle.pos_attachments{1,4}(i,2),scale*muscle.pos_attachments{1,4}(i,3),'ro')
                        %Plot the muscle insertion
                        plot3(scale*muscle.pos_attachments{end,4}(i,1),scale*muscle.pos_attachments{end,4}(i,2),scale*muscle.pos_attachments{end,4}(i,3),'r+')
                        %Plot the joint of interest
                        plot3(scale*obj.joint_obj{joint}.sim_position_profile(i,1),scale*obj.joint_obj{joint}.sim_position_profile(i,2),scale*obj.joint_obj{joint}.sim_position_profile(i,3),'b.')
                        %Plot the Ext/Flx axis
                        plot3(jointvecco1(:,1),jointvecco1(:,2),jointvecco1(:,3),'r')
                        %Plot the Add/Abd axis
                        plot3(jointvecco2(:,1),jointvecco2(:,2),jointvecco2(:,3),'g')
                        %Plot the ExR/InR axis
                        plot3(jointvecco3(:,1),jointvecco3(:,2),jointvecco3(:,3),'b')
                        %Plot the muscle projection on the plane of the axis of interest
                        plot3(pointprojection(:,1),pointprojection(:,2),pointprojection(:,3),'m--')
                        %Plot the hip
                        plot3(scale*obj.joint_obj{1}.sim_position_profile(i,1),scale*obj.joint_obj{1}.sim_position_profile(i,2),scale*obj.joint_obj{1}.sim_position_profile(i,3),'kp')
                        %Plot the femur
                        plot3(femur(:,1),femur(:,2),femur(:,3),'k--')
                        %Plot the knee
                        plot3(scale*obj.joint_obj{2}.sim_position_profile(i,1),scale*obj.joint_obj{2}.sim_position_profile(i,2),scale*obj.joint_obj{2}.sim_position_profile(i,3),'ks')
                        %Plot the tibia
                        plot3(tibia(:,1),tibia(:,2),tibia(:,3),'k-.')
                        %Plot the ankle
                        plot3(scale*obj.joint_obj{3}.sim_position_profile(i,1),scale*obj.joint_obj{3}.sim_position_profile(i,2),scale*obj.joint_obj{3}.sim_position_profile(i,3),'kd')
                        %Plot the foot
                        plot3(foot(:,1),foot(:,2),foot(:,3),'k:')
                        hold on
                        grid on
                        %Set figure properties
                        title([{[muscle.muscle_name(4:end),' ',axis(j,:),' about the ',obj.joint_obj{joint}.name(4:end),' joint']},...
                               {['Moment arm length: ',num2str(round(moment_arm_profile(count2,j+2),2)),' mm']},...
                               {[obj.joint_obj{joint}.name(4:end),' joint angle: ',num2str(obj.joint_obj{joint}.rec_angle_profile(i)*(180/pi))]}])
                        legend('Muscle','Moment arm','Muscle Origin','Muscle Insertion','Joint of Interest','Joint vector Ext/Flx','Joint vector Add/Abd','Joint vector ExR/InR',...
                            'Muscle Projection onto Joint Plane','Hip','Femur','Knee','Tibia','Ankle','Foot')
                        view([0 90])
%                         xlim([-20 35])
%                         ylim([-60 10])
                        xlim(limscale.*xlims)
                        ylim(limscale.*ylims)
                        zlim([zlims(1)-sign(zlims(1))*limscale*zlims(1) zlims(2)+sign(zlims(2))*limscale*zlims(2)])
                        xlabel('X')
                        ylabel('Y')
                        zlabel('Z')
                        %To avoid distorted 3D plots, set the aspect ratio of the plot relative to the lengths of the axes
                        axeslengths = [range(xlim);range(ylim);range(zlim)];
                        normedaxes = axeslengths/norm(axeslengths);
                        pbaspect(normedaxes)
                        hold off
                        pause(.05)
                    end
                end 
            end
        end
        %% Function: GRAVITY MOMENT ARM: Compute the gravity moment arm for a given body or joint
        function [moment_arm_profile] = compute_gravity_moment_arm(obj,inObj,joint,plotval)
            
            objName = inObj.name;
            if any(ismember(obj.joints,objName))
                jointInd = find(contains(obj.joints,objName),1,'first');
                if isempty(jointInd)
                    error('compute_gravity_moment_arm: Joint object not found in FullLeg.')
                elseif joint == jointInd
                    error('compute_gravity_moment_arm: Provided joint will not generate a moment arm about itself.')
                else
                    point_profile = obj.joint_obj{jointInd}.sim_position_profile';
                end
            elseif any(ismember(obj.bodies,objName))
                bodyInd = find(contains(obj.bodies,objName),1,'first');
                if isempty(bodyInd)
                    error('compute_gravity_moment_arm: Body object not found in FullLeg.')
                else
                    point_profile = move_point_through_walking(obj,bodyInd,obj.body_obj{bodyInd}.com);
                end
            else 
                error('compute_gravity_moment_arm: inObj must be a body or a joint.')
            end
           
            antiGrav = point_profile-[0 -.005 0]';
            axis_used = 1;
            
            if nargin < 4 || ~ismember(plotval,[0 1])
                plotval = 0;
            end

            scale = 1000;
            
            if ~ismember(joint,[1 2 3])
                joint = input('Joint specified incorrectly. hip = 1, knee = 2, ankle = 3\nInput joint number: \n');
            end

            [beg,ennd,~] = find_step_indices(obj);
            %Higher div, the longer the program takes.
            div = 500;
            xx = floor(linspace(beg,ennd,div));
            xx = linspace(1,length(obj.theta_motion),length(obj.theta_motion));
            xx = obj.sampling_vector;

            axis = ['Ext/Flx';'Abd/Add';'ExR/InR'];
            %Defining values for plot axis limits[moment_arm_profile] = compute_body_moment_arm(obj,obj.body_obj{2},1,1,0);
            if plotval == 1 
                jointmat = [obj.joint_obj{1}.sim_position_profile(beg:ennd,:);...
                                obj.joint_obj{2}.sim_position_profile(beg:ennd,:);...
                                obj.joint_obj{3}.sim_position_profile(beg:ennd,:)];
                toemat = obj.musc_obj{20,1}.pos_attachments{5,4};
                jointaxmat = (1/100).*[obj.joint_obj{joint}.uuw_joint(:,xx)';...
                    obj.joint_obj{joint}.uuw_joint2(:,xx)';...
                    obj.joint_obj{joint}.uuw_joint3(:,xx)'];
                
                pointsmax = max(point_profile,[],2)';
                pointsmin = min(point_profile,[],2)';
                jointmax = max(jointmat);
                jointmin = min(jointmat);
                toemax = max(toemat);
                toemin = min(toemat);
                jointaxmax = max(jointaxmat);
                jointaxmin = min(jointaxmat);

                posmax = scale*max([jointmax;...
                                     pointsmax;...
                                     toemax;...
                                     jointaxmax]);
                posmin = scale*min([jointmin;...
                                    pointsmin;...
                                    toemin;...
                                    jointaxmin]);
                limits =  [posmin' posmax'];
                xlims = limits(1,:);
                ylims = limits(2,:);
                zlims = limits(3,:);
                %Can change limscale to give more or less viewing space
                limscale = 1;
            end 
                moment_arm_profile = zeros(size(xx,2),5);
                moment_arm_profile(:,1) = obj.joint_obj{joint}.rec_angle_time(xx);
                moment_arm_profile(:,2) = obj.joint_obj{joint}.rec_angle_profile(xx)*(180/pi);

            for j = axis_used
                %f1 = figure(1);
                %for i=1:length(jointprofile)
                count2 = 0;
                for i=xx
                    count2 = count2 + 1;
                    % Joint axis in global coordinates
                    jointv = (scale/100).*[obj.joint_obj{joint}.uuw_joint(:,i),obj.joint_obj{joint}.uuw_joint2(:,i),obj.joint_obj{joint}.uuw_joint3(:,i)];
                    pointrel = zeros(2,3);
                    pointprojection = pointrel;
                    for k = 1:2
                        pointrel(1,:) = antiGrav(:,i) - obj.joint_obj{joint}.sim_position_profile(i,:)';
                        pointrel(2,:) = point_profile(:,i) - obj.joint_obj{joint}.sim_position_profile(i,:)';
                        pointprojection(k,:) = scale*(pointrel(k,:) - (dot(pointrel(k,:),jointv(:,j)')/norm(jointv(:,j)')^2)*jointv(:,j)')+scale*obj.joint_obj{joint}.sim_position_profile(i,:);
                    end
                    %Matrix of muscle projections perpendicular to the joint axis
                        fflatmat = pointprojection(2,:)-pointprojection(1,:);
                    %Storing the moment arms in a matrix
                        moment_arm_long = cross(fflatmat,jointv(:,j));
                        scaledjoint = scale*obj.joint_obj{joint}.sim_position_profile(i,:);
                        PA2 = [pointprojection(1,:);scaledjoint];
                        PB2 = [pointprojection(2,:);scaledjoint+moment_arm_long];
                        %lineINtersect3D gives the scaled vector of the moment arms for each segment
                        moment_arm = lineIntersect3D(obj,PA2,PB2);
                        %Storing the moment arms in a matrix
                        %A modifier to determine whether the moment arm is positive or negative
                        sig_momentarm = moment_arm-scaledjoint;
                        sig_muscleprojection = (pointprojection(2,:)-pointprojection(1,:));
                        signal2 = sign(dot(cross(sig_momentarm,sig_muscleprojection),jointv(:,j)'));
                        %moment_arm_length2(i-beg+1,j+1) = momentlen2;
                        moment_arm_profile(count2,j+2) = signal2*norm(sig_momentarm);
                    %% For plotting the muscle and joint in 3D as the muscle moves
                    %Unnecessary to plot every time step. This sets it to plot only after a certain number of steps
                    if mod(count2,20)==0
                        plotter = 1;
                    else
                        plotter = 0;
                    end
                    if plotval == 1 && plotter
                        if i==xx(20)
                            figure('name','legplot','Position',[-955   130   960   985]);
                            pause
                        end
                        %The muscle vector
                        for k = 1:2
                            pointvecco(k,:) = scale*point_profile(:,i);
                        end
                        jointvecco1 = [scaledjoint;scaledjoint+jointv(:,1)'];
                        jointvecco2 = [scaledjoint;scaledjoint+jointv(:,2)'];
                        jointvecco3 = [scaledjoint;scaledjoint+jointv(:,3)'];
                        femur = scale*[obj.joint_obj{1}.sim_position_profile(i,1),obj.joint_obj{1}.sim_position_profile(i,2),obj.joint_obj{1}.sim_position_profile(i,3);...
                            obj.joint_obj{2}.sim_position_profile(i,1),obj.joint_obj{2}.sim_position_profile(i,2),obj.joint_obj{2}.sim_position_profile(i,3)];
                        tibia = scale*[obj.joint_obj{2}.sim_position_profile(i,1),obj.joint_obj{2}.sim_position_profile(i,2),obj.joint_obj{2}.sim_position_profile(i,3);...
                            obj.joint_obj{3}.sim_position_profile(i,1),obj.joint_obj{3}.sim_position_profile(i,2),obj.joint_obj{3}.sim_position_profile(i,3)];
                        foot = scale*[obj.joint_obj{3}.sim_position_profile(i,1),obj.joint_obj{3}.sim_position_profile(i,2),obj.joint_obj{3}.sim_position_profile(i,3);...
                            obj.musc_obj{20, 1}.pos_attachments{5,4}(i,:)];
                                %flongco = [fflat(:,i-beg+1)'+scale*muscle.pos_attachments{1,4}(i,:);flong(:,i-beg+1)'+fflat(:,i-beg+1)'+scale*muscle.pos_attachments{1,4}(i,:)];
                                %fflatco = [scale*muscle.pos_attachments{1,4}(i,:);fflat(:,i-beg+1)'+scale*muscle.pos_attachments{1,4}(i,:)];
                                %mprojection = [originprojection';insprojection'];
                        %Plot the muscle
                        gcf;
                        plot3(pointvecco(:,1),pointvecco(:,2),pointvecco(:,3),'r', 'LineWidth', 2)
                        %Thicken the muscle
%                         set(findobj(gca, 'Type', 'Line', 'Linestyle', '-'), 'LineWidth', 2);
                        hold on
                        %Plot the moment arm average
                        singlesegmentmomentarm = [scaledjoint;moment_arm];
                        plot3(singlesegmentmomentarm(:,1),singlesegmentmomentarm(:,2),singlesegmentmomentarm(:,3),'m','LineWidth',2)                  
                        
                        %Plot the muscle origin
                        plot3(scale*antiGrav(1,i),scale*antiGrav(2,i),scale*antiGrav(3,i),'ro')
                        %Plot the muscle insertion
                        plot3(scale*point_profile(1,i),scale*point_profile(2,i),scale*point_profile(3,i),'r+')
                        %Plot the joint of interest
                        plot3(scale*obj.joint_obj{joint}.sim_position_profile(i,1),scale*obj.joint_obj{joint}.sim_position_profile(i,2),scale*obj.joint_obj{joint}.sim_position_profile(i,3),'b.')
                        %Plot the Ext/Flx axis
                        plot3(jointvecco1(:,1),jointvecco1(:,2),jointvecco1(:,3),'r')
                        %Plot the Add/Abd axis
                        plot3(jointvecco2(:,1),jointvecco2(:,2),jointvecco2(:,3),'g')
                        %Plot the ExR/InR axis
                        plot3(jointvecco3(:,1),jointvecco3(:,2),jointvecco3(:,3),'b')
                        %Plot the muscle projection on the plane of the axis of interest
                        plot3(pointprojection(:,1),pointprojection(:,2),pointprojection(:,3),'m--')
                        %Plot the hip
                        plot3(scale*obj.joint_obj{1}.sim_position_profile(i,1),scale*obj.joint_obj{1}.sim_position_profile(i,2),scale*obj.joint_obj{1}.sim_position_profile(i,3),'kp')
                        %Plot the femur
                        plot3(femur(:,1),femur(:,2),femur(:,3),'k--')
                        %Plot the knee
                        plot3(scale*obj.joint_obj{2}.sim_position_profile(i,1),scale*obj.joint_obj{2}.sim_position_profile(i,2),scale*obj.joint_obj{2}.sim_position_profile(i,3),'ks')
                        %Plot the tibia
                        plot3(tibia(:,1),tibia(:,2),tibia(:,3),'k-.')
                        %Plot the ankle
                        plot3(scale*obj.joint_obj{3}.sim_position_profile(i,1),scale*obj.joint_obj{3}.sim_position_profile(i,2),scale*obj.joint_obj{3}.sim_position_profile(i,3),'kd')
                        %Plot the foot
                        plot3(foot(:,1),foot(:,2),foot(:,3),'k:')
                        hold on
                        grid on
                        %Set figure properties
                        title([{[objName,' ',axis(j,:),' about the ',obj.joint_obj{joint}.name(4:end),' joint']},...
                               {['Moment arm length: ',num2str(round(moment_arm_profile(count2,j+2),2)),' mm']},...
                               {[obj.joint_obj{joint}.name(4:end),' joint angle: ',num2str(obj.joint_obj{joint}.rec_angle_profile(i)*(180/pi))]}],'Interpreter','None')
                        legend('Muscle','Moment arm','Muscle Origin','Muscle Insertion','Joint of Interest','Joint vector Ext/Flx','Joint vector Add/Abd','Joint vector ExR/InR',...
                            'Muscle Projection onto Joint Plane','Hip','Femur','Knee','Tibia','Ankle','Foot')
                        view([0 90])
%                         xlim([-20 35])
%                         ylim([-60 10])
                        xlim(limscale.*xlims)
                        ylim(limscale.*ylims)
                        zlim([zlims(1)-sign(zlims(1))*limscale*zlims(1) zlims(2)+sign(zlims(2))*limscale*zlims(2)])
                        xlabel('X')
                        ylabel('Y')
                        zlabel('Z')
                        %To avoid distorted 3D plots, set the aspect ratio of the plot relative to the lengths of the axes
                        axeslengths = [range(xlim);range(ylim);range(zlim)];
                        normedaxes = axeslengths/norm(axeslengths);
                        pbaspect(normedaxes)
                        hold off
                        pause(.05)
                    end
                end 
            end
        end
        %% Function: JOINT MOMENT ARM: Compute the relevant moment arms of all muscles articulating a designated joint for a specified configuration
        function [moment_output] = compute_joint_moment_arms(obj,joint,axis)
            %Joint axis 1: ExR/InR
            %Joint axis 2: Ext/Flx
            %Joint axis 3: Abd/Add
            relevant_muscles = [];
            num_muscles = length(obj.musc_obj);
            
            for i = 1:num_muscles
                attachment_bodies = cell2mat(obj.musc_obj{i}.pos_attachments(:,3));
                switch joint
                    case 1
                        if attachment_bodies(1) == 1
                            relevant_muscles = [relevant_muscles;i];
                        end
                    case 2
                        if attachment_bodies(end) == 3 || (attachment_bodies(end)==4 && attachment_bodies(1)==2)
                            relevant_muscles = [relevant_muscles;i];
                        end
                    case 3
                        if attachment_bodies(end) == 4
                            relevant_muscles = [relevant_muscles;i];
                        end
                end
            end
            
            for i=1:length(relevant_muscles)
                muscle_num = relevant_muscles(i);
                moment_arm_profile = compute_muscle_moment_arm(obj,obj.musc_obj{muscle_num},1,joint,0);
                moment_output(muscle_num,:) = moment_arm_profile(:,axis+2)';
            end
            
            moment_output(num_muscles+1,:) = moment_arm_profile(:,1)';
            moment_output(num_muscles+2,:) = moment_arm_profile(:,2)';
            
            plotter = 0;
            if plotter
                figure
                count = 1;
                CM = hsv(length(relevant_muscles));
                legendcell = {};
                for i=1:num_muscles
                    if moment_output(i,1) ~=0
                        plot(moment_output(i,:),'color',CM(count,:),'LineWidth',1.5)
                        legendcell{end+1} = obj.musc_obj{i}.muscle_name(4:end);
                        count = count +1;
                        hold on
                    end
                end
                    title(['Muscle moment arms about the ',obj.joint_obj{joint}.name(4:end)])
                    xlabel('Percent Stride')
                    ylabel('Moment arm length (mm)')
                    legend(legendcell,'Interpreter','none','Location','eastoutside')
                    set(gcf,'Position',[500 500 900 500])
                    saveas(gcf,['G:\My Drive\Rat\Optimizer\OutputFigures\moment_arms','\muscle_moment_arms','_',obj.joint_obj{joint}.name(4:end),'_',datestr(datetime('now'),'yyyymmdd'),'.png'])
            end
        end
        %% Function: MUSCLE PASSIVE TENSION Compute a single muscle's passive tension over a joint's motion
        function [T_find,T_out,T] = compute_muscle_passive_tension(obj,muscle,length_find)
            
            %Higher div, the longer the program takes.
            div = 100;
            count = 0;
            
            %Find the maximum change in muscle length (from lowest trough to highest peak)
             muscleprofile = muscle.muscle_length_profile;
%             [~,maxlocs] = findpeaks(muscleprofile);
%             [~,minlocs] = findpeaks(-muscleprofile);
%             %minvals = minvals*-1;
%             a3 = unique([maxlocs;minlocs]);
%             a4 = muscleprofile(a3);
%             for i = 1:length(a4)-1
%                 %signer(i,1) = sign(a4(i+1,1)-a4(i,1));
%                 range(i,1) = abs(a4(i+1,1)-a4(i,1));
%             end
%             [~,dd] = max(range);
%             dd = dd(1);
%             beg = a3(dd);
%             ennd = a3(dd+1);
            [beg,ennd,~] = obj.find_step_indices;
            xx = floor(linspace(beg,ennd,div));
            xx = linspace(1,length(obj.theta_motion),length(obj.theta_motion));
            xx = obj.sampling_vector;
            
            T = zeros(length(xx),1);
            T2 = zeros(length(xx),1);
            L_pass = muscleprofile(xx);
            
            ks = muscle.Kse;
            kp = muscle.Kpe;
            dt = obj.dt_motion;
            b = muscle.damping;
            Lr = muscle.RestingLength;
            
            musclelength = muscle.muscle_length_profile;
            musclevelocity = muscle.muscle_velocity_profile;
            
            % The solution will oscillate to infinity if abs(c)>1. To ensure this doesn't happen, we set the timestep very small and accept some error in the
            % passive tension solution.
            dt = .001;
            
            a = dt*ks/b;
            c = (1-a*(1+(kp/ks)));
            
            if abs(c) > 1
                error(['The passive tension solution for muscle ',muscle.muscle_name,' will oscillate to infinity.'])
            end
            
            for i = 1:length(musclevelocity)
                if i == 1
                    mV(i,1) = musclevelocity(i);
                    mL(i,1) = ks*max(0,musclelength(i)-Lr);
                    T(i,1) = mL(i,1);
                else
                    mV(i,1) = (a*b)*musclevelocity(i-1);
                    mL(i,1) = (a*kp)*max(0,musclelength(i-1)-Lr);
                    % Based on observations with the program, it appears as though Animatlab does not factor in velocity when calculating passive tension
                    T(i,1) = c*T(i-1,1) + mL(i,1);
                    %+mV(i,1)
                end
            end
            
            T(T<0) = 0;
            Tlen = min([length(muscleprofile) length(T)]);
            
             T_out(:,1) = muscleprofile(1:Tlen);
            T_out(:,2) = T(1:Tlen);
            
            %[~,dd] = findpeaks(-L_pass);
            tri = delaunayn(unique(T_out(:,1)));
            soughtindex = dsearchn(T_out(:,1),tri,length_find);
            T_find = T(soughtindex,1);
            
            return
            
            if (T(1)-T(end)) < 0
                    T_out(:,1) = L_pass;
                    T_out(:,2) = T;
            else

                avgblocks = 7;
                %converted to coefficients for the filtfilt function
                coeffblocks = ones(1,avgblocks)/avgblocks;

                Tsmooth = [T;flip(T);T;flip(T)];

                %For each joint, do the averaging of 10 data points many (originally set to
                %50) times

                for i=1:20
                    Tsmooth(:,1) = filtfilt(coeffblocks,1,Tsmooth(:,1));
                end
                Tsmooth = rescale(Tsmooth,min([T;flip(T);T;flip(T)]),max([T;flip(T);T;flip(T)]));
                Ts = Tsmooth(200:299,1);
                bb = T-Ts;
                cc = abs(gradient(bb));
                ccnorm = (cc-min(cc))/(max(cc)-min(cc));
                dd = gradient(ccnorm);
                [~,LOCS] = findpeaks(dd);
                gg = [Ts(1:LOCS(1));T(LOCS(1):end)];
                T_out(:,1) = L_pass;
                T_out(:,2) = gg(2:end);
                
%                 Tsmooth = rescale(Tsmooth,min([T;flip(T);T;flip(T)]),max([T;flip(T);T;flip(T)]));
%                 T_out(:,1) = L_pass;
%                 T_out(:,2) = Tsmooth(200:299,1);
            end
        end
        %% Function: lineInstersect3D: Find the closest point of intersection betweeen two vectors
        function [P_intersect,distances] = lineIntersect3D(obj,PA,PB)
            % Find intersection point of lines in 3D space, in the least squares sense.
            % PA :          Nx3-matrix containing starting point of N lines
            % PB :          Nx3-matrix containing end point of N lines
            % P_Intersect : Best intersection point of the N lines, in least squares sense.
            % distances   : Distances from intersection point to the input lines
            % Anders Eikenes, 2012
            %from: https://www.mathworks.com/matlabcentral/fileexchange/37192-intersection-point-of-lines-in-3d-space

            Si = PB - PA; %N lines described as vectors
            ni = Si ./ (sqrt(sum(Si.^2,2))*ones(1,3)); %Normalize vectors
            nx = ni(:,1); ny = ni(:,2); nz = ni(:,3);
            SXX = sum(nx.^2-1);
            SYY = sum(ny.^2-1);
            SZZ = sum(nz.^2-1);
            SXY = sum(nx.*ny);
            SXZ = sum(nx.*nz);
            SYZ = sum(ny.*nz);
            S = [SXX SXY SXZ;SXY SYY SYZ;SXZ SYZ SZZ];
            CX  = sum(PA(:,1).*(nx.^2-1) + PA(:,2).*(nx.*ny)  + PA(:,3).*(nx.*nz));
            CY  = sum(PA(:,1).*(nx.*ny)  + PA(:,2).*(ny.^2-1) + PA(:,3).*(ny.*nz));
            CZ  = sum(PA(:,1).*(nx.*nz)  + PA(:,2).*(ny.*nz)  + PA(:,3).*(nz.^2-1));
            C   = [CX;CY;CZ];
            P_intersect = (S\C)';

            if nargout>1
                N = size(PA,1);
                distances=zeros(N,1);
                for i=1:N %This is faster:
                ui=(P_intersect-PA(i,:))*Si(i,:)'/(Si(i,:)*Si(i,:)');
                distances(i)=norm(P_intersect-PA(i,:)-ui*Si(i,:));
                end
            end
        end
        %% Function: Joint Axis Profile
        function [add_axis,R_axis] = joint_axis_profile(obj,jointMotion,joint)
            add_axis = zeros(3,size(jointMotion,1));
            R_axis = add_axis;
            for i =1:length(jointMotion)
                a = axis_angle_rotation(obj,jointMotion(i,1),obj.joint_obj{1}.uu_joint);
                b = axis_angle_rotation(obj,jointMotion(i,2),obj.joint_obj{2}.uu_joint);
                c = axis_angle_rotation(obj,jointMotion(i,3),obj.joint_obj{3}.uu_joint);
                
                if contains(joint.name,'Knee')
                    add_axis(:,i) = 10*(obj.CR_bodies(:,:,1)*a*obj.CR_bodies(:,:,2)*b*obj.CR_bodies(:,:,3)*[1;0;0])+joint.sim_position_profile(i,:)'*1000;
                    R_axis(:,i) = 10*(obj.CR_bodies(:,:,1)*a*obj.CR_bodies(:,:,2)*b*obj.CR_bodies(:,:,3)*[0;1;0])+joint.sim_position_profile(i,:)'*1000;
                elseif contains(joint.name,'Ankle')
                    add_axis(:,i) = 10*(obj.CR_bodies(:,:,1)*a*obj.CR_bodies(:,:,2)*b*obj.CR_bodies(:,:,3)*c*obj.CR_bodies(:,:,4)*[1;0;0])+joint.sim_position_profile(i,:)'*1000;
                    R_axis(:,i) = 10*(obj.CR_bodies(:,:,1)*a*obj.CR_bodies(:,:,2)*b*obj.CR_bodies(:,:,3)*c*obj.CR_bodies(:,:,4)*[0;1;0])+joint.sim_position_profile(i,:)'*1000;
                elseif contains(joint.name,'Hip')
                    add_axis(:,i) = 10*(obj.CR_bodies(:,:,1)*a*obj.CR_bodies(:,:,2)*[1;0;0])+joint.sim_position_profile(i,:)'*1000;
                    R_axis(:,i) = 10*(obj.CR_bodies(:,:,1)*a*obj.CR_bodies(:,:,2)*[0;1;0])+joint.sim_position_profile(i,:)'*1000;
                else
                    keyboard
                end
                %joint_axis_w(:,i) = ankle_R_axis(:,i)*10+joint.sim_position_profile(i,:)'*1000;
            end
%             joint_axis_w(:,1:2000:20000);
%             jointMotion(1:2000:20000,1:3)';
        end
        %% Function: Find Muscle Index
        function number = find_muscle_index(obj,muscle)
            number = 0;
            if contains(muscle(end-2:end),'\n')
                if contains(muscle,'origin')
                    cutoff = strfind(muscle,'origin');
                else
                    if contains(muscle,'via')
                        cutoff = strfind(muscle,'via');
                    else
                        if contains(muscle,'ins')
                            cutoff = strfind(muscle,'ins');
                        else
                            keyboard
                        end
                    end
                end
                muscle = muscle(3:cutoff-1);    
            end
            for i = 1:length(obj.musc_obj)
                if contains(obj.musc_obj{i}.muscle_name,muscle)
                    number = i;
                    break
                end
            end
        end
        %% Function: Change Plot View with Input Angle
        function [az,el]=normalToAzimuthElevationDEG(obj,x,y,z,applyView)
            if nargin < 3
                applyView = 0;
            end
            if length(x)>1
                v         = x;
                if nargin > 1
                    applyView = y;
                end
                x=v(1);
                y=v(2);
                z=v(3);
            end
            if x==0 && y==0
                x =eps;
            end
            vNorm = sqrt(x^2+y^2+z^2);
            x=x/vNorm;
            y=y/vNorm;
            z=z/vNorm;
            az = (180/pi)*asin(x/sqrt(x^2+y^2));
            el = (180/pi)*asin(z);
            if applyView
                thereIsAnOpenFig = ~isempty(findall(0,'Type','Figure'));
                if thereIsAnOpenFig
                    axis equal
                    view([az,el]);
                    %the normal to the plane should now appear as a point
                    plot3([0,x],[0,y],[0,z],'linewidth',3)
                end
            end
        end
        %% Function: Compute the Spatial Manipulator Jacobian 
        function [Jac,p_foot] = compute_jac(obj,theta,thetaindex)
            %To clarify, this Jacobian is not the classical numerical jacobian
            %(differential of a function for each generalized coordinate). This
            %is the spatial manipulator Jacobian (see "A Mathematical Introduction to Robotic Manipulation" Murray, Li, and Sastry 1994)
            if length(theta) == obj.num_bodies - 1
                if size(theta,2) > size(theta,1)
                    theta = theta';
                end
                theta = [0;theta];
            elseif length(theta) == obj.num_bodies
                %do nothing
            elseif isempty(theta)
                theta = zeros(obj.num_bodies,1);
            else
                disp('Orientation vector is not proper length. Should be a nx1 vector where n=num_bodies (first element is time)')
                p_foot = -1;
                Jac = -1;
                return
            end
            
            %thetaindex = find(obj.theta_motion(:,2)==theta(3),1);

            r_N = zeros(3,obj.num_bodies);
            j_N = zeros(3,obj.num_bodies-1);
            
            for i = 2:obj.num_bodies
                %pos_bodies should be counted from 2 to end. Each column is that body's
                %position relative to the first body (which is 0s for the first).
                %Second body's position is given in the frame of the first.
                %r_N is the "World Position" in Animatlab of each body
                r_N(:,i) = obj.pos_bodies_w_profile{i,1}(thetaindex,:)';
                %Similarly, j_N is the "World Position" in Animatlab of each joint. 
                j_N(:,i-1) = obj.joint_obj{i-1,1}.sim_position_profile(thetaindex,:)';
            end
                
            %Axes of joint rotation
            omega = zeros(3,obj.num_bodies-1);

            %The jacobian matrix
            Jac = zeros(6,obj.num_bodies-1);

            %Use the local position of the toe.
            toe_pos = [20.733;-6.866;-1.867]/1000;

            a = axis_angle_rotation(obj,theta(2),obj.joint_obj{1}.uu_joint);
            b = axis_angle_rotation(obj,theta(3),obj.joint_obj{2}.uu_joint);
            c = axis_angle_rotation(obj,theta(4),obj.joint_obj{3}.uu_joint);

            obj.foot_vec = r_N(:,4) + obj.CR_bodies(:,:,1)*a*obj.CR_bodies(:,:,2)*b*obj.CR_bodies(:,:,3)*c*obj.CR_bodies(:,:,4)*toe_pos;
            obj.toe_pos_known = 1;
           
            %This adds the starting world position of the toe (not after
            %theta rotation)
            j_N(:,end+1) = obj.foot_vec;
            % Stores the world positions of the joint minus the hip joint
            obj.p_st = j_N - repmat(j_N(:,1),1,size(j_N,2));
            %Hip joint
            obj.leg_attach_pt = j_N(:,1);
            
            obj.vec_len = .002;
            
            for i=1:obj.num_bodies-1
                omega(:,i) = obj.joint_obj{i}.uuw_joint(:,thetaindex);
                if strcmp(obj.joint_obj{i}.type,'Hinge')
                    %This is the joint twist. Crossing the axis of rotation
                    %with the member being rotated provides the twist
                    %Joint twists make up the top half of the spatial
                    %manipulator jacobian                    
                    Jac(1:3,i) = -cross(omega(:,i),j_N(:,i));
                    Jac(4:6,i) = omega(:,i);
                elseif strcmp(obj.joint_obj{i}.type,'Prismatic')
                    Jac(1:3,i) = omega(:,i);
                else
                    disp('Unidentified joint type.')
                end
            end
            
            p_foot = j_N(:,end);
        end
        %% Function: Compute LOAD TORQUES
        function foot_force_joint_torques = compute_load_torques(obj,toplot)

                %Using a list of GRF at the foot, compute the maximum torques
                %that the joints should be able to apply.

                [beg,ennd,~] = find_step_indices(obj);

                %Midstance, ToeOff, MidSwing, ToeContact
                load([pwd,'\Data\MuirGRFData.mat'])
                m = size(obj.theta_motion(beg:ennd,:),1);
                n = length(VerticalNoSwing);
                animalmass = .30112;

                %Make kinematic swing data as long as stance data and put it all together
                lateralF = -interp1(1:n,LateralNoSwing,linspace(1,n,m))*animalmass;
                verticalF = interp1(1:n,VerticalNoSwing,linspace(1,n,m))*animalmass;
                propulsiveF = interp1(1:n,PropulsiveNoSwing,linspace(1,n,m))*animalmass;
                
                %Smoothing
                    %number of blocks to average around
                    avgblocks = floor(.01*size(lateralF,2));
                    %converted to coefficients for the filtfilt function
                    coeffblocks = ones(1,avgblocks)/avgblocks;

                    forcesmooth = [lateralF,lateralF,lateralF;verticalF,verticalF,verticalF;propulsiveF,propulsiveF,propulsiveF]';

                    %For each joint, do the averaging of 10 data points many (originally set to 50) times
%                     for j=1:3
%                         for i=1:20
%                             forcesmooth(:,j) = filtfilt(coeffblocks,1,forcesmooth(:,j));
%                         end
%                     end
                    
                    forcesmooth = smoothdata(forcesmooth,'gaussian',30);

                    sizer = floor(size(forcesmooth,1)/6);
                    forcesmooth = forcesmooth(3*sizer:5*sizer,:);
                    
                    %This needs to be rotated to match the Animatlab xyz environment. This is relatively simple at the moment, we just need to rotate planar
                    %forces by about 16 degrees clockwise along the global Z axis.
                    theta = 16.3*(pi/180);
                    rotmat = [cos(theta) -sin(theta) 0;...
                        sin(theta) cos(theta) 0;...
                        0 0 1];
                    forcesmooth = rotmat*forcesmooth';
                    
                    lateralF = forcesmooth(1,:);
                    verticalF = forcesmooth(2,:)';
                    propulsiveF = forcesmooth(3,:)';
                    
                    foot_force_joint_torques = zeros(3,size(lateralF,2));
                    foot_position = foot_force_joint_torques;
                for i=beg:ennd
                    pointer = i-beg+1;

                    current_config = obj.theta_motion(i,:)';

                    %Input the joint angles that we just computed, and
                    %insert them into the configuration of the leg.
                    [J,foot_position(:,pointer)] = obj.compute_jac(current_config,i);    

                    %Flip signs because this is the force the toe is pushing against the ground. This analysis is effector focused so motion of the
                    %effector, forces made by the effector, etc.
                    force_vec = [lateralF(pointer);verticalF(pointer);propulsiveF(pointer)];

                    %Calculate J'*F to find the joint torques. 
                    %J is in the spatial frame, so we don't need to modify it to the body frame as was done in the past
                    %For reference, consult Sastry 1994 p. 121 (Eq 3.60)
                    %foot_wrench_body = [force_vec(:,k);cross(foot_position(:,i),force_vec(:,k))];
                    foot_wrench = [force_vec;0;0;0];
                    current_torques = J'*foot_wrench;
                    
                    %The torques in each joint as generated only by the force of the ground on the foot
                    foot_force_joint_torques(:,pointer) = current_torques;
                end

                %Plotting
                if toplot
                    close all
                    figure
                        ylimms1(1) = 1.1*min(min(forcesmooth));
                        ylimms1(2) = 1.1*max(max(forcesmooth));
                        ylimms2(1) = 1.1*min(min(1000*foot_force_joint_torques));
                        ylimms2(2) = 1.1*max(max(1000*foot_force_joint_torques));
                        subA = subplot(3,1,1);
                            plot(100*linspace(0,1,size(obj.theta_motion(beg:ennd,:),1)),(180/pi)*obj.theta_motion(beg:ennd,:),'LineWidth',2)
                            title('Joint Angles for Stance and Swing')
                            ylabel('Angle (deg)')
                            xlabel('% Stride')
                            legend({'Hip','Knee','Ankle'},'Location','eastoutside')
                            grid on
                        subB = subplot(3,1,2);
                            plot(100*linspace(0,1,size(forcesmooth',1)),forcesmooth','LineWidth',2)
                            %hold on
                            %plot(linspace(0,1,size(bb,1)),bb)
                            title('Ground Reaction Forces')
                            xlabel('% Stride')
                            ylabel('Force (N)')
                            legend({'Lateral','Vertical','Propulsive'},'Location','eastoutside')
                            ylim(ylimms1)
                            grid on
                        subC = subplot(3,1,3);
                            plot(100*linspace(0,1,size(foot_force_joint_torques',1)),1000*foot_force_joint_torques','LineWidth',2)
                            title('Load Torques')
                            xlabel('% Stride')
                            ylabel('Torque (mN-m)')
                            legend({'Hip','Knee','Ankle'},'Location','eastoutside')
                            ylim(ylimms2)
                            grid on
%                         subD = subplot(4,1,4);
%                             plot(100*linspace(0,1,size(obj.torque_motion,1)),1000*obj.torque_motion,'LineWidth',2)
%                             title('Torque for Stance and Swing')
%                             ylabel('Torque (mN-m)')
%                             xlabel('% Stride')
%                             legend({'Hip','Knee','Ankle'},'Location','eastoutside')
%                             grid on
                        
                       
%                             set(gcf,'Position',[700 100 600 900])
                            set(gcf,'Position',[-1910 130 1920 985])
                            pause(.5)
                            subApos = get(subA,'Position');
                            subBpos = get(subB,'Position');
                            set(subB,'Position',[subApos(1) subBpos(2) subApos(3) subApos(4)]);
                            
                            
                            ax = findobj(gcf,'Type','Axes');
                            for i=1:length(ax)
                                set(ax(i).XLabel,'FontSize',14)
                                set(ax(i).YLabel,'FontSize',14)
                                set(ax(i).Title,'FontSize',16)
                            end
                         
                        %saveas(gcf,[pwd,'\OutputFigures\Images\compute_total_joint_torque\','total_load_torque_',datestr(datetime('now'),'yyyymmdd'),'.png'])
                end
        end
        %% Function: Compute Body Torques
        function [totaltorque] = compute_body_torques(obj)
            [beg,ennd,~] = find_step_indices(obj);
            %Higher div, the longer the program takes.
            div = 500;
            xx = floor(linspace(beg,ennd,div));
            xx = linspace(1,length(obj.theta_motion),length(obj.theta_motion));
            xx = obj.sampling_vector;
            
            gravDir = [0,-9.8,0]; % Gravity is along the Y-axis at -9.8 m/s^2

            for ii = 1:length(obj.bodies)
                % Body mass in kg
                bodyMass(ii) = obj.body_obj{ii}.mass./1000;
            end
                
            totaltorque = zeros(length(xx),length(obj.joints));
            for jj = 1:length(obj.joints)
                [bodyArm] = compute_gravity_moment_arm(obj,obj.body_obj{jj+1},jj,0);
                bodyTorque = (bodyArm(:,3)./1000).*bodyMass(jj+1).*9.8;
                if jj==3
                    nJointArm = zeros(size(bodyArm));
                else
                    [nJointArm] = compute_gravity_moment_arm(obj,obj.joint_obj{jj+1},jj,0);
                end
                nJointTorque = (nJointArm(:,3)./1000).*sum(bodyMass(jj+2:end)).*9.8;
                totaltorque(:,jj) = bodyTorque'+nJointTorque';
            end
        end
        %% Function: Compute PASSIVE JOINT TORQUE: Compute the passive joint torque for a joint over an entire walking cycle
        function [passive_joint_torque,passive_joint_motion,passive_muscle_torque,Tpass_profile] = compute_passive_joint_torque(obj)
            %Joint axis 1: ExR/InR
            %Joint axis 2: Ext/Flx
            %Joint axis 3: Abd/Add
            num_muscles = length(obj.musc_obj);
            
            [beg,ennd,~] = find_step_indices(obj);
            %Higher div, the longer the program takes.
            div = 500;
            xx = floor(linspace(beg,ennd,div));
            xx = linspace(1,length(obj.theta_motion),length(obj.theta_motion));
            xx = obj.sampling_vector;
            
            %load([pwd,'\Data\passiveTension.mat'],'passiveTension')
            %n = length(passiveTension);
            m = length(obj.sampling_vector);
            %passiveTension = interp1(1:n,passiveTension,linspace(1,n,m));

         for k = 1:3   
            [moment_output] = compute_joint_moment_arms(obj,k,1);
            Tpass_profile = zeros(size(moment_output));
            
            relevant_muscles{k} = find(sum(moment_output(1:num_muscles,:),2)~=0);
            
            %theta = (180/pi)*obj.theta_motion(:,k);
            passTorque = zeros(num_muscles,size(moment_output,2));
            %time = moment_output(39,:);
            for i = 1:length(relevant_muscles{k})
                muscle_num = relevant_muscles{k}(i);
                %muscle_length = obj.musc_obj{muscle_num}.muscle_length_profile(round(time./obj.dt_motion));
                [~,T_out] = compute_muscle_passive_tension(obj,obj.musc_obj{muscle_num},0);
                %tlocations = ceil(rescale(muscle_length(xx),1,100));
%                 for j = 1:size(moment_output,2)
%                     r = muscle_length(j);
%                     [~,tLoc] = min(abs(T_out(:,1)-r));
%                     tLoclog(1,j) = tLoc;
%                     Tpass_profile(muscle_num,j) = T_out(tLoc,2);
%                     %%Tpass_profile(muscle_num,j) = T_out(tlocations(j),2);
%                     passTorque(muscle_num,j) = Tpass_profile(muscle_num,j)*moment_output(muscle_num,j)/1000;
%                 end
%
%                   Tpass_profile(muscle_num,:) = T_out(xx,2)';
%                    passTorque(muscle_num,:) = T_out(xx,2)'.*moment_output(muscle_num,:)./1000;
                    pt = obj.musc_obj{muscle_num}.passive_tension;
                    n = length(pt);
                    pt = interp1(1:n,pt,linspace(1,n,m));
                    Tpass_profile(muscle_num,:) = pt;
                    passTorque(muscle_num,:) = pt.*moment_output(muscle_num,:)./1000;
            end
            
            passive_joint_motion(k,:) = moment_output(40,:);
            passive_joint_torque(k,:) = sum(passTorque);
            passive_muscle_torque(:,:,k) = passTorque;
         end
            plotter = 0;
            if plotter
                figure
                plot(passive_joint_torque')
                title('Passive Joint Torque')
                xlabel('Percent Stride')
                ylabel('Torque (Nm)')
                legend({'Hip','Knee','Ankle'})
                for k = 1:3
                    figure
                    count = 1;
                    CM = hsv(size(relevant_muscles{k},1));
                    legendcell = {};
                    for i = 1:length(relevant_muscles{k})
                        muscle_num = relevant_muscles{k}(i);
                        plot(1000*passive_muscle_torque(muscle_num,:,k),'color',CM(count,:),'LineWidth',1.5)
                        legendcell{end+1} = obj.musc_obj{muscle_num}.muscle_name(4:end);
                        count = count + 1;
                        %title(obj.musc_obj{i}.muscle_name,'Interpreter','none')
                        hold on
                    end
                    title(['Passive Muscle Torque of the ',obj.joint_obj{k}.name(4:end)])
                    xlabel('Percent Stride')
                    ylabel('Torque (mN-m)')
                    legend(legendcell,'Interpreter','none','Location','eastoutside')
                    set(gcf,'Position',[500 500 900 500])
                    saveas(gcf,['G:\My Drive\Rat\Optimizer\OutputFigures\passive_torque','_',obj.joint_obj{k}.name(4:end),'_',datestr(datetime('now'),'yyyymmdd'),'.png'])
                end
            end
        end
        %% Function: Compute Total Joint Torque (Passive)
        function  joint_torques = compute_total_joint_torque(obj,toplot)
            load_torque = compute_load_torques(obj,0);
            [passive_joint_torque,passive_joint_motion] = compute_passive_joint_torque(obj);
            n = size(load_torque,2);
            m = length(passive_joint_motion);
            load_torque = interp1(1:n,load_torque',linspace(1,n,m));
            [body_torque] = compute_body_torques(obj);
            
            % Modify this statement to account for different torque types. With the free hanging leg, load torques aren't included.
            joint_torques = passive_joint_torque'+body_torque;
            
            %Plotting
            if toplot
                scale = 1.2;
                aa = figure;
                subplot(5,1,1)
                    plot(linspace(0,100,size(passive_joint_motion',1)),passive_joint_motion','LineWidth',2)
                    title('Joint Motion')
                    ylabel('Angle (deg)')
                    xlabel('Percent Stride')
                    legend('Hip','Knee','Ankle','Location','eastoutside')
                    ylimms1(1) = scale*min(min(passive_joint_motion));
                    ylimms1(2) = scale*max(max(passive_joint_motion));
                    ylim(ylimms1);
                subplot(5,1,2)
                    plot(linspace(0,100,length(passive_joint_torque)),1000*passive_joint_torque','LineWidth',2)
                    title('Passive Torque (Muscle)')
                    ylabel('Torque (mN-m)')
                    xlabel('Percent Stride')
                    legend('Hip','Knee','Ankle','Location','eastoutside')
                    ylimms2(1) = 1000*scale*min(passive_joint_torque,[],'all');
                    ylimms2(2) = 1000*scale*max(passive_joint_torque,[],'all');
                    ylim(ylimms2);
                subplot(5,1,3)
                    plot(linspace(0,100,length(load_torque)),1000*load_torque,'LineWidth',2)
                    title('Passive Torque (Load)')
                    ylabel('Torque (mN-m)')
                    xlabel('Percent Stride')
                    legend('Hip','Knee','Ankle','Location','eastoutside')
                    ylimms3(1) = 1000*scale*min(load_torque,[],'all');
                    ylimms3(2) = 1000*scale*max(load_torque,[],'all');
                    ylim(ylimms3);
                subplot(5,1,4)
                    plot(linspace(0,100,length(body_torque)),1000.*body_torque,'LineWidth',2)
                    title('Body Torques')
                    ylabel('Torque (mN-m)')
                    xlabel('Percent Stride')
                    legend('Hip','Knee','Ankle','Location','eastoutside')
                subplot(5,1,5)
                    x4 = linspace(0,100,size(joint_torques,1));
                    xfill = [x4, fliplr(x4)];
                    inBetween1 = 1000*[joint_torques(:,1)', fliplr(passive_joint_torque(1,:))];
                    inBetween2 = 1000*[joint_torques(:,2)', fliplr(passive_joint_torque(2,:))];
                    inBetween3 = 1000*[joint_torques(:,3)', fliplr(passive_joint_torque(3,:))];
                    A = fill(xfill, inBetween1, 'r','facealpha',.1,'edgealpha',0);
                    hold on
                    B = fill(xfill, inBetween2, 'g','facealpha',.1,'edgealpha',0);
                    C = fill(xfill, inBetween3, 'b','facealpha',.1,'edgealpha',0);
                    a = plot(x4,1000*joint_torques(:,1),'LineWidth',2);
                    b = plot(x4,1000*joint_torques(:,2),'LineWidth',2);
                    c = plot(x4,1000*joint_torques(:,3),'LineWidth',2);
                    set(A,'FaceColor',get(a,'Color'))
                    set(B,'FaceColor',get(b,'Color'))
                    set(C,'FaceColor',get(c,'Color'))
                    title('Total Passive Joint Torque')
                    ylabel('Torque (mN-m)')
                    xlabel('Percent Stride')
                    legend([a b c],{'Hip','Knee','Ankle'},'Location','eastoutside');
%                 set(gcf,'Position',[700 20 800 950])
                set(gcf,'Position',[392,294,1130,707])
                saveas(aa,[pwd,'\OutputFigures\total_passive_torque_',datestr(datetime('now'),'yyyymmdd'),'.png'])
            end
        end
        %% Function: NEXT ADVENTURE Compute Muscle Force for Motion (outputs Musc_Tension, Musc_Length, Musc_vel, MN_act, etc)
        function [musc_tension, musc_length, musc_velocity, musc_act, MN_act, musc_tension_passive] = compute_musc_force_for_motion(obj)
            %Using data built into the object, compute the muscle forces
            %necessary to generate known torques at known positions and
            %velocities.
            if isempty(obj.torque_motion) || isempty(obj.theta_motion) || isempty(obj.theta_dot_motion)
                disp('call Joint.set_torque_and_kinematic_data()')
            end
            muscle_num = 1;
            joint_num = 1;
            axis_num = 1;
            muscle = obj.musc_obj{muscle_num};
            %The load on the joint is the opposite of the applied torque
%             torque_load = -obj.torque_motion;
            net_torque = zeros(size(-obj.torque_motion));

            
            %Initialize our outputs
            if isempty(obj.theta_motion)
                T_ext = zeros(obj.num_pts,1);
                T_flx = zeros(obj.num_pts,1);
                obj.L_ext = zeros(obj.num_pts,1);
                obj.L_flx = zeros(obj.num_pts,1);
                num_loops = obj.num_pts;
                obj.theta_motion = obj.theta;
            else
                num_loops = length(obj.theta_motion);
                T_musc = zeros(num_loops,1);
%                 T_flx = zeros(num_loops,1);
                musc_act = zeros(num_loops,2);
                musc_group_act = zeros(num_loops,2);
                MN_act = -.100 + zeros(num_loops,2);
                MN_group_act = -.1 + zeros(num_loops,2);
                musc_tension_passive = MN_act;
            end
            
%             [ext_data,flx_data] = obj.compute_musc_velocities_for_motion();
%             extensor_length = ext_data{1};
%             extensor_velocity = ext_data{2};
%             flexor_length = flx_data{1};
%             flexor_velocity = flx_data{2};
            muscle_length = muscle.muscle_length_profile;
            muscle_velocity = muscle.muscle_velocity_profile;
            
            for i=1:num_loops
                
                %We will now solve for the muscle forces required to
                %counter the maximum torque with the maximum joint
                %stiffness. 
%                 bnds = [0,max(obj.Amp_ext,obj.Amp_flx)];
                bnds = [0,max(muscle.max_force)];

%                 %Explicit solution
%                 %Flexor passive force
%                 if i > 1
%                     T_passive_flx = (obj.c_flx*T_flx(i-1)/obj.dt_motion/obj.kse_flx + obj.kpe_flx*max(0,flexor_length(i)-obj.l_flx_rest) + obj.c_flx*flexor_velocity(i))/(1+(obj.kpe_flx + obj.c_flx/obj.dt_motion)/obj.kse_flx);
%                 else
%                     T_passive_flx = (obj.kpe_flx*max(0,flexor_length(i)-obj.l_flx_rest) + obj.c_flx*flexor_velocity(i))/(1+(obj.kpe_flx + obj.c_flx)/obj.kse_flx);
%                 end
%                 %Extensor passive force
%                 if i > 1
%                     T_passive_ext = (obj.c_ext*T_ext(i-1)/obj.dt_motion/obj.kse_ext + obj.kpe_ext*max(0,extensor_length(i)-obj.l_ext_rest) + obj.c_ext*extensor_velocity(i))/(1+(obj.kpe_ext + obj.c_ext/obj.dt_motion)/obj.kse_ext);
%                 else
%                     T_passive_ext = (obj.kpe_ext*max(0,extensor_length(i)-obj.l_ext_rest) + obj.c_ext*extensor_velocity(i))/(1+(obj.kpe_ext + obj.c_ext)/obj.kse_ext);
%                 end
                theta = (180/pi)*obj.theta_motion(:,joint_num);
                [moment_output] = compute_joint_moment_arms(obj,joint_num,axis_num);
                [~,T_out] = compute_muscle_passive_tension(obj,muscle,0);
                for j = 1:size(moment_output,2)
                    angle = moment_output(40,j);
                    soughtindex = find(theta==angle,1,'first');
                    r = muscle_length(soughtindex,1);
                    [~,tLoc] = min(abs(T_out(:,1)-r));
                    Tpass_profile(1,j) = T_out(tLoc,2);
                    passTorque(1,j) = Tpass_profile(1,j)*moment_output(muscle_num,j)/1000;
                end
                
               
%                 ext_passive_muscle_torque = obj.net_joint_torque(T_passive_ext,'ext',obj.theta_motion(i),0,0);
%                 flx_passive_muscle_torque = obj.net_joint_torque(T_passive_flx,'flx',obj.theta_motion(i),0,0);
                muscpasstorque = passTorque;
                
                net_torque(i) = -obj.torque_motion(i) + muscpasstorque;

%                 if net_torque(i) > 0
%                     %The external load is positive, so the joint must be
%                     %supplying a negative torque to cancel it.
%                     active_musc_string = 'flx';
%                 else
%                     active_musc_string = 'ext';
%                 end

                %Now, find the muscle tension that will generate the
                %desired torque by finding the zero of this function.
                f_to_opt = @(x)obj.net_joint_torque(x,active_musc_string,obj.theta_motion(i),net_torque(i),0);
                if isnan(f_to_opt(bnds(1))) || isnan(f_to_opt(bnds(2)))
                    disp('f isnan')
                    keyboard
                end
                T_active = bisect(f_to_opt,bnds(1),bnds(2),1e-6,1e-6,1000);
                
                if net_torque(i) > 0
                    %The flexor is applying the net force
                    T_flx(i) = T_active + T_passive_flx;
                    T_ext(i) = T_passive_ext;
                else
                    %The extensor is applying the net force
                    T_flx(i) = T_passive_flx;
                    T_ext(i) = T_active + T_passive_ext;
                end
                    
                musc_tension_passive(i,:) = [T_passive_ext,T_passive_flx];
                
            end

            musc_tension = [T_ext,T_flx];
            T_ext_dot = [0;diff(T_ext)/obj.dt_motion];
            T_flx_dot = [0;diff(T_flx)/obj.dt_motion];
%             keyboard
            musc_length = [extensor_length,flexor_length];
            musc_velocity = [extensor_velocity,flexor_velocity];
            
            if strcmp(obj.joint_name,'LH_HipZ')
                muscle_num = [10;8];
            elseif strcmp(obj.joint_name,'LH_Knee')
                muscle_num = [3;6];
            elseif strcmp(obj.joint_name,'LH_AnkleZ')
                muscle_num = [9;2];
            end
            
            for i=1:num_loops
                if net_torque(i) > 0  
                    %flexor is providing force
                    
                    %Solve for the activation of the muscle by rearranging
                    %dT/dt
                    Act_flx = max(0,((obj.c_flx/obj.kse_flx)*T_flx_dot(i) + (obj.kse_flx+obj.kpe_flx)*T_flx(i))/obj.kse_flx - obj.kpe_flx*max(0,(flexor_length(i)-obj.l_flx_rest)) - obj.c_flx*flexor_velocity(i));  
%                   Act_flx = max(0,(obj.c_flx*T_flx_dot(i) + (obj.kse_flx+obj.kpe_flx)*T_flx(i))/obj.kse_flx - obj.kpe_flx*max(0,(flexor_length(i)-obj.l_flx_rest)) - obj.c_flx*flexor_velocity(i));
%                   Act_flx = max(0,(obj.c_flx*T_flx_dot(i) + (obj.kse_flx+obj.kpe_flx)*T_flx(i))/obj.kse_flx - obj.kpe_flx*(flexor_length(i)-obj.l_flx_rest) - obj.c_flx*flexor_velocity(i)); 
                    musc_act(i,2) = Act_flx;
                    musc_group_act(i,2) = Act_flx/muscle_num(2);
                    flx_sigmoid = @(V) (obj.Amp_flx./(1+exp(obj.steep_flx*(obj.xoff_flx-V))) + obj.yoff_flx)*max(0,(1-(flexor_length(i)-obj.l_flx_rest)^2/(obj.l_width_flx)^2));
                    f_act_flx = @(V) Act_flx - (0*(V <= -.100) + obj.Amp_flx*(V >= 0) + (V>-.100 && V < 0)*flx_sigmoid(V));
                    MN_act(i,2) = bisect(f_act_flx,-.100,0,1e-6,1e-6,1000);
                    MN_group_act(i,2) = ((MN_act(i,2)+.1)/muscle_num(2))-.1;
                else
                    Act_ext = max(0,((obj.c_ext/obj.kse_ext)*T_ext_dot(i) + (obj.kse_ext+obj.kpe_ext)*T_ext(i))/obj.kse_ext - obj.kpe_ext*max(0,(extensor_length(i)-obj.l_ext_rest)) - obj.c_ext*extensor_velocity(i));
%                   Act_ext = max(0,(obj.c_ext*T_ext_dot(i) + (obj.kse_ext+obj.kpe_ext)*T_ext(i))/obj.kse_ext - obj.kpe_ext*max(0,(extensor_length(i)-obj.l_ext_rest)) - obj.c_ext*extensor_velocity(i));
%                   Act_ext = max(0,(obj.c_ext*T_ext_dot(i) + (obj.kse_ext+obj.kpe_ext)*T_ext(i))/obj.kse_ext - obj.kpe_ext*(extensor_length(i)-obj.l_ext_rest) - obj.c_ext*extensor_velocity(i));
                    musc_act(i,1) = Act_ext;
                    musc_group_act(i,1) = Act_ext/muscle_num(1);
                    %Calculate the tension generated by a stimulus. This is
                    %the Animatlab "Stimulus-Tension" curve
                    ext_sigmoid = @(V) (obj.Amp_ext./(1+exp(obj.steep_ext*(obj.xoff_ext-V))) + obj.yoff_ext)*max(0,(1-(extensor_length(i)-obj.l_ext_rest)^2/(obj.l_width_ext)^2));
                    f_act_ext = @(V) Act_ext - (0*(V <= -.100) + obj.Amp_ext*(V >= 0) + (V>-.100 && V < 0)*ext_sigmoid(V));
                    MN_act(i,1) = bisect(f_act_ext,-.100,0,1e-6,1e-6,1000);
                    MN_group_act(i,1) = ((MN_act(i,1)+.1)/muscle_num(1))-.1;
                end
                
            end
            %%
            if 0
                xx = linspace(0,100,num_loops/3)';
                width = 8;
                fontsize = 50;
                fontsizeax = 45;
                figW = 2000;
                %figH = figW/1.75;
                figH = 1050;
                lshift = 10;
                fig1 = figure('pos',[0 0 figW figH]);
                title(strcat(strrep(obj.joint_name,'LH_',' '),' MN Group Activation'),'fontsize',fontsize)
                %view([-38.4 42]);
                ylabel('Percent Stride (%)','fontsize',fontsize)
                view([90 0]);
                %grid off
                hold on
                for ii = 1:muscle_num(1)
                    zz = ii*ones(size(xx));
    %                qq = area(xx,ii*musc_group_act(:,1),'FaceColor',[0 0 0]);
    %                plot3(zz,xx,qq)
                    %plot3(zz,xx,ii*musc_group_act(:,1),'b');
                    %%Exponential
                    %plot3(zz,xx,(ii*(MN_group_act(:,1)+.1)-.1),'Color',[1-(ii*1/muscle_num(1))^2 1-(ii*1/muscle_num(1))^2 1])
                    %%Linear
                    kk = plot3(zz,xx,1000*(ii*(MN_group_act(1:100,1)+.1)-.1),'Color',[1-ii*1/muscle_num(1) 1-ii*1/muscle_num(1) 1]);
                end

                for ii = 1:muscle_num(2)
                    zz = ii*ones(size(xx));
                    %plot3(zz,xx,ii*musc_group_act(:,2),'r')
                    %%Exponential
                    %plot3(zz,xx,(ii*(MN_group_act(:,2)+.1)-.1),'Color',[1 1-(ii*1/muscle_num(2))^2 1-(ii*1/muscle_num(2))^2])
                    kk = plot3(zz,xx,1000*(ii*(MN_group_act(1:100,2)+.1)-.1),'Color',[1 1-ii*1/muscle_num(2) 1-ii*1/muscle_num(2)]);
                end
                    set(findall(gca, 'Type', 'Line'),'LineWidth',width);
                    set(gca,'FontSize',fontsizeax)
                    zlh = zlabel('MN Activation (mV)','fontsize',fontsize);
%                    set(get(gca,'zlabel'),'VerticalAlignment','middle')
%                     zlabpos = get(zlh,'Position');
%                     zlabpos(2) = zlabpos(2)-lshift;
%                     set(zlh,'Position',zlabpos);

                figure('pos',[0 0 figW figH])
                title(strcat(strrep(obj.joint_name,'LH_',' '),' Muscle Group Tension'),'fontsize',fontsize)
                %view([-38.4 42]);
                ylabel('Percent Stride (%)','fontsize',fontsize)
                view([90 0]);
                %grid off
                hold on
                for ii = 1:muscle_num(1)
                    zz = ii*ones(size(xx));
    %                 qq = area(xx,ii*musc_group_act(:,1),'FaceColor',[0 0 0]);
    %                 plot3(zz,xx,qq)
                    %plot3(zz,xx,ii*musc_group_act(:,1),'b');
                    %%Exponential
                    %plot3(zz,xx,(ii*musc_group_act(:,1)),'Color',[1-(ii*1/muscle_num(1))^2 1-(ii*1/muscle_num(1))^2 1])
                    %%Linear
                    pp = plot3(zz,xx,(ii*musc_group_act(1:100,1)),'Color',[1-ii*1/muscle_num(1) 1-ii*1/muscle_num(1) 1]);

                end

                for ii = 1:muscle_num(2)
                    zz = ii*ones(size(xx));
                    %plot3(zz,xx,ii*musc_group_act(:,2),'r')
                    %%Exponential
                    %plot3(zz,xx,(ii*musc_group_act(:,2)),'Color',[1 1-(ii*1/muscle_num(2))^2 1-(ii*1/muscle_num(2))^2])
                    %%Linear
                    pp = plot3(zz,xx,(ii*musc_group_act(1:100,2)),'Color',[1 1-ii*1/muscle_num(2) 1-ii*1/muscle_num(2)]);
                end
                    zlh = zlabel('Muscle Activation (N)','fontsize',fontsize);
                    set(findall(gca, 'Type', 'Line'),'LineWidth',width);
                    set(gca,'FontSize',fontsizeax)
%                    set(get(gca,'zlabel'),'VerticalAlignment','middle')
%                     zlabpos = get(zlh,'Position');
%                     zlabpos(2) = zlabpos(2)-lshift;
%                     set(zlh,'Position',zlabpos);
                
                uu = figure('pos',[0 0 figW figH]);
%                 axes1 = axes('Parent',uu,...
%                     'Position',[0.251423921887714 0.231477657085432 0.653576078112285 0.638281796312414]);
                hold on
                for ii = 1:size(MN_act,2)
                    ooo = plot(xx,1000*MN_act(1:100,ii));
                    if ii == 1
                        set(ooo,'Color',[0 0 1]);
                    else
                        set(ooo,'Color',[1 0 0]);
                    end
                end
                title('Hip Generalized MN Activation','fontsize',fontsize);
                xlabel('Percent Stride (%)','fontsize',fontsize);
                ylh = ylabel('MN Activation (mV)','fontsize',fontsize);
                set(gca,'FontSize',fontsizeax);
%                set(get(gca,'ylabel'),'VerticalAlignment','middle');
%                 ylabpos = get(ylh,'Position');
%                 ylabpos(1) = ylabpos(1)-lshift;
%                 set(ylh,'Position',ylabpos);
                legend({'Extension','Flexion'},'FontSize',fontsize);
                set(findall(gca, 'Type', 'Line'),'LineWidth',width);
                
                ll = figure('pos',[0 0 figW figH]);
%                 axes1 = axes('Parent',ll,...
%                     'Position',[0.209260908281389 0.231477657085432 0.695739091718609 0.638281796312414]);
                hold on
                for ii = 1:size(musc_act,2)
                    bbb = plot(xx,musc_act(1:100,ii));
                    if ii == 1
                        set(bbb,'Color',[0 0 1]);
                    else
                        set(bbb,'Color',[1 0 0]);
                    end
                end
                title('Hip Generalized Muscle Activation','fontsize',fontsize);
                xlabel('Percent Stride (%)','fontsize',fontsize);
                ylh = ylabel('Muscle Activation (N)','fontsize',fontsize);
                set(gca,'FontSize',fontsizeax);
%                set(get(gca,'ylabel'),'VerticalAlignment','middle')
%                 ylabpos = get(ylh,'Position');
%                 ylabpos(1) = ylabpos(1)-lshift;
%                 set(ylh,'Position',ylabpos);
                legend({'Extension','Flexion'},'FontSize',fontsize);
                set(findall(gca, 'Type', 'Line'),'LineWidth',width);
                
                %fname = 'G:\My Drive\Rat\Optimizer\OutputFiguresGroup';
                fname = [pwd,'\OutputFigures\Images'];
                saveas(kk, fullfile(fname,strcat(obj.joint_name,' MN Group Activation')), 'png');
                saveas(pp, fullfile(fname,strcat(obj.joint_name,' Muscle Group Tension')), 'png');
                saveas(uu, fullfile(fname,strcat(obj.joint_name,' Generalized MN Activation')), 'png');
                saveas(ll, fullfile(fname,strcat(obj.joint_name,' Generalized Muscle Activation')), 'png');
                close all
            end
                %keyboard
            %%
            %Now that we have muscle lengths, velocities, and forces for
            %every time step in our data, we can calculate what our
            %activation and passive parameters should look like
            for i=1:obj.num_pts
                %We will now solve for the muscle forces required to
                %counter the maximum torque with the maximum joint
                %stiffness. This is a smooth function, so we will use our
                %quasi-newton optimizer. We will constrain it such that
                %muscle forces are greater than 0. A large upper bound will
                %be used since none is not an option. 
                bnds = [0,1e6];

                %First, assume the torque is positive. This will reveal the
                %maximum flexor force.
                f_to_opt = @(x)obj.net_joint_torque(x,'ext',obj.theta(i),obj.torque_motion(i),1);
                [T,ext_res] = con_opti(f_to_opt,[1],{[],[]},bnds,[],[],[1000,1e-12,1e-12],'min','bfgs',[],'linesearch',{'backtrack',[1e-4,.9,.5,500],[],[]},[],0);
                T_ext_ext(i) = T(1);
                T_flx_ext(i) = 0;
                
                obj.L_ext(i) = obj.l_ext_current;
                
                f_to_opt = @(x)obj.net_joint_torque(x,'flx',obj.theta(i),-obj.torque_motion(i),1);
                [T,flx_res] = con_opti(f_to_opt,[1],{[],[]},bnds,[],[],[1000,1e-12,1e-12],'min','bfgs',[],'linesearch',{'backtrack',[1e-4,.9,.5,500],[],[]},[],0);
                T_ext_flx(i) = 0;
                T_flx_flx(i) = T(1);

                obj.L_flx(i) = obj.l_flx_current;
                
                force_residuals(i,:) = [ext_res,flx_res];
            end
            
            %Stores the minimum lengths calculated into the properties
            obj.min_l_ext = min(obj.L_ext);
            obj.max_l_ext = max(obj.L_ext);
            obj.min_l_flx = min(obj.L_flx);
            obj.max_l_flx = max(obj.L_flx);
            
            obj.MN_ext = MN_act(:,1);
            obj.MN_flx = MN_act(:,2);
        end
        %% Function: Calculate the Muscle Tension Using Joint Angle and Applied Torque
        function Tension = calc_musc_tension(obj,angle,torque)
            %Inputs are the joint angle and the applied torque
            %Output is the tension required to counteract the input torque
            
%             if torque<0
%                 active_musc_string = 'ext';
%             else
%                 active_musc_string = 'flx';
%             end
            
            bnds = [0,1000];
            
            f_to_opt = @(x)obj.net_joint_torque(x,active_musc_string,angle,torque,0);
            if isnan(f_to_opt(bnds(1))) || isnan(f_to_opt(bnds(2)))
                disp('f isnan')
                %keyboard
            end
            Tension = bisect(f_to_opt,bnds(1),bnds(2),1e-6,1e-6,1000);
        end
        %% Function: Calculate the Residual Torque Given Input Forces, Desired EquilPos, and External Torque
        function objective = net_joint_torque(obj,joint,musc_force,config_des,torque_load,norm_net_torque)
            %This function calculates the residual torque on a joint for
            %the given input forces, desired equilibrium position, and
            %external torque. This can be used for a variety of purposes.
            
            %For a given angle, the forces required to counter an applied
            %torque can be found by seeking output = 0. 
            %For given forces, the equilibrium position can be found again 
            %by seeking output = 0. The forces are constrained to provide 
            %the maximum stiffness the joint should be able to generate.
            
            T = musc_force;
%             if any(size(T) ~= [length(muscle),1])
%                 disp('Invalid force input for Joint.net_joint_torque()')
%             end
            
            musc_torque = 0;
            [moment_output] = compute_joint_moment_arms(obj,joint,2);
            relevant_muscle = moment_output(:,1)~=0;
                
            musc_torque = musc_torque + obj.u_joint'*cross(r_rotated,T*u);

            %Find the net torque 
            net_torque = obj.u_joint'*(obj.u_joint*torque_load) + musc_torque;
            if norm_net_torque
                objective = net_torque^2;
            else
                objective = net_torque;
            end
            
        end
        %% Function: Compute the Muscle Forces for Postural Stability 
        function output = compute_musc_force_for_PS(obj, load, set_musc_properties)
            if isempty(load)
                torque_load = obj.max_torque;
            else
                torque_load = abs(load);
            end
            
            T_ext_ext = zeros(obj.num_pts,1);
            T_flx_ext = zeros(obj.num_pts,1);
            T_ext_flx = zeros(obj.num_pts,1);
            T_flx_flx = zeros(obj.num_pts,1);
            obj.L_ext = zeros(obj.num_pts,1);
            obj.L_flx = zeros(obj.num_pts,1);
            force_residuals = zeros(obj.num_pts,2);
            
            for i=1:obj.num_pts
                %We will now solve for the muscle forces required to
                %counter the maximum torque with the maximum joint
                %stiffness. This is a smooth function, so we will use our
                %quasi-newton optimizer. We will constrain it such that
                %muscle forces are greater than 0. A large upper bound will
                %be used since none is not an option. 
                bnds = [0,1e6];

                %First, assume the torque is positive. This will reveal the
                %maximum flexor force.
                f_to_opt = @(x)obj.net_joint_torque(x,'ext',obj.theta(i),-torque_load,1);
                [T,ext_res] = con_opti(f_to_opt,[1],{[],[]},bnds,[],[],[1000,1e-12,1e-12],'min','bfgs',[],'linesearch',{'backtrack',[1e-4,.9,.5,500],[],[]},[],0);
                T_ext_ext(i) = T(1);
                T_flx_ext(i) = 0;
                
                obj.L_ext(i) = obj.l_ext_current;
                
                f_to_opt = @(x)obj.net_joint_torque(x,'flx',obj.theta(i),torque_load,1);
                [T,flx_res] = con_opti(f_to_opt,[1],{[],[]},bnds,[],[],[1000,1e-12,1e-12],'min','bfgs',[],'linesearch',{'backtrack',[1e-4,.9,.5,500],[],[]},[],0);
                T_ext_flx(i) = 0;
                T_flx_flx(i) = T(1);

                obj.L_flx(i) = obj.l_flx_current;
                
                force_residuals(i,:) = [ext_res,flx_res];
            end
            
            obj.min_l_ext = min(obj.L_ext);
            obj.max_l_ext = max(obj.L_ext);
            obj.min_l_flx = min(obj.L_flx);
            obj.max_l_flx = max(obj.L_flx);

            obj.to_plot = 1;
            if obj.to_plot
                figure(gcf+1)
                clf
                hold on
                title([obj.joint_name,' muscle forces under maximum extending torque'])
                plot(obj.theta,T_ext_ext,'Linewidth',2)
                plot(obj.theta,T_flx_ext,'g','Linewidth',2)
                legend('T_{ext}','T_{flx}')
                hold off
                
                figure(gcf+1)
                clf
                hold on
                title([obj.joint_name,' muscle forces under maximum flexing torque'])
                plot(obj.theta,T_ext_flx,'Linewidth',2)
                plot(obj.theta,T_flx_flx,'g','Linewidth',2)
                legend('T_{ext}','T_{flx}')
                hold off
                
                figure(gcf+1)
                clf
                hold on
                title([obj.joint_name,' muscle lengths for desired position and stiffness'])
                plot(obj.theta,obj.L_ext,'b','Linewidth',2);
                plot(obj.theta,obj.L_flx,'g','Linewidth',2);
                legend('L_{ext}','L_{flx}')
                hold off

            end
            
            if set_musc_properties
            
                %To determine if the position and force maps are accurate, we
                %can make sure that the angle-length curve is monotonically
                %decreasing (for the extensor), and that its rate of change is
                %sufficiently large. If the slope approaches 0 or becomes
                %larger than 0, our muscle forces will become enormous and our
                %muscle lengths will not have a unique joint rotation.
                dldtheta_norm = diff(obj.L_ext)/range(obj.L_ext);
                d2ldtheta2_norm = diff(dldtheta_norm)/range(obj.L_ext);
                if any(dldtheta_norm > -.05) || any(abs(d2ldtheta2_norm) > 2.5)
                    fprintf('The %s rotation - extensor length mapping is poorly conditioned. \nPosition mapping will not be accurate,\nand muscle forces will be disproportionally large.\nModify muscle attachment points until this \nis no longer the case.\n\n',obj.joint_name);
                end

                %A factor of 1/0.8=1.25 is applied to all of the tension
                %calculations to account for the L-T relationship, which we
                %limit to 0.8 of the maximum.
                %We will use the biological trend of l_width = 1/3*l_rest
                %unless this will not give us desireable muscle force over the
                %range of motion.
                obj.l_width_ext = obj.l_ext_rest/3; %m
                obj.l_width_flx = obj.l_flx_rest/3; %m

                [~,ext_activation_max_L] = obj.steady_state_tension(1,max(obj.L_ext),'ext');
                [~,flx_activation_max_L] = obj.steady_state_tension(1,max(obj.L_flx),'flx');

                [~,ext_activation_min_L] = obj.steady_state_tension(1,min(obj.L_ext),'ext');
                [~,flx_activation_min_L] = obj.steady_state_tension(1,min(obj.L_flx),'flx');

                if ext_activation_max_L < 4/5 || ext_activation_min_L < 4/5
                    if ext_activation_max_L < ext_activation_min_L
                        %ext_activation_max_L is the more constricting case
                        obj.l_width_ext = sqrt(5)*(max(obj.L_ext) - obj.l_ext_rest);
                    else
                        obj.l_width_ext = -sqrt(5)*(min(obj.L_ext) - obj.l_ext_rest);
                    end
                    %Our activation curve will need to account for L-T affects
                    %up to 0.8.
                    ext_act_factor = 1.25; %1/0.8
                else
                    ext_act_factor = 1/min(ext_activation_max_L,ext_activation_min_L);
                end

                if flx_activation_max_L < 4/5 || flx_activation_min_L < 4/5
                    if flx_activation_max_L < flx_activation_min_L
                        %flx_activation_max_L is the more constricting case
                        obj.l_width_flx = sqrt(5)*(max(obj.L_flx) - obj.l_flx_rest);
                    else
                        obj.l_width_flx = -sqrt(5)*(min(obj.L_flx) - obj.l_flx_rest);
                    end
                    %Our activation curve will need to account for L-T affects
                    %up to 0.8.
                    flx_act_factor = 1.25; %1/0.8
                else
                    flx_act_factor = 1/min(flx_activation_max_L,flx_activation_min_L);
                end


                %From max muscle forces, we can calculate the passive component
                %parameters. in our notes, b=1 to establish this relationship.
                obj.kse_ext = ext_act_factor*max([T_ext_ext;T_ext_flx])/(obj.max_l_ext - obj.min_l_ext);
                obj.kse_flx = flx_act_factor*max([T_flx_flx;T_flx_ext])/(obj.max_l_flx - obj.min_l_flx);

                %From our notes, a = .2.
                %To ensure that the passive forces do not overwhelm the muscle
                %dynamics, compute kpe based on the maximum force and length of
                %the muscles.
                obj.kpe_ext = ext_act_factor*0.2*max([T_ext_ext;T_ext_flx])/(obj.max_l_ext - obj.min_l_ext);
                obj.kpe_flx = flx_act_factor*0.2*max([T_flx_flx;T_flx_ext])/(obj.max_l_flx - obj.min_l_flx);

                %With these parameters, A_max will be T_max, adjusted by the
                %L-T relationship.
                obj.Amp_ext = ext_act_factor*max([T_ext_ext;T_ext_flx]);
                obj.Amp_flx = flx_act_factor*max([T_flx_flx;T_flx_ext]);

                %Compute c based on the maximum contraction velocity (which is
                %based on muscle length and the min/max velocities of the
                %joint). 
                [ext_data,flx_data] = obj.compute_musc_velocities();
                ext_vel = ext_data{1,2};
                flx_vel = flx_data{1,2};

                %If kx+cx_dot=(kse+kpe)/kse*T-F, all of the applied force F is
                %absorbed by the passive elements when T=0. Ignoring kx,
                %c=F_max/x_dot_max.            
                obj.c_ext = .5*obj.Amp_ext/max(-ext_vel(:,end)); %max ext vel while extending
                obj.c_flx = .5*obj.Amp_flx/max(-flx_vel(:,1)); %max flx vel while flexing

                %Steepness will be tuned later
                obj.steep_ext = 200;
                obj.steep_flx = 200;

                %This value will be kept at 0
                obj.yoff_ext = 0;
                obj.yoff_flx = 0;

                %This value will be kept at -.040
                obj.xoff_ext = -.040; %V
                obj.xoff_flx = -.040; %V            

    %             disp('Tensions calculated. Preliminary properties assigned.')

                output = 1;
            else
%                 output = 
            end
        end
        %% Function: Plot Joint Axes Over Time
        function joint_axes_vis(obj)
            %This function is meant to help visualize each joint's uu_joint, uu_joint2, and uu_joint3. Use this to help make Ext/Flx,Abd/Add,Exr/InR motion
            %make sense. Remember: uu_joints are all defined in the distal body's coordinate system (knee->tibia, ankle->foot) 
            scale = 1000;
            i = 1;
            pausebutton = uicontrol('Style', 'ToggleButton', ...
                'Units',    'pixels', ...
                'Position', [125 400 60 20], ...
                'String',   'Pause', ...
                'Value',    0);
            stopbutton = uicontrol('Style', 'ToggleButton', ...
                'Units',    'pixels', ...
                'Position', [125 370 60 20], ...
                'String',   'Debug', ...
                'Value',    0);
            while i <= length(obj.theta_motion)
                %for i=1:10:length(obj.theta_motion)
%                 a = obj.joint_obj{1,1}.joint_rotmat_profile(:,:,i);
%                 b = obj.joint_obj{2,1}.joint_rotmat_profile(:,:,i);
%                 c = obj.joint_obj{3,1}.joint_rotmat_profile(:,:,i);
                femur = [scale*obj.joint_obj{1}.sim_position_profile(i,1),scale*obj.joint_obj{1}.sim_position_profile(i,2),scale*obj.joint_obj{1}.sim_position_profile(i,3);...
                scale*obj.joint_obj{2}.sim_position_profile(i,1),scale*obj.joint_obj{2}.sim_position_profile(i,2),scale*obj.joint_obj{2}.sim_position_profile(i,3)];
                tibia = [scale*obj.joint_obj{2}.sim_position_profile(i,1),scale*obj.joint_obj{2}.sim_position_profile(i,2),scale*obj.joint_obj{2}.sim_position_profile(i,3);...
                scale*obj.joint_obj{3}.sim_position_profile(i,1),scale*obj.joint_obj{3}.sim_position_profile(i,2),scale*obj.joint_obj{3}.sim_position_profile(i,3)];
                foot = scale*[obj.joint_obj{3}.sim_position_profile(i,1),obj.joint_obj{3}.sim_position_profile(i,2),obj.joint_obj{3}.sim_position_profile(i,3);...
                            obj.musc_obj{20, 1}.pos_attachments{5,4}(i,:)];
                hippos = scale*obj.joint_obj{1}.sim_position_profile(i,:);
                kneepos = scale*obj.joint_obj{2}.sim_position_profile(i,:);
                anklepos = scale*obj.joint_obj{3}.sim_position_profile(i,:);
                %Plot the hip
                    plot3(scale*obj.joint_obj{1}.sim_position_profile(i,1),scale*obj.joint_obj{1}.sim_position_profile(i,2),scale*obj.joint_obj{1}.sim_position_profile(i,3),'kp')
                    hold on
                    %plot hip uu_joint
                        hipuu  = [hippos;hippos+(obj.joint_obj{1}.uuw_joint)'*10];
                        hipuu2 = [hippos;hippos+(obj.joint_obj{1}.uuw_joint2(:,i))'*10];
                        hipuu3 = [hippos;hippos+(obj.joint_obj{1}.uuw_joint3(:,i))'*10];
%                         hipuu  = [hippos;hippos+(obj.CR_bodies(:,:,1)*obj.joint_obj{1}.uu_joint)'*10];
%                         hipuu2 = [hippos;hippos+(obj.CR_bodies(:,:,1)*obj.joint_obj{1}.uu_joint2(:,i))'*10];
%                         hipuu3 = [hippos;hippos+(obj.CR_bodies(:,:,1)*obj.joint_obj{1}.uu_joint3(:,i))'*10];
                        plot3(hipuu(:,1),hipuu(:,2),hipuu(:,3),'r')
                        %plot hip uu_joint2
                        plot3(hipuu2(:,1),hipuu2(:,2),hipuu2(:,3),'g')
                        %plot hip uu_joint3
                        plot3(hipuu3(:,1),hipuu3(:,2),hipuu3(:,3),'b')
                %Plot the knee
                    plot3(scale*obj.joint_obj{2}.sim_position_profile(i,1),scale*obj.joint_obj{2}.sim_position_profile(i,2),scale*obj.joint_obj{2}.sim_position_profile(i,3),'ks')
                    hold on
                    %plot knee uu_joint
                        kneeuu  = [kneepos;kneepos+(obj.joint_obj{2}.uuw_joint)'*10];
                        kneeuu2 = [kneepos;kneepos+(obj.joint_obj{2}.uuw_joint2(:,i))'*10];
                        kneeuu3 = [kneepos;kneepos+(obj.joint_obj{2}.uuw_joint3(:,i))'*10];
%                         kneeuu  = [kneepos;kneepos+(obj.CR_bodies(:,:,1)*obj.CR_bodies(:,:,2)*obj.joint_obj{2}.uu_joint)'*10];
%                         kneeuu2 = [kneepos;kneepos+(obj.CR_bodies(:,:,1)*obj.CR_bodies(:,:,2)*obj.joint_obj{2}.uu_joint2(:,i))'*10];
%                         kneeuu3 = [kneepos;kneepos+(obj.CR_bodies(:,:,1)*obj.CR_bodies(:,:,2)*obj.joint_obj{2}.uu_joint3(:,i))'*10];
                        plot3(kneeuu(:,1),kneeuu(:,2),kneeuu(:,3),'r')
                        %plot knee uu_joint2
                        plot3(kneeuu2(:,1),kneeuu2(:,2),kneeuu2(:,3),'g')
                        %plot knee uu_joint3
                        plot3(kneeuu3(:,1),kneeuu3(:,2),kneeuu3(:,3),'b')  
                %Plot the femur
                plot3(femur(:,1),femur(:,2),femur(:,3),'k--')
                %Plot the tibia
                plot3(tibia(:,1),tibia(:,2),tibia(:,3),'k-.')
                %Plot the ankle
                    plot3(scale*obj.joint_obj{3}.sim_position_profile(i,1),scale*obj.joint_obj{3}.sim_position_profile(i,2),scale*obj.joint_obj{3}.sim_position_profile(i,3),'kd')
                    hold on
                    %plot ankle uu_joint
                        ankleuu  = [anklepos;anklepos+(obj.joint_obj{3}.uuw_joint)'*10];
                        ankleuu2 = [anklepos;anklepos+(obj.joint_obj{3}.uuw_joint2(:,i))'*10];
                        ankleuu3 = [anklepos;anklepos+(obj.joint_obj{3}.uuw_joint3(:,i))'*10];
%                         ankleuu  = [anklepos;anklepos+(obj.CR_bodies(:,:,1)*obj.CR_bodies(:,:,2)*obj.CR_bodies(:,:,3)*obj.joint_obj{3}.uu_joint)'*10];
%                         ankleuu2 = [anklepos;anklepos+(obj.CR_bodies(:,:,1)*obj.CR_bodies(:,:,2)*obj.CR_bodies(:,:,3)*obj.joint_obj{3}.uu_joint2(:,i))'*10];
%                         ankleuu3 = [anklepos;anklepos+(obj.CR_bodies(:,:,1)*obj.CR_bodies(:,:,2)*obj.CR_bodies(:,:,3)*obj.joint_obj{3}.uu_joint3(:,i))'*10];
                        plot3(ankleuu(:,1),ankleuu(:,2),ankleuu(:,3),'r')
                        %plot ankle uu_joint2
                        plot3(ankleuu2(:,1),ankleuu2(:,2),ankleuu2(:,3),'g')
                        %plot ankle uu_joint3
                        plot3(ankleuu3(:,1),ankleuu3(:,2),ankleuu3(:,3),'b')  
                %Plot the foot
                plot3(foot(:,1),foot(:,2),foot(:,3),'k-.')
                grid on
                %Oblique View
%                 view([-32 31])
                view([0 90])
                xlim([-50 50])
                ylim([-80 10])
                zlim([0 30])
                axeslengths = [max(xlim)-min(xlim);max(ylim)-min(ylim);max(zlim)-min(zlim)];
                normedaxes = axeslengths/norm(axeslengths);
                pbaspect(normedaxes)
                set(gcf,'Position',[500 100 1300 800]);
                figure(1)
                hold off
                i = i + 10;
                pause_value = get(pausebutton, 'Value');
                stop_value = get(stopbutton, 'Value');
                pause(.001)
                if stop_value
                    keyboard
                end
                while pause_value
%                     pause
%                     set(buttonH,'Value',0)
%                     button_value = 0;
                    pause(.001)
                    pause_value = get(pausebutton, 'Value');
                    if get(stopbutton, 'Value')
                        keyboard
                    end
                end  
            end
            %end
        end
        %% Function: Plot and Classify Muscles by Muscle Lengths vs. Joint Angles
        function plot_muscles_by_joint(obj,jointnum,fullstep)
            relevant_muscles = [];
            num_muscles = length(obj.musc_obj);
            
            %classifyby = 1 is by muscle length
            %classifyby = 2 is by momentarm
            classifyby = 2;
            
            if fullstep ~= 0 && fullstep ~=1
                fullstep = input('Please specify whether you want to plot a full step of motion (1) or want to see the joint range of motion (0)');
                if fullstep ~= 0 && fullstep ~=1
                    fullstep = 1;
                end
            end
            
            for i = 1:num_muscles
                attachment_bodies = cell2mat(obj.musc_obj{i}.pos_attachments(:,3));
                if jointnum == 1 
                    if attachment_bodies(1) == 1
                        relevant_muscles = [relevant_muscles;i];
                    end
                elseif jointnum == 2
                    if attachment_bodies(end) == 3
                        relevant_muscles = [relevant_muscles;i];
                    end
                elseif jointnum == 3
                    if attachment_bodies(end) == 4
                        relevant_muscles = [relevant_muscles;i];
                    end
                end
            end
            
            if fullstep
                [beg,ennd,~] = find_step_indices(obj);
            else
                jointprofile = obj.joint_obj{jointnum}.rec_angle_profile;
                [~,maxlocs] = findpeaks(jointprofile);
                [~,minlocs] = findpeaks(-jointprofile);
                a3 = unique([maxlocs;minlocs]);
                a4 = jointprofile(a3);
                for i = 1:length(a4)-1
                    range(i,1) = abs(a4(i+1,1)-a4(i,1));
                end
                [~,dd] = max(range);
                dd = dd(1);
                beg = a3(dd);
                ennd = a3(dd+1);
            end

            fig1count = 0;
            fig2count = 0;
            fig3count = 0;
            
            %Change subplots to 4 to add an extra plot that shows the signs of the muscle derivative. Used to classify the muscles.
            subplots = 4;
            figure
            for i = 1:size(relevant_muscles,1)
                if classifyby == 2
                    [moment_arm_profile,mom_arm_inst_length] = compute_muscle_moment_arm(obj,obj.musc_obj{relevant_muscles(i)},1,jointnum,0,0);
                    muscle_length = moment_arm_profile(:,3);
                    time = moment_arm_profile(:,1);
                    mnorm = muscle_length/(max(muscle_length) - min(muscle_length));
                else
                    muscle_length = obj.musc_obj{relevant_muscles(i)}.muscle_length_profile(beg:ennd);
                    time = obj.joint_obj{jointnum}.rec_angle_time(beg:ennd);
                    mnorm = (muscle_length - min(muscle_length)) / (max(muscle_length) - min(muscle_length));
                end
                jointangle = obj.joint_obj{jointnum}.rec_angle_profile(beg:ennd)*(180/pi);
                tnorm = (time - min(time)) / (max(time) - min(time));
                if fullstep
                    dy=smooth(diff(mnorm)./diff(tnorm));
                else
                    dy=smooth(diff(mnorm)./diff(jointangle));
                end
                a = size(dy);
                dy = dy(~any(isnan(dy) | isinf(dy) | dy==0,2),:);
                b = size(dy);
                dy = sign(dy);
                jointangle = jointangle(2+(a-b):end);
                mnorm = mnorm(2+(a-b):end);
                tnorm = tnorm(2+(a-b):end);         
                dd = diff(dy);
                dd = dd(dd~=0);
                transitions = find(diff(dy));
                switches = size(dd,1);
                if size(unique(dy),1) > 1 
                    [c,~]=histogram(dy,unique(dy));
                end
                
                if subplots == 4
                    subplot(subplots,1,4)
                    if fullstep
                        plot(tnorm,dy)
                        title(['Slope switches =',num2str(switches),' firsttrans =',num2str(dd(1))]);
                    else
                        plot(jointangle,dy)
                        title([num2str(imp),' numpos=',num2str(bb),' numneg=',num2str(cc)]);
                    end
                end
                if classifyby == 2
                    if fullstep
                        if jointnum == 1
                            if dd(1) == -2
                                figurecount = 1;
                                fig1count = fig1count + 1;
                                legender{1,figurecount}(fig1count,:) = cellstr(obj.musc_obj{relevant_muscles(i)}.muscle_name(4:end));
                            else
                                if mean(dy(1:floor(size(dy,1)/2))) > 0
                                    figurecount = 2;
                                    fig2count = fig2count + 1;
                                    legender{1,figurecount}(fig2count,:) = cellstr(obj.musc_obj{relevant_muscles(i)}.muscle_name(4:end));
                                else
                                    figurecount = 3;
                                    fig3count = fig3count + 1;
                                    legender{1,figurecount}(fig3count,:) = cellstr(obj.musc_obj{relevant_muscles(i)}.muscle_name(4:end));
                                end
                            end
%                             if switches > 1 && abs(mean(dy(1:floor(size(dy,1)/2)))) < .9
%                                 figurecount = 3;
%                                 fig3count = fig3count + 1;
%                                 legender{1,figurecount}(fig3count,:) = cellstr(obj.musc_obj{relevant_muscles(i)}.muscle_name(4:end));
%                             else
%                                 if mean(dy(1:floor(size(dy,1)/2))) > 0
%                                     figurecount = 2;
%                                     fig2count = fig2count + 1;
%                                     legender{1,figurecount}(fig2count,:) = cellstr(obj.musc_obj{relevant_muscles(i)}.muscle_name(4:end));
%                                 else
%                                     figurecount = 1;
%                                     fig1count = fig1count + 1;
%                                     legender{1,figurecount}(fig1count,:) = cellstr(obj.musc_obj{relevant_muscles(i)}.muscle_name(4:end));
%                                 end
%                             end
                        elseif jointnum == 2
                            if abs(mean(dy(1:floor(size(dy,1)/2)))) > .85
                                if mean(dy(1:floor(size(dy,1)/2))) > 0
                                    figurecount = 1;
                                    fig1count = fig1count + 1;
                                    legender{1,figurecount}(fig1count,:) = cellstr(obj.musc_obj{relevant_muscles(i)}.muscle_name(4:end));
                                else
                                    figurecount = 2;
                                    fig2count = fig2count + 1;
                                    legender{1,figurecount}(fig2count,:) = cellstr(obj.musc_obj{relevant_muscles(i)}.muscle_name(4:end));
                                end
                            else
                                figurecount = 3;
                                fig3count = fig3count + 1;
                                legender{1,figurecount}(fig3count,:) = cellstr(obj.musc_obj{relevant_muscles(i)}.muscle_name(4:end));
                            end
                        elseif jointnum == 3
                            if transitions(1) < 500 && dd(1) < 0
                                figurecount = 3;
                                fig3count = fig3count + 1;
                                legender{1,figurecount}(fig3count,:) = cellstr(obj.musc_obj{relevant_muscles(i)}.muscle_name(4:end));
                            elseif transitions(1) < 500 && dd(1) > 0
                                    figurecount = 2;
                                    fig2count = fig2count + 1;
                                    legender{1,figurecount}(fig2count,:) = cellstr(obj.musc_obj{relevant_muscles(i)}.muscle_name(4:end));
                            else
                                figurecount = 1;
                                fig1count = fig1count + 1;
                                legender{1,figurecount}(fig1count,:) = cellstr(obj.musc_obj{relevant_muscles(i)}.muscle_name(4:end));
                            end
                        end
                    else
                        extflex = mean(dy);
                        [bb,~] = size(dy(dy>0));
                        [cc,~] = size(dy(dy<0));
                        imp = (dy(1)-mean(dy))/mean(dy);
                        if abs((bb-cc)/bb) < .6
                            figurecount = 3;
                            fig3count = fig3count + 1;
                            legender{1,figurecount}(fig3count,:) = cellstr(obj.musc_obj{relevant_muscles(i)}.muscle_name(4:end));
                        else
                            if extflex < 0 
                                figurecount = 1;
                                fig1count = fig1count + 1;
                                legender{1,figurecount}(fig1count,:) = cellstr(obj.musc_obj{relevant_muscles(i)}.muscle_name(4:end));
                            else
                                figurecount = 2;
                                fig2count = fig2count + 1;
                                legender{1,figurecount}(fig2count,:) = cellstr(obj.musc_obj{relevant_muscles(i)}.muscle_name(4:end));
                            end
                        end
                    end
                else
                    if fullstep
                        if jointnum == 1
                            if switches > 1 && abs(mean(dy(1:floor(size(dy,1)/2)))) < .9
                                figurecount = 3;
                                fig3count = fig3count + 1;
                                legender{1,figurecount}(fig3count,:) = cellstr(obj.musc_obj{relevant_muscles(i)}.muscle_name(4:end));
                            else
                                if mean(dy(1:floor(size(dy,1)/2))) > 0
                                    figurecount = 2;
                                    fig2count = fig2count + 1;
                                    legender{1,figurecount}(fig2count,:) = cellstr(obj.musc_obj{relevant_muscles(i)}.muscle_name(4:end));
                                else
                                    figurecount = 1;
                                    fig1count = fig1count + 1;
                                    legender{1,figurecount}(fig1count,:) = cellstr(obj.musc_obj{relevant_muscles(i)}.muscle_name(4:end));
                                end
                            end
                        elseif jointnum == 2
                            if abs(mean(dy(1:floor(size(dy,1)/2)))) > .85
                                if mean(dy(1:floor(size(dy,1)/2))) > 0
                                    figurecount = 1;
                                    fig1count = fig1count + 1;
                                    legender{1,figurecount}(fig1count,:) = cellstr(obj.musc_obj{relevant_muscles(i)}.muscle_name(4:end));
                                else
                                    figurecount = 2;
                                    fig2count = fig2count + 1;
                                    legender{1,figurecount}(fig2count,:) = cellstr(obj.musc_obj{relevant_muscles(i)}.muscle_name(4:end));
                                end
                            else
                                figurecount = 3;
                                fig3count = fig3count + 1;
                                legender{1,figurecount}(fig3count,:) = cellstr(obj.musc_obj{relevant_muscles(i)}.muscle_name(4:end));
                            end
                        elseif jointnum == 3
                            if transitions(1) < 500 && dd(1) < 0
                                figurecount = 3;
                                fig3count = fig3count + 1;
                                legender{1,figurecount}(fig3count,:) = cellstr(obj.musc_obj{relevant_muscles(i)}.muscle_name(4:end));
                            elseif transitions(1) < 500 && dd(1) > 0
                                    figurecount = 2;
                                    fig2count = fig2count + 1;
                                    legender{1,figurecount}(fig2count,:) = cellstr(obj.musc_obj{relevant_muscles(i)}.muscle_name(4:end));
                            else
                                figurecount = 1;
                                fig1count = fig1count + 1;
                                legender{1,figurecount}(fig1count,:) = cellstr(obj.musc_obj{relevant_muscles(i)}.muscle_name(4:end));
                            end
                        end
                    else
                        extflex = mean(dy);
                        [bb,~] = size(dy(dy>0));
                        [cc,~] = size(dy(dy<0));
                        imp = (dy(1)-mean(dy))/mean(dy);
                        if abs((bb-cc)/bb) < .6
                            figurecount = 3;
                            fig3count = fig3count + 1;
                            legender{1,figurecount}(fig3count,:) = cellstr(obj.musc_obj{relevant_muscles(i)}.muscle_name(4:end));
                        else
                            if extflex < 0 
                                figurecount = 1;
                                fig1count = fig1count + 1;
                                legender{1,figurecount}(fig1count,:) = cellstr(obj.musc_obj{relevant_muscles(i)}.muscle_name(4:end));
                            else
                                figurecount = 2;
                                fig2count = fig2count + 1;
                                legender{1,figurecount}(fig2count,:) = cellstr(obj.musc_obj{relevant_muscles(i)}.muscle_name(4:end));
                            end
                        end
                    end
                end
                subplot(subplots,1,figurecount)
                if fullstep
                    plot(tnorm,mnorm)
                    if figurecount == 1
                        title(obj.joint_obj{jointnum}.name(4:end))
                    end
                    xlabel('Normalized step cycle')
                    ylabel('Muscle length (normalized)')
                else
                    plot(jointangle,mnorm)
                    xlabel(['Joint ',num2str(jointnum),' Angle (degrees)'])
                    ylabel('Muscle length (normalized)')
                end 
                legend(legender{1,figurecount},'Location','eastoutside')
                hold on
                extflex = 0;
            end
            if fullstep
                footer = [obj.joint_obj{jointnum}.name(4:end),'FullStep'];
            else
                footer = [obj.joint_obj{jointnum}.name(4:end),'JointRange'];
            end
            set(gcf,'Position',[500 50 900 900])
            axh = findobj( gcf, 'Type', 'Axes' );
            ax1 = get(axh(1),'Position');
            ax2= get(axh(2),'Position');
            ax3= get(axh(3),'Position');
            ax = [ax1;ax2;ax3];
            ax(:,3) = 1.5*min(ax(:,3));
            set(axh(1),'Position',ax(1,:))
            set(axh(2),'Position',ax(2,:))
            set(axh(3),'Position',ax(3,:))
            saveas(gcf,[pwd,'\OutputFigures\muscle_lengths_step\',footer,datestr(datetime('now'),'yyyymmdd'),'.png'])
        end
        %% Function: Plot Joint Passive Torque for a Step
        function plot_joint_passive_torque(obj)
            legendvec = cell(3,1);
            for i = 1:3
                subplot(2,1,1)
                [passive_joint_torque,passive_joint_motion] = compute_passive_joint_torque(obj);
                legendvec{i,1} = obj.joint_obj{i}.name(4:end);
                xax = linspace(0,1,length(passive_joint_torque));
                plot(xax,passive_joint_torque(i,:))
                xlabel('Normalized Step Cycle')
                ylabel('Passive Joint Torque (Nm)')
                title('Passive Joint Torque - One Step')
                hold on
                subplot(2,1,2)
                plot(xax,passive_joint_motion(i,:))
                xlabel('Normalized Step Cycle')
                ylabel('Joint Motion (degrees)')
                hold on
            end
            subplot(2,1,1)
            legend(legendvec{:,1},'Location','eastoutside')
            set(findall(gca, 'Type', 'Line'),'LineWidth',2);
            subplot(2,1,2)
            legend(legendvec{:,1},'Location','eastoutside')
            set(gcf,'Position',[200 200 1250 650])
            set(findall(gca, 'Type', 'Line'),'LineWidth',2);
            saveas(gcf,[pwd,'\OutputFigures\Images\passive_joint_torque\','pjt',datestr(datetime('now'),'yyyymmdd'),'.png'])
        end
        %% Function: Optimize Forces for Torque Induction (older but works)
        function [act_wpass,act_nopass,tau2_wpass,tau2_nopass,moment_output,fval_wpass,fval_nopass,Fmt] = compute_forces_for_torque(obj,optset,toplot)
            %timeangle = [];
            tstart = tic;
            
            [beg,ennd,~] = find_step_indices(obj);
            div = 500;
            xx = floor(linspace(beg,ennd,div));
            xx = obj.sampling_vector;
            
            x = 1:length(obj.torque_motion);
            v = obj.torque_motion;
            
            xq = linspace(1,length(obj.torque_motion),length(xx));
            measured_torque = interp1(x,v,xq);
            
            joint_torques = obj.compute_total_joint_torque(0);
            x = 1:length(joint_torques);
            xq = linspace(1,length(joint_torques),length(xx));
            passive_torque = interp1(x,joint_torques,xq);
            
            %%Hip and Knee data in wrong order
            holder  = measured_torque(:,1);
            measured_torque(:,1) = measured_torque(:,2);
            measured_torque(:,2) = holder;
            
            tau2_wpass = measured_torque+passive_torque;
            tau2_nopass = measured_torque;
            
            options = optimoptions('linprog','Display','none');
            mintypes = {'minfatigue','minsqforces','minforce','minwork','minforcePCSA','minforcetemporal'};
            nummuscles = size(obj.musc_obj,1);
            %optset = 3;
            forces = zeros(nummuscles,size(xx,2));

            % Force Length: Hill model
                fl = @(Lm,Lr,Lw) 1-(Lm-Lr).^2./Lw^2;
            % parameters developed from Optimizer/Llord_work/fvsolver_params.m
            % 1.6Fmax at 3Lo/s and 0 at -4Lo/s
                fv = @(V) .8*(1+tanh(3.0648*(V+.083337)));
            %Thelen 2003, slightly modified eqn (3)
                fp = @(k,Lm,eo) (exp(k*(Lm-1)/eo)-1)/(exp(.5*(k/eo))-1);
            phi = @(Lopt,phiopt,Lm) asin((Lopt.*sin(phiopt))./(Lm));
            Lopt = @(Lr,gamma,a) Lr*(gamma*(1-a)+1);
            
            switch mintypes{optset}
                case 'minfatigue'
                        p = gcp('nocreate');
                        if isempty(p)
                            parpool('local',4);
                        end
                    options = optimoptions('fmincon','Display','none');
                    fun = @(x) sum(x.^2);
                    for i=1:nummuscles
                        muscle = obj.musc_obj{i};
                        Lm = muscle.muscle_length_profile(xx);
                        Vm = muscle.muscle_velocity_profile(xx);
                        Fmax(i,1) = muscle.max_force;
                        Lw = abs(muscle.l_max-muscle.l_min)/sqrt(.3);
                        Lr = muscle.RestingLength;
                        force_len = fl(Lm,Lr,Lw);
                        force_vel = fv(Vm);
                        force_pass = fp(6,Lm,.6);
                        phiopt = muscle.pennation_angle*(pi/180);
                        phi_now = cos(phi(Lopt(Lr,.15,0),phiopt,Lm));
                        Fmt(:,i) = Fmax(i,1).*(force_len.*force_vel+force_pass).*cos(phi_now);
                    end
                    ub = ones(nummuscles,1);
                    lb = ub.*0;
%                     lb = .005.*Fmax;
                    x0 = lb;
                    moment_output = zeros(nummuscles+2,size(xx,2),3);
                    for i=1:3
                        moment_output(:,:,i) = compute_joint_moment_arms(obj,i,1);
                    end
                    parfor j = 1:length(xx)
                        p = [moment_output(1:nummuscles,j,1)';moment_output(1:nummuscles,j,2)';moment_output(1:nummuscles,j,3)'];
                        beq_wpass = tau2_wpass(j,:);
                        beq_nopass = tau2_nopass(j,:);
                        Aeq = repmat(Fmt(j,:),[3 1]).*p/1000;
                        [act_wpass(:,j),fval_wpass(j,1),exitflag] = fmincon(fun,x0,[],[],Aeq,beq_wpass,lb,ub,[],options);
                        [act_nopass(:,j),fval_nopass(j,1),exitflag] = fmincon(fun,x0,[],[],Aeq,beq_nopass,lb,ub,[],options);
                    end
                case 'minsqforces'
                        p = gcp('nocreate');
                        if isempty(p)
                            parpool('local',4);
                        end
                    options = optimoptions('fmincon','Display','none');
                    for i=1:nummuscles
                        muscle = obj.musc_obj{i};
                        Lm = muscle.muscle_length_profile(xx);
                        Vm = muscle.muscle_velocity_profile(xx);
                        Fmax(i,1) = muscle.max_force;
                        Lw = abs(muscle.l_max-muscle.l_min)/sqrt(.3);
                        Lr = muscle.RestingLength;
                        force_len = fl(Lm,Lr,Lw);
                        force_vel = fv(Vm);
                        force_pass = fp(6,Lm,.6);
                        phiopt = muscle.pennation_angle*(pi/180);
                        phi_now = cos(phi(Lopt(Lr,.15,0),phiopt,Lm));
                        Fmt(:,i) = Fmax(i,1).*(force_len.*force_vel+force_pass).*cos(phi_now);
                        s = muscle.steepness;
                        xoff = muscle.x_off;
                        %lb(i,1) = Fmax(i,1)/(1+exp(s.*(xoff--.06)));
                    end
                    fun = @(x) sum((x./Fmax).^2);
%                     ub = ones(nummuscles,1);
                    ub = Fmax;
                    lb = ub.*0;
                    %lb = .005.*Fmax;
                    x0 = lb;
                    moment_output = zeros(nummuscles+2,size(xx,2),3);
                    for i=1:3
                        moment_output(:,:,i) = compute_joint_moment_arms(obj,i,1);
                    end
                    parfor j = 1:length(xx)
                        p = [moment_output(1:nummuscles,j,1)';moment_output(1:nummuscles,j,2)';moment_output(1:nummuscles,j,3)'];
                        beq_wpass = tau2_wpass(j,:);
                        beq_nopass = tau2_nopass(j,:);
                        Aeq = p/1000;
                        [act_wpass(:,j),fval_wpass(j,1),exitflag] = fmincon(fun,x0,[],[],Aeq,beq_wpass,lb,ub,[],options);
                        [act_nopass(:,j),fval_nopass(j,1),exitflag] = fmincon(fun,x0,[],[],Aeq,beq_nopass,lb,ub,[],options);
                    end
                case 'minforce'
%                     f = ones(1,nummuscles);
                    f = ones(1,nummuscles);
%                     f(1,[2,3,4,7,11,12,16,18,22,28,29,31,33,35]) = 1;
                    lb = zeros(1,nummuscles);
                    for i=1:nummuscles
                        if f(i) ~= 0
%                             ub(1,i) = .99*obj.musc_obj{i}.max_force;
                            ub(1,i) = obj.musc_obj{i}.max_force;
                        else
                            ub(1,i) = 0;
                        end
                    end
                    moment_output = zeros(nummuscles+2,size(xx,2),3);
                    for i=1:3
                        moment_output(:,:,i) = compute_joint_moment_arms(obj,i,1);
                    end
                    for j = 1:length(xx)
                        p = [moment_output(1:nummuscles,j,1)';moment_output(1:nummuscles,j,2)';moment_output(1:nummuscles,j,3)'];
                        beq = tau2_wpass(j,:);
                        Aeq = p/1000;
                        [forces(:,j),fval(j,1),exitflag] = linprog(f,[],[],Aeq,beq,lb,ub,[],options);
                    end
                case 'minwork'
                    lb = zeros(1,nummuscles);
                    moment_output = zeros(nummuscles+2,size(xx,2),3);
                    extlens = zeros(nummuscles,100);
                    fval = lb';
                    for i=1:3
                        moment_output(:,:,i) = compute_joint_moment_arms(obj,i,1);
                    end
                    for j = 1:length(xx)
                        f = zeros(nummuscles,1);
                        bb = [2,3,4,7,11,12,16,18,22,28,29,31,33,35];
                        for k = 1:nummuscles
                            if sum(bb==k) ~= 0
                                muscle = obj.musc_obj{k};
                                f(k,1) = abs(muscle.muscle_length_profile(xx(j),1)-muscle.l_min);
%                                 ub(1,k) = .99*obj.musc_obj{k}.max_force;
                                    ub(1,k) = 1000*obj.musc_obj{k}.max_force;
                            else
                                ub(1,k) = 0;
                            end
                        end
                        extlens(:,j) = f;
                        p = [moment_output(1:nummuscles,j,1)';moment_output(1:nummuscles,j,2)';moment_output(1:nummuscles,j,3)'];
                        beq = tau2(j,:);
                        Aeq = p/1000;
                        [forces(:,j),fval(j,1)] = linprog(f,[],[],Aeq,beq,lb,ub,[],options);
                    end
                case 'minforcePCSA'
                    lb = zeros(1,nummuscles);
                    moment_output = zeros(nummuscles+2,size(xx,2),3);
                    fval = lb';
                    for i=1:3
                        moment_output(:,:,i) = compute_joint_moment_arms(obj,i,1);
                    end                    
                    for gg = 1:nummuscles
                        PCSA(gg,1) = (cos(obj.musc_obj{gg}.pennation_angle*(pi/180))*(obj.musc_obj{gg}.mass/1000))/((obj.musc_obj{gg}.opt_fiber_length/10)*1.0564);
                        ub(1,gg) = .99*obj.musc_obj{gg}.max_force;
                    end
                    npcsa = (PCSA-min(PCSA))/(max(PCSA)-min(PCSA));
                    f = npcsa;
                    for j = 1:length(xx)
                        p = [moment_output(1:nummuscles,j,1)';moment_output(1:nummuscles,j,2)';moment_output(1:nummuscles,j,3)'];
                        beq = tau2(j,:);
                        Aeq = p/1000;
                        [forces(:,j),fval(j,1)] = linprog(f,[],[],Aeq,beq,lb,ub,[],options);
                    end
                case 'minforcetemporal'
                    lb = zeros(1,nummuscles);
                    moment_output = zeros(nummuscles+2,size(xx,2),3);
                    fval = lb';
                    f_log =zeros(nummuscles,div);
                    for i=1:3
                        moment_output(:,:,i) = compute_joint_moment_arms(obj,i,1);
                    end
                    counter = zeros(nummuscles,1);
                    dampers = zeros(nummuscles,div-1);
                    for j = 1:length(xx)
                        f = ones(nummuscles,1);
                        f_log(:,j) = f;
                        p = [moment_output(1:nummuscles,j,1)';moment_output(1:nummuscles,j,2)';moment_output(1:nummuscles,j,3)'];
                        if j>1
                            forceindex = double(forces(:,j-1)~=0);
                            counter = (counter.*forceindex)+forceindex;
%                             counter = double(counter.*forces(:,j-1)~=0)+double(forces(:,j-1)~=0);
                            multbyf = 1./(ones(nummuscles,1)+counter);
                            dampers(:,j-1) = multbyf;
                             %f = f.*multbyf;
                             p = p.*multbyf';
                        end
                        beq = tau2(j,:);
                        Aeq = p/1000;
                        [forces(:,j),fval(j,1)] = linprog(f,[],[],Aeq,beq,lb,[],[],options);
                    end
            end
            telapsed = toc(tstart);
            disp(['Force Optimization Time:',' (',num2str(telapsed),'s)'])
            clear tstart telapsed
            
            if toplot
                forces2plot = [];
                legender = {};
                for ii = 1:size(forces,1)
                    if sum(forces(ii,:)) == 0
                    else
                        legender{1,end+1} = obj.musc_obj{ii}.muscle_name(4:end);
                        forces2plot = [forces2plot;forces(ii,:)];
                    end
                end
%                 sparsefig = figure;
%                     set(gcf,'Position',[600 200 800 800])
%                     [aa,bb] = find(forces);
%                     clr = forces(forces~=0);
%                     scatter(bb,aa,[],clr,'MarkerFaceColor','flat');
%                     set(gca,'YDir','rev')
%                     title(mintypes{optset})
%                     clear aa bb clr

                forcefig = figure;
                    set(forcefig,'Position',[200 100 1500 900])
                    forcesub = subplot(4,1,1);
                        CM = hsv(size(forces2plot,1));
                        for jj = 1:size(forces2plot,1)
                            plot(forces2plot(jj,:)','Color',CM(jj,:),'LineWidth',2)
                            hold on
                        end
                        title(mintypes{optset})
                        legend(legender,'Location','eastoutside')
                    colorforces2 = subplot(4,1,2);
                        CM = lines(size(forces2plot,1));
                        for jj = 1:size(forces2plot,1)
                            plot(forces2plot(jj,:)','Color',CM(jj,:),'LineWidth',2)
                            hold on
                        end
                        %colorforces2.Position = [colorforces2.Position(1) colorforces2.Position(2) forcesub.Position(3) forcesub.Position(4)];
                        legend(legender,'Location','westoutside')
                        drawnow
                        colorforces2.Position = [forcesub.Position(1) colorforces2.Position(2) forcesub.Position(3) forcesub.Position(4)];
                    jointsub = subplot(4,1,3);
                        plot(obj.theta_motion(xx,:),'LineWidth',2)
                        legend({'Hip','Knee','Ankle'},'Location','eastoutside')
                        drawnow;
                        jointsub.Position = [jointsub.Position(1) jointsub.Position(2) forcesub.Position(3) forcesub.Position(4)];
                    torquesub = subplot(4,1,4);
                        plot(tau2,'LineWidth',2)
                        legend({'Hip','Knee','Ankle'},'Location','eastoutside')
                        drawnow;
                        torquesub.Position = [torquesub.Position(1) torquesub.Position(2) forcesub.Position(3) forcesub.Position(4)];
                    clear CM

                easyforce = figure;
                    set(easyforce,'Position',[200 100 1500 900])
                    numforces = size(legender,2);
                    for ii = 1:4
                        subplot(4,1,ii)
                        if 5*ii > length(legender)
                            forceindex = ((1+5*(ii-1)):numforces);
                        else
                            forceindex = (1+5*(ii-1)):(5*ii);
                        end
                        plot(forces2plot(forceindex,:)','LineWidth',2.5)
                        hold on
                        ylimit = ylim;
                        ylimit = ylimit(2);
                        shadeheight = ones(1,100)*ylimit;
                        shadelength  = 1:100;
                        area(shadelength(1:50),shadeheight(1:50),'FaceAlpha',.05,'EdgeAlpha',0,'FaceColor',[1 0 0])
                        area(shadelength(50:100),shadeheight(50:100),'FaceAlpha',.05,'EdgeAlpha',0,'FaceColor',[0 0 1])
                        legend(legender(forceindex),'Location','eastoutside','FontSize',12);
                        xlabel('Percent Stride (%)')
                        ylabel('Force (N)')
                        grid on
                        if ii == 1
                            firstsubposition = get(gca,'Position');
                            title(mintypes{optset})
                        else
                            currentsubposition = get(gca,'Position');
                            currentsubposition = [currentsubposition(1:2),firstsubposition(3:4)];
                            set(gca,'Position',currentsubposition);
                        end
                    end
                    clear numforces ylimit shadeheight shadelength firstsubposition currentsubposition

%                 indivmuscprof = figure;
%                     set(gcf,'Position',[500 50 500 900])
%                     tt = sum(forces,2);
%                     tt2 = find(tt>0);
%                     muscles = size(tt2,1);
%                     counter = 1;
%                     for i = 1:size(forces,1)
%                         if sum(forces(i,:)) ~= 0
%                             subplot(muscles,1,counter)
%                             plot(forces(i,:))
%                             title(obj.musc_obj{i}.muscle_name(4:end))
%                             counter = counter + 1;
%                         end
%                     end
%                     clear tt tt2 muscles counter
                
%                     if ismac
%                         saveas(sparsefig,['/Volumes/GoogleDrive/My Drive/Rat/ForceMinimizationAnalysis/',datestr(datetime('now'),'yyyymmdd'),'_',mintypes{optset},'_sparse','.png'])
%                         saveas(forcefig,['/Volumes/GoogleDrive/My Drive/Rat/ForceMinimizationAnalysis/',datestr(datetime('now'),'yyyymmdd'),'_',mintypes{optset},'_forces','.png'])
%                         saveas(easyforce,['/Volumes/GoogleDrive/My Drive/Rat/ForceMinimizationAnalysis/',datestr(datetime('now'),'yyyymmdd'),'_',mintypes{optset},'_split','.png'])
%                         saveas(indivmuscprof,['/Volumes/GoogleDrive/My Drive/Rat/ForceMinimizationAnalysis/',datestr(datetime('now'),'yyyymmdd'),'_',mintypes{optset},'_indivforces','.png'])
%                     else
%                         saveas(sparsefig,['G:\My Drive\Rat\ForceMinimizationAnalysis\',datestr(datetime('now'),'yyyymmdd'),'_',mintypes{optset},'_sparse','.png'])
%                         saveas(forcefig,['G:\My Drive\Rat\ForceMinimizationAnalysis\',datestr(datetime('now'),'yyyymmdd'),'_',mintypes{optset},'_forces','.png'])
%                         saveas(easyforce,['G:\My Drive\Rat\ForceMinimizationAnalysis\',datestr(datetime('now'),'yyyymmdd'),'_',mintypes{optset},'_split','.png'])
%                         saveas(indivmuscprof,['G:\My Drive\Rat\ForceMinimizationAnalysis\',datestr(datetime('now'),'yyyymmdd'),'_',mintypes{optset},'_indivforces','.png'])
%                     end
            end
        end
        %% Function: Optimize Forces for Torque Induction (newer, input cost function)
        function [force,tau2,moment_output,fval,telapsed,indiv_torques] = optimize_forces(obj,fun,toplot)
            tstart = tic;
            if nargin < 2
                toplot = 0;
            end
            
%             [beg,ennd,~] = find_step_indices(obj);
%             div = 500;
%             xx = floor(linspace(beg,ennd,div));
%             xx = linspace(1,length(obj.theta_motion),length(obj.theta_motion));
            xx = obj.sampling_vector;
            beg = obj.sampling_vector(1);
            ennd = obj.sampling_vector(end);
            
            v = [obj.torque_motion' obj.torque_motion'  obj.torque_motion']';
            x = 1:length(v);
           
            xq = linspace(1,length(v),length(xx));
            measured_torque = interp1(x,v,xq);
            
            passive_torque = obj.compute_total_joint_torque(0);
            x = 1:length(passive_torque);
            xq = linspace(1,length(passive_torque),length(xx));
            passive_torque = interp1(x,passive_torque,xq);
            clear x xq 
            
            %%Hip and Knee data in wrong order
            holder  = measured_torque(:,1);
            measured_torque(:,1) = measured_torque(:,2);
            measured_torque(:,2) = holder;
            
             %tau2 = measured_torque-passive_torque;
             jointprofile = obj.theta_motion;
             trimmedjp = jointprofile(floor(length(jointprofile)*.1):floor(length(jointprofile)*.9),:);
            
            if mean(range(trimmedjp)) < 1e-3
                 tau2 = -passive_torque;
                 %tau2 = zeros(length(obj.sampling_vector),3);
                %[body_torque] = compute_body_torques(obj);
                %tau2 = body_torque;
            else
%                 tau2 = measured_torque-passive_torque;
                tau2 = -passive_torque;
                [body_torque] = compute_body_torques(obj);
                tau2 = body_torque;
            end
                [body_torque] = compute_body_torques(obj);
%                 tau2 = body_torque;
%                 tau2 = passive_torque;
%                 [passive_joint_torque,passive_joint_motion] = compute_passive_joint_torque(obj);
%                 tau2 = passive_joint_torque';
%                 tau2 = -passive_joint_torque';
                tau2 = -body_torque;
            
            mintypes = {'minfatigue','minsqforces','minforce','minwork','minforcePCSA','minforcetemporal'};
            nummuscles = size(obj.musc_obj,1);
            %optset = 3;

%             % Force Length: Hill model
%                 fl = @(Lm,Lr,Lw) 1-(Lm-Lr).^2./Lw^2;
%             % parameters developed from Optimizer/Llord_work/fvsolver_params.m
%             % 1.6Fmax at 3Lo/s and 0 at -4Lo/s
%                 fv = @(V) .8*(1+tanh(3.0648*(V+.083337)));
%             %Thelen 2003, slightly modified eqn (3)
%                 fp = @(k,Lm,eo) (exp(k*(Lm-1)/eo)-1)/(exp(.5*(k/eo))-1);
%                 phi = @(Lopt,phiopt,Lm) asin((Lopt.*sin(phiopt))./(Lm));
%                 Lopt = @(Lr,gamma,a) Lr*(gamma*(1-a)+1);
                if ischar(fun)
%                     options = optimoptions('linprog','Display','none');
                else
                    options = optimoptions('fmincon','Display','none','Algorithm','sqp','OptimalityTolerance',1e-4);
                    %,'PlotFcn',{@optimplotx,@optimplotfval,@optimplotfirstorderopt}
                end
                
                sagvec = -obj.joint_obj{1}.uuw_joint(:,1)';
                
                %% Assemble an array Q, to act as a lower bound that ensures that Am values are possible
                parfor i=1:nummuscles
                    muscle = obj.musc_obj{i};
                    Lm = muscle.muscle_length_profile(xx);
                    if length(xx) > length(muscle.muscle_velocity_profile)
                        Vm = [0;muscle.muscle_velocity_profile];
                    else
                        Vm = muscle.muscle_velocity_profile(xx);
                    end
                    Fmax(i,1) = muscle.max_force;
% %                     Lw = abs(muscle.l_max-muscle.l_min)/sqrt(.3);
%                     Lw = muscle.l_width;
                     Lr = muscle.RestingLength;
%                     force_len = fl(Lm,Lr,Lw);
%                     force_vel = fv(Vm);
%                     force_pass = fp(6,Lm,.6);
%                     phiopt = muscle.pennation_angle*(pi/180);
%                     phi_now = cos(phi(Lopt(Lr,.15,0),phiopt,Lm));
%                     Fmt(:,i) = Fmax(i,1).*(force_len.*force_vel+force_pass).*cos(phi_now);
%                     s = muscle.steepness;
%                     xoff = muscle.x_off;
                    delLmat(i,:) = max(Lm-Lr,0);
%                     mVmat(i,:) = Vm';
                    kp = muscle.Kpe;
                    ks = muscle.Kse;
                    dt = obj.dt_motion;
                    dt = ((ennd-beg)*dt)/length(xx);
                    b = muscle.damping;
                    a = b/(ks*dt);
                    Q(i,1) = (a/(1+a+kp/ks));
                    lengthmat(i,:) = (1/a)*(Q(i,1)*kp*delLmat(i,:)+Q(i,1)*b*Vm');
                    % This optimization is for sagittal plane forces and torques. In order to calculate full muscle forces (including out of plane muscle
                    % portions, we must "penalize" the sagittal plane upper bound values because they do not represent the true maximum tensions
                        % Determine which segment to use to determine "out of planeness", for the time being this is just the first free segment
                    freeseg = find(diff(cell2mat(muscle.pos_attachments(:,3))),1,'last');
                        % Create an array of muscle vectors pointing along the free muscle segment
                    mvec = muscle.pos_attachments{freeseg+1,4}(xx,:) - muscle.pos_attachments{freeseg,4}(xx,:);
                        % Create an array of the saggital plane vector, in this case just the negative hip joint vector
                    sagvecmat = sagvec.*ones(size(mvec));
                        % Multiple Fmax by the sine of the angle between the muscle segment and the sagittal vector
                        % When the muscle is perfectly sagittal, the sine portion will equal 1. When perfectly non-sagittal, it will be 0
                    ub(i,:) = Fmax(i,1).*sin(atan2d(vecnorm(cross(sagvecmat,mvec,2),2,2),dot(sagvecmat,mvec,2))*(pi/180))'; % "out of plane penalty"
                end
                x0 = zeros(size(ub,1),1);
                moment_output = zeros(nummuscles+2,size(xx,2),3);
                parfor i=1:3
                    moment_output(:,:,i) = compute_joint_moment_arms(obj,i,1);
                end
                force = zeros(nummuscles,length(xx));
                %lb(:,1) = lengthmat(:,1);
                momentArmsHip = moment_output(:,:,1);
                momentArmsKnee = moment_output(:,:,2);
                momentArmsAnkle = moment_output(:,:,3);
                lb = zeros(size(force));
                for j = 1:length(xx)
                    % Set the lower bound such that the result is forced to satisfy the passive-active inequality that guarantees that Am > 0 
                    %p = [moment_output(1:nummuscles,j,1)';moment_output(1:nummuscles,j,2)';moment_output(1:nummuscles,j,3)'];
                    p = [momentArmsHip(1:nummuscles,j),momentArmsKnee(1:nummuscles,j),momentArmsAnkle(1:nummuscles,j)]';
                    beq = tau2(j,:);
                    Aeq = p/1000;
                    if j == 1
                        lb(:,j) = lengthmat(:,1);
                    else
                        lb(:,j) = max(Q.*force(:,j-1)+lengthmat(:,j),0); 
                    end
                    if ischar(fun)
%                         if strcmp(fun,'minwork')
%                             [force(:,j),fval(j,1),exitflag] = linprog(delLmat(:,j),[],[],Aeq,beq,lb(:,j),ub,[],options);
%                         end
                    else
                        if j>1
                            x0 = force(:,j-1);
                        end
                          [force(:,j),~,exitflag(j)] = fmincon(fun,x0,[],[],Aeq,beq,lb(:,j),ub(:,j),[],options);
                    end
                end
                
                for ii = 1:3
                    indiv_torques(1:nummuscles,:,ii) = moment_output(1:nummuscles,:,ii)./1000.*force;
                end
                fval = 10;
            telapsed = toc(tstart);
            if telapsed>60
                min = num2str(floor(telapsed/60));
                sec = num2str(round(mod(telapsed,60)));
            else
                min = num2str(0);
                sec = num2str(round(telapsed,2));
            end
            disp(['Force Optimization Time:',' (',min,'m ',sec,'s)'])
            %clear tstart telapsed
            %%
            if toplot
                forces2plot = [];
                legender = {};
                for ii = 1:size(force,1)
                    if sum(force(ii,:)) == 0
                    else
                        legender{1,end+1} = obj.musc_obj{ii}.muscle_name(4:end);
                        forces2plot = [forces2plot;force(ii,:)];
                    end
                end

                forcefig = figure;
                    set(forcefig,'Position',[200 100 1500 900])
                    forcesub = subplot(4,1,1);
                        CM = hsv(size(forces2plot,1));
                        for jj = 1:size(forces2plot,1)
                            plot(forces2plot(jj,:)','Color',CM(jj,:),'LineWidth',2)
                            hold on
                        end
                        %title(mintypes{optset})
                        legend(legender,'Location','eastoutside')
                    colorforces2 = subplot(4,1,2);
                        CM = lines(size(forces2plot,1));
                        for jj = 1:size(forces2plot,1)
                            plot(forces2plot(jj,:)','Color',CM(jj,:),'LineWidth',2)
                            hold on
                        end
                        %colorforces2.Position = [colorforces2.Position(1) colorforces2.Position(2) forcesub.Position(3) forcesub.Position(4)];
                        legend(legender,'Location','westoutside')
                        drawnow
                        colorforces2.Position = [forcesub.Position(1) colorforces2.Position(2) forcesub.Position(3) forcesub.Position(4)];
                    jointsub = subplot(4,1,3);
                        plot(obj.theta_motion(xx,:),'LineWidth',2)
                        legend({'Hip','Knee','Ankle'},'Location','eastoutside')
                        drawnow;
                        jointsub.Position = [jointsub.Position(1) jointsub.Position(2) forcesub.Position(3) forcesub.Position(4)];
                    torquesub = subplot(4,1,4);
                        plot(tau2,'LineWidth',2)
                        legend({'Hip','Knee','Ankle'},'Location','eastoutside')
                        drawnow;
                        torquesub.Position = [torquesub.Position(1) torquesub.Position(2) forcesub.Position(3) forcesub.Position(4)];
                    clear CM

                easyforce = figure;
                    set(easyforce,'Position',[200 100 1500 900])
                    numforces = size(legender,2);
                    for ii = 1:4
                        subplot(4,1,ii)
                        if 5*ii > length(legender)
                            forceindex = ((1+5*(ii-1)):numforces);
                        else
                            forceindex = (1+5*(ii-1)):(5*ii);
                        end
                        plot(forces2plot(forceindex,:)','LineWidth',2.5)
                        hold on
                        ylimit = ylim;
                        ylimit = ylimit(2);
                        shadeheight = ones(1,100)*ylimit;
                        shadelength  = 1:100;
                        area(shadelength(1:50),shadeheight(1:50),'FaceAlpha',.05,'EdgeAlpha',0,'FaceColor',[1 0 0])
                        area(shadelength(50:100),shadeheight(50:100),'FaceAlpha',.05,'EdgeAlpha',0,'FaceColor',[0 0 1])
                        legend(legender(forceindex),'Location','eastoutside','FontSize',12);
                        xlabel('Percent Stride (%)')
                        ylabel('Force (N)')
                        grid on
                        if ii == 1
                            firstsubposition = get(gca,'Position');
                            title('Optimized')
                        else
                            currentsubposition = get(gca,'Position');
                            currentsubposition = [currentsubposition(1:2),firstsubposition(3:4)];
                            set(gca,'Position',currentsubposition);
                        end
                    end
                    clear numforces ylimit shadeheight shadelength firstsubposition currentsubposition
            end
        end    
        %% Function: Pedotti Optimization
        function results_cell = pedotti_optimization(obj)
            p = gcp('nocreate');
            if isempty(p)
                parpool('local',4);
            end

            results_cell = cell(7,2);
            results_cell(:,1) = {'';
                                 'Forces';
                                 'Torques';
                                 'Moment Arm output';
                                 'Fval';
                                 'Opt Time';
                                 'Indiv Torques'};
            results_cell(1,2) = {' '};
            num_muscles = length(obj.musc_obj);
            for i=1:num_muscles
                muscle = obj.musc_obj{i};
                Fmax(i,1) = muscle.max_force;
            end

            fun = @(x) sum((x./Fmax).^2);
            %fun = @(x) sum((x.^2)./Fmax);

            [forces,tau,moment_output,fval,telapsed,indiv_torques] = obj.optimize_forces(fun,0);
            results_cell{2,2} = forces;
            results_cell{3,2} = tau;
            results_cell{4,2} = moment_output;
            results_cell{5,2} = fval;
            results_cell{6,2} = telapsed;
            results_cell{7,2} = indiv_torques;
        end
        %% Function: Optimized Torque Forces to MN
        function [Amval] = forces_to_MN(obj)
            [beg,ennd,~] = find_step_indices(obj);
            force_profiles = obj.compute_forces_for_torque(1,0);
            nummuscles = length(obj.musc_obj);
            Al = @(L,Lr) 1-((min(abs(L-Lr),.5*Lr)).^2)./(.25.*Lr.^2);
            Am = @(T,dT,L,dL,Lr,ks,kp,b,Alval) (1./max(.001,Alval)).*((b/ks).*T-kp.*max(0,L-Lr)-b.*dL+(1+kp/ks).*T);
            n = length(force_profiles);
            m = length(obj.theta_motion(beg:ennd));
            Amval = zeros(nummuscles,m);
            for ii = 1:nummuscles
               T = interp1(1:n,force_profiles(ii,:),linspace(1,n,m))';
               muscle = obj.musc_obj{ii};
               dT = gradient(T,obj.dt_motion);
               L = muscle.muscle_length_profile(beg:ennd);
               dL = muscle.muscle_velocity_profile(beg:ennd);
               Lr = muscle.RestingLength;
               ks = muscle.Kse;
               kp = muscle.Kpe;
               b = muscle.damping;
               Alval = Al(L,Lr);
               if mean(T) < .005
                Amval(ii,:) = zeros(1,m);
               else
                Amval(ii,:) = Am(T,dT,L,dL,Lr,ks,kp,b,Alval);
               end
            end
        end
        %% Function: Visualize Force Activation over Stride
        function [colorlog,widthlog] = vis_walking(obj,forces,minmethnum,to_save)
            % All forces must be positive for this to work. Line widths are based on activation level, can't be negative.\
            close all
            [beg,ennd,~] = find_step_indices(obj);
            div = 500;
            xx = floor(linspace(beg,ennd,div));
            xx = obj.sampling_vector;
            tempcolor = hot(256);
            mintitles = {'Minimizing the Summed Forces of All Muscles','Minimizing the Summed Work of All Muscles'};
            mintypes = {'minfatigue','minsqforces','minforce','minwork','minforcePCSA','minforcetemporal'};
            if 0
                filecounter = 1;
                if ismac
                    filename = ['/Volumes/GoogleDrive/My Drive/Rat/ForceMinimizationAnalysis/Videos/',mintypes{minmethnum},'_',datestr(datetime('now'),'yyyymmdd'),'.gif'];
                else
                    filename = ['G:\My Drive\Rat\ForceMinimizationAnalysis\Videos\',mintypes{minmethnum},'_',datestr(datetime('now'),'yyyymmdd'),'.gif'];
                end
                while isfile(filename)
                    filecounter = filecounter + 1;
                    filename = ['G:\My Drive\Rat\ForceMinimizationAnalysis\Videos\',mintypes{minmethnum},'_',datestr(datetime('now'),'yyyymmdd'),'_',num2str(filecounter),'.gif'];
                end
            end
        for minmethod = minmethnum
                %[forces,mintypes] = compute_forces_for_torque(obj,minmethod,0);
%                 [forces,forces_nopass,tau2_wpass,tau2_nopass,moment_output,fval_wpass,fval_nopass,Fmt] = compute_forces_for_torque(obj,minmethod,0);
                %CM = CM(3:end-20,:);
                limrecorder =zeros(length(xx),6);
                colorlog = [];
                widthlog = [];
%                 v = VideoWriter(['G:\My Drive\Rat\ForceMinimizationAnalysis\Videos\',mintypes{minmethod},'_',datestr(datetime('now'),'yyyymmdd')],'Motion JPEG AVI');
%                 v.Quality = 100;
%                 v.FrameRate = 5;
%                 open(v);
            figure
            trim = floor(.2*length(tempcolor));
            tempcolor = tempcolor(trim:(end-trim),:);
            CM = colormap(flipud(tempcolor));
            maxforce = max(max(forces));
            for timecount = 1:2:length(xx)
                colorvec = 1+floor(length(CM)*(forces(:,timecount)./maxforce));
                %colorvec = floor(rescale((forces(:,timecount)),1,length(CM)));
                %widthvec = rescale(colorvec,.1,3);
                widthvec = 2*(forces(:,timecount));
                colorlog(:,timecount) = colorvec;
                widthlog(:,timecount) = widthvec;
                num_muscles = length(obj.musc_obj);
                for musnum = 1:num_muscles
                    muscle = obj.musc_obj{musnum};
                    musclemat =zeros(size(muscle.pos_attachments,1),3);
                    for hh = 1:size(muscle.pos_attachments,1)
                        temp = 1000*muscle.pos_attachments{hh,4}(xx(timecount),:);
                        musclemat(hh,:) = temp;
                    end
                    if colorvec(musnum,1) == 0
                        colorvec(musnum,1) = 1;
                        widthvec(musnum,1) = .01;
                    end
                    plot3(musclemat(:,1),musclemat(:,2),musclemat(:,3),'Color',CM(colorvec(musnum,1),:),'Linewidth',widthvec(musnum,1))
                    hold on
                end
                jointmat = zeros(4,3);
                for jointnum = 1:3
                    joint = obj.joint_obj{jointnum};
                    temp  = 1000*joint.sim_position_profile(xx(timecount),:);
                    jointmat(jointnum,:) = temp;
                end
                jointmat(4,:) = 1000*obj.musc_obj{20}.pos_attachments{end,4}(xx(timecount),:);
                plot3(jointmat(:,1),jointmat(:,2),jointmat(:,3),'m--','LineWidth',2,'Marker','o','MarkerFaceColor','k','MarkerSize',5)
                hold on
                    limrecorder(timecount,:) = [xlim,ylim,zlim];
                    xlimits = [-40 60];
                    ylimits = [-80 20];
                    zlimits = [0 30];
                    xlim(xlimits);
                    ylim(ylimits);
                    zlim(zlimits);
                    a =sum(abs(xlimits));
                    b =sum(abs(ylimits));
                    c =sum(abs(zlimits));
                    axeslengths = [a b c];
                    normedaxes = axeslengths/norm(axeslengths);
                pbaspect(normedaxes)
                view([0 90])
                 set(gcf,'Position',[800 50 1000 900])
                %set(gcf,'Position',[0 0 1910 1070])
%                 title(mintypes{minmethod},'FontSize',15)
                title(mintitles{minmethod},'FontSize',15)
                cbar = colorbar('Ticks',[0;1],'TickLabels',{'low act','high act'},'FontSize',15);
                grid on
                pause(.001)
                if to_save
                    frame = getframe(gcf);
                    im = frame2im(frame); 
                    [imind,cm] = rgb2ind(im,256); 
                    if timecount == 1 
                        imwrite(imind,cm,filename,'gif', 'Loopcount',inf,'DelayTime',.1); 
                    else 
                        imwrite(imind,cm,filename,'gif','WriteMode','append','DelayTime',.1); 
                    end 
                end
%                 writeVideo(v,frame);
                hold off
            end
%             close(v)
            clear forces v
        end
        end
        %% Function: Force to Voltage
        function voltage = force_to_voltage(obj,muscle,force)
            amplitude = muscle.max_force;
            steepness = muscle.steepness;
            xoff = muscle.x_off;
            yoff = muscle.y_off;
            
            if length(force) == 1
                voltage = xoff-log((amplitude/(force-yoff))-1)/steepness;
            else
                for ii = 1:length(force)
                    voltage(ii) = xoff-log((amplitude/(force(ii)-yoff))-1)/steepness;
                end
            end
        end
        %% Function: Biomimetics Paper 2018
        function biomimetics(obj)
            % BFA 33
            % Pect 5
            % Semimembranosus 31
            % VastusIntermedius 18
            % ExtDiGLong 20
            % MedGastroc 17
            % TibialisAnt 22
            joint = 1;
            axis = 1;
            [beg,ennd,~] = find_step_indices(obj);
            %Higher div, the longer the program takes.
            div = 100;
            xx = floor(linspace(beg,ennd,div));
            xx = obj.sampling_vector;
            if 0
                muscles = [11 11 33 33 27 27;...
                     1 2 1 2 2 3;...
                     2 1 2 1 3 2];
                 limitsy = [9.5 10 3 .8 4.6 0;...
                     16 16.5 8 4.2 5.6 6];
                musclenames = {'Biceps Femoris Posterior (Hip)';'Biceps Femoris Posterior (Knee)';'Biceps Femoris Anterior (Hip)';...
                    'Biceps Femoris Anterior (Knee)';'Plantaris (Knee)';'Plantaris (Ankle)'};
                jointnames = {'Hip';'Knee';'Ankle'};
                 for i=1:length(muscles)
                    subplot(3,2,i)
                    m = obj.musc_obj{muscles(1,i)};
                    [moment_arm_profile,~] = compute_muscle_moment_arm(obj,m,axis,muscles(2,i),0,0);
                    momprof = moment_arm_profile(:,3);
                    %momprof = (momprof-min(momprof))/(max(momprof)-min(momprof));
                    bb = m.muscle_length_profile(xx);
                    %bb = (bb-min(bb))/(max(bb)-min(bb));
                    tdlo = [];
                    tdlo(1,1) = 1000*bb(6);
                    tdlo(1,2) = momprof(6);
                    tdlo(2,1) = 1000*bb(51);
                    tdlo(2,2) = momprof(51);
                    plot(1000*bb,momprof,'k','LineWidth',1)
                    hold on
                    %Touch Down Marker
                    td = plot(tdlo(1,1),tdlo(1,2),'ro','MarkerSize',10,'MarkerFaceColor',[1 0 0]);
                    %Lift Off Marker
                    lo = plot(tdlo(2,1),tdlo(2,2),'go','MarkerSize',10,'MarkerFaceColor',[0 1 0]);
                    cc = [1000*bb,momprof];
                    ee = cc(10:20:end,:);
                    ff = cc(11:20:end,:);
                    C = ff([1;1]*(1:size(ff,1)),:);
                    C(1:2:end,:) = ee;
                                        cc2 = moment_arm_profile(:,2);
                    ee2 = cc2(10:20:end,:);
                    ff2 = cc2(11:20:end,:);
                    C2 = ff2([1;1]*(1:size(ff2,1)),:);
                    C2(1:2:end,:) = ee2;
                    for j = 1:2:length(C)-1
%                         arrow(C(j,:),C(j+1,:),'Length',5,'Width',0,'TipAngle',30,'Color','b')
                        arrow(C(j,:),C(j+1,:),'Length',5,'Width',.001,'Color','b')
                        [warnMsg, ~] = lastwarn;
                        if ~isempty(warnMsg)
                            arrow FIXLIMITS
                            lastwarn('');
                        end
                    end
                    grid on
                    title(musclenames{i})
                    ylabel('Moment Arm Length (mm)')
                    xlabel('Muscle Length (mm)')
                    ylim(limitsy(:,i))
                    switch i
                        case 1
                            legend([td,lo],{'Touch Down','Lift Off'},'Location','northeast')
                        case 2
                            legend([td,lo],{'Touch Down','Lift Off'},'Location','southeast')          
                        case 3
                            legend([td,lo],{'Touch Down','Lift Off'},'Location','southwest')
                        case 4
                            legend([td,lo],{'Touch Down','Lift Off'},'Location','southeast')
                        case 5
                            legend([td,lo],{'Touch Down','Lift Off'},'Location','northeast')
                        case 6
                            legend([td,lo],{'Touch Down','Lift Off'},'Location','southeast')
                    end
                 end
                 set(gcf,'Position',[800 10 1000 800])
                 keyboard
            end
            %This generates the charles_compare_figure subplots, the main isolated joint figures. There's no sensitivity analysis in this one.
            if 0
                muscles = [33 5 31 18 17 22 8 9 36;...
                        1 1 2 2 3 3 1 1 1];
                color = [1 0 0;0 1 0;0 0 1];
                limitsy = [-.4 -.2 -.3 0 -.15 -.05 -.143 -.063 -.08;...
                            .2 .2 0 .15 0 .15 .57 .043 .086];
                limitsx = [-30 -30 -145 -145 -50 -50 -30 -30 -30;...
                    50 50 -45 -45 50 50 50 50 50];
                musclenames = {'Biceps Femoris Anterior';'Pectineus';'Semimembranosus';...
                'Vastus Intermedius';'Medial Gastrocnemius';'Tibialis Anterior';'Tensor Fascia Latae';'Obturator Externus';'Obturator Internus'};
                for i=1:length(muscles)
                    if size(unique(obj.theta_motion(:,muscles(2,i))),1)==1
                    else
                        [moment_arm_profile,~] = compute_muscle_moment_arm(obj,obj.musc_obj{muscles(1,i)},axis,muscles(2,i),0,0);
                        figure
                        if muscles(2,i) == 1
                            jointangle = -moment_arm_profile(:,2);
                            musclename = obj.joint_obj{muscles(2,i)}.name(4:end-1);
                            momentarm = -moment_arm_profile(:,3)/35.745;
                        elseif muscles(2,i) == 2
                            jointangle = moment_arm_profile(:,2)-81.578;
                            musclename = obj.joint_obj{muscles(2,i)}.name(4:end);
                            momentarm = -moment_arm_profile(:,3)/35.745;
                        else
                            jointangle = -moment_arm_profile(:,2)-20.0535;
                            musclename = obj.joint_obj{muscles(2,i)}.name(4:end-1);
                            momentarm = -moment_arm_profile(:,3)/35.745;
                        end
                        %momnorm = (momentarm-min(momentarm))/(max(momentarm)-min(momentarm));
                        if muscles(2,i) == 2
                            plot(jointangle,-momentarm,'color',color(axis,:),'LineWidth',2)
                        else
                            plot(jointangle,momentarm,'color',color(axis,:),'LineWidth',2)
                        end
                        grid off
                        title(musclenames{i})
                        ylabel('Moment Arm/thigh length')
                        xlabel([musclename,' Angle (deg)'],'Interpreter','none')
                        ylim(limitsy(:,i))
                        xlim(limitsx(:,i))
                        %saveas(gcf,['G:\My Drive\Rat\BiomimeticsPaper\complete_charles_comparisons\',obj.musc_obj{muscles(1,i)}.muscle_name(4:end),'_',datestr(datetime('now'),'yyyymmdd'),'.png'])
                        %saveas(gcf,['G:\My Drive\Rat\BiomimeticsPaper\complete_charles_comparisons\',obj.musc_obj{muscles(1,i)}.muscle_name(4:end),'_',datestr(datetime('now'),'yyyymmdd'),'.eps'])
                    end
                end
                keyboard
            end
            %Sensitivity analysis of the isolated joint moment arms
            if 0      
                muscles = [33 5 31 18 17 22 8 9 36;...
                        1 1 2 2 3 3 1 1 1];
                color = [1 0 0;0 1 0;0 0 1];
                limitsy = [-.4 -.2 -.3 0 -.15 -.05 -.143 -.063 -.08;...
                            .2 .2 0 .15 0 .15 .57 .043 .086];
                limitsx = [-30 -30 -145 -145 -50 -50 -30 -30 -30;...
                    50 50 -45 -45 50 50 50 50 50];
                musclenames = {'Biceps Femoris Anterior';'Pectineus';'Semimembranosus';...
                'Vastus Intermedius';'Medial Gastrocnemius';'Tibialis Anterior';'Tensor Fascia Latae';'Obturator Externus';'Obturator Internus'};
                switch obj.sensemuscname
                    case 'BFA'
                        i = 1;
                    case 'Pect'
                        i = 2;
                    case 'SM'
                        i = 3;
                    case 'VI'
                        i = 4;
                    case 'MG'
                        i = 5;
                    case 'TA'
                        i = 6;
                end
                switch obj.sensecasenum
                    case 1
                        linecolor = 'k';
                        linetype = '-';
                        pathpath = ['G:\My Drive\Rat\BiomimeticsPaper\Sensitivity\',obj.sensemuscname,'\Base.png'];
                    case 2
                        linecolor = 'b';
                        linetype = '-';
                        pathpath = ['G:\My Drive\Rat\BiomimeticsPaper\Sensitivity\',obj.sensemuscname,'\Origin+.png'];
                    case 3
                        linecolor = 'b';
                        linetype = '--';
                        pathpath = ['G:\My Drive\Rat\BiomimeticsPaper\Sensitivity\',obj.sensemuscname,'\Origin-.png'];
                    case 4
                        linecolor = 'r';
                        linetype = '-';
                        pathpath = ['G:\My Drive\Rat\BiomimeticsPaper\Sensitivity\',obj.sensemuscname,'\Ins+.png'];
                    case 5
                        linecolor = 'r';
                        linetype = '--';
                        pathpath = ['G:\My Drive\Rat\BiomimeticsPaper\Sensitivity\',obj.sensemuscname,'\Ins-.png'];
                end
                if size(unique(obj.theta_motion(:,muscles(2,i))),1)==1
                else
                    [moment_arm_profile,~] = compute_muscle_moment_arm(obj,obj.musc_obj{muscles(1,i)},axis,muscles(2,i),0,0);
                    figure
                    if muscles(2,i) == 1
                        jointangle = -moment_arm_profile(:,2);
                        musclename = obj.joint_obj{muscles(2,i)}.name(4:end-1);
                        momentarm = -moment_arm_profile(:,3)/35.745;
                    elseif muscles(2,i) == 2
                        jointangle = moment_arm_profile(:,2)-81.578;
                        musclename = obj.joint_obj{muscles(2,i)}.name(4:end);
                        momentarm = -moment_arm_profile(:,3)/35.745;
                    else
                        jointangle = -moment_arm_profile(:,2)-20.0535;
                        musclename = obj.joint_obj{muscles(2,i)}.name(4:end-1);
                        momentarm = -moment_arm_profile(:,3)/35.745;
                    end
                    %momnorm = (momentarm-min(momentarm))/(max(momentarm)-min(momentarm));
                    if muscles(2,i) == 2
                        plot(jointangle,-momentarm,'color',linecolor,'LineWidth',2,'LineStyle',linetype)
                    else
                        plot(jointangle,momentarm,'color',linecolor,'LineWidth',2,'LineStyle',linetype)
                    end
                    grid off
                    title(musclenames{i})
                    ylabel('Moment Arm/thigh length')
                    xlabel([musclename,' Angle (deg)'],'Interpreter','none')
                    ylim(limitsy(:,i))
                    xlim(limitsx(:,i))
                    saveas(gcf,pathpath)
                end
                return
            end
            %Generates the 6 simulation images (with the flyouts) for the BFP, BFA, and the Plantaris
            if 0
                muscles = [11 11 33 33 27 27;...
                    1 2 1 2 2 3;...
                    2 1 2 1 3 2];
                for ii = 1:size(muscles,2)
                    compute_muscle_moment_arm(obj,obj.musc_obj{muscles(1,ii)},axis,muscles(2,ii),0,1);
                end
                keyboard
            end
            if 0
%                 muscles = [11 12 33 14 27 20;...
%                     1 1 1 1 2 2;...
%                     2 2 2 2 3 3];
%                 musclenames = {'Biceps Femoris Posterior';'Rectus Femoris';'Biceps Femoris Anterior';...
%                     'Semitendinosus Accessory';'Plantaris';'Extensor Digitorum Longus'};
                muscles = [11 11 33 33 27 27;...
                     1 2 1 2 2 3;...
                     2 1 2 1 3 2];
                zlimits = [9.8 9.8 1.4 1.4 .8 .8
                    16 16 7.8 7.8 6 6];
                musclenames = {'Biceps Femoris Posterior (Hip)';'Biceps Femoris Posterior (Knee)';'Biceps Femoris Anterior (Hip)';...
                    'Biceps Femoris Anterior (Knee)';'Plantaris (Knee)';'Plantaris (Ankle)'};
                jointnames = {'Hip';'Knee';'Ankle'};
                count2 = 1;
                for ii = 1:size(muscles,2)
                    [moment_arm_profile,~] = compute_muscle_moment_arm(obj,obj.musc_obj{muscles(1,ii)},axis,muscles(2,ii),0,0);
                    jointangles = [-obj.joint_obj{1}.rec_angle_profile(xx)*(180/pi),...
                        obj.joint_obj{2}.rec_angle_profile(xx)*(180/pi)-81.578,...
                        -obj.joint_obj{3}.rec_angle_profile(xx)*(180/pi)-20.0535];
                    %mnorm = (moment_arm_profile(:,3)-min(moment_arm_profile(:,3)))/(max(moment_arm_profile(:,3))-min(moment_arm_profile(:,3)));
                    subplot(3,2,ii)
                    zlim(zlimits(:,count2)')
                    bb = fill3(jointangles(:,muscles(2,count2)),jointangles(:,muscles(3,count2)),moment_arm_profile(:,3),'r');
                    %bb = fill3(jointangles(:,muscles(3,count2)),jointangles(:,muscles(2,count2)),mnorm,'r');
                    hold on
                    bb.FaceAlpha = .5;
                    bb.LineWidth = 2;
                    pbaspect([1 1 1])
                    view([-37 16])
                    title(musclenames{ii})
                    grid on
                    xlabel([jointnames{muscles(2,count2)},' (deg)'])
                    ylabel([jointnames{muscles(3,count2)},' (deg)'])
                    zlabel('Moment Arm Length (mm)')
                    count2 = count2 - 2*mod(ii,2) + 2;
                end
                set(gcf,'Position',[1000 10 800 975])
                saveas(gcf,['G:\My Drive\Rat\BiomimeticsPaper\projectedpathsfigure','\threeD','_',datestr(datetime('now'),'yyyymmdd'),'.png'])
                %%Flat graphs
                figure
                count2 = 1;
                for ii = 1:size(muscles,2)
                    [moment_arm_profile,~] = compute_muscle_moment_arm(obj,obj.musc_obj{muscles(1,ii)},axis,muscles(2,ii),0,0);
                    jointangles = [-obj.joint_obj{1}.rec_angle_profile(xx)*(180/pi),...
                        obj.joint_obj{2}.rec_angle_profile(xx)*(180/pi)-81.578,...
                        -obj.joint_obj{3}.rec_angle_profile(xx)*(180/pi)-20.0535];
                    momprof = moment_arm_profile(:,3);
                    %mnorm = (moment_arm_profile(:,3)-min(moment_arm_profile(:,3)))/(max(moment_arm_profile(:,3))-min(moment_arm_profile(:,3)));
                    subplot(3,2,ii)
                    if mod(ii,2)==1
                        sig = 1;
                    else
                        sig = -1;
                    end
                    row = 2;
                    bb = plot(sig*jointangles(:,muscles(row,ii)),moment_arm_profile(:,3),'k');
                    %bb = fill3(jointangles(:,muscles(3,count2)),jointangles(:,muscles(2,count2)),mnorm,'r');
                    hold on
                    tdlo = [];
                    tdlo(1,1) = sig*jointangles(6,muscles(row,ii));
                    tdlo(1,2) = momprof(6);
                    tdlo(2,1) = sig*jointangles(51,muscles(row,ii));
                    tdlo(2,2) = momprof(51);
                    %Touch Down Marker
                    td = plot(tdlo(1,1),tdlo(1,2),'ro','MarkerSize',10,'MarkerFaceColor',[1 0 0]);
                    %Lift Off Marker
                    lo = plot(tdlo(2,1),tdlo(2,2),'go','MarkerSize',10,'MarkerFaceColor',[0 1 0]);
                    %cc = [jointangles(:,muscles(2,count2)),jointangles(:,muscles(3,count2)),moment_arm_profile(:,3)];
                    cc = [sig*jointangles(:,muscles(row,ii)),moment_arm_profile(:,3)];
                    ee = cc(10:20:end,:);
                    ff = cc(11:20:end,:);
                    C = ff([1;1]*(1:size(ff,1)),:);
                    C(1:2:end,:) = ee;
                    for j = 1:2:length(C)-1
%                         arrow(C(j,:),C(j+1,:),'Length',5,'Width',0,'TipAngle',30,'Color','b')
                        arrow(C(j,:),C(j+1,:),'Length',5,'Width',.001,'Color','b')
                        [warnMsg, ~] = lastwarn;
                        if ~isempty(warnMsg)
                            arrow FIXLIMITS
                            lastwarn('');
                        end
                    end
                    bb.LineWidth = 2;
                    grid on
                    xlabel([jointnames{muscles(row,ii)},' (deg)'])
                    ylabel('Moment Arm Length (mm)')
                    ylim(zlimits(:,ii)')
                    pbaspect([1 1 1])
                    title(musclenames{ii})
                    count2 = count2 - 2*mod(ii,2) + 2;
                end
                set(gcf,'Position',[10 10 800 975])
                saveas(gcf,['G:\My Drive\Rat\BiomimeticsPaper\projectedpathsfigure','\flat','_',datestr(datetime('now'),'yyyymmdd'),'.png'])
                keyboard
            end
            if 0
                limits = [9.8 9.8 1.4 1.4 .8 .8
                    16 16 7.8 7.8 6 6];
                muscles = [11 11 33 33 27 27;...
                    1 2 1 2 2 3];
                musclenames = {'Biceps Femoris Posterior (Hip)';'Biceps Femoris Posterior (Knee)';'Biceps Femoris Anterior (Hip)';...
                                    'Biceps Femoris Anterior (Knee)';'Plantaris (Knee)';'Plantaris (Ankle)'};
                for ii = 1:size(muscles,2)
                    [moment_arm_profile,~] = compute_muscle_moment_arm(obj,obj.musc_obj{muscles(1,ii)},axis,muscles(2,ii),0,0);
                    jointangles = [-obj.joint_obj{1}.rec_angle_profile(xx)*(180/pi),...
                        obj.joint_obj{2}.rec_angle_profile(xx)*(180/pi)-81.578,...
                        -obj.joint_obj{3}.rec_angle_profile(xx)*(180/pi)-20.0535];
                    %jointangle = moment_arm_profile(:,2);
                    momentarm = moment_arm_profile(:,3);
                    if muscles(2,ii) ~= 2
                        jointname = obj.joint_obj{muscles(2,ii)}.name(4:end-1);
                    else
                        jointname = obj.joint_obj{muscles(2,ii)}.name(4:end);
                    end
                    subplot(3,2,ii)
                    plot(jointangles(:,muscles(2,ii)),momentarm,'LineWidth',2);
                    grid on
                    title(musclenames{ii})
                    xlabel([jointname,' Joint Angle (deg)'])
                    ylabel('Moment Arm Length (mm)')
                    pbaspect([1 1 1])
                    ylim(limits(:,ii)')
                end
                set(gcf,'Position',[500 20 600 900])
                %saveas(gcf,['G:\My Drive\Rat\BiomimeticsPaper','\momentarmloops','_',datestr(datetime('now'),'yyyymmdd'),'.png'])
                keyboard
            end
            for i=1:length(muscles)
%                     figure
%                         subplot(4,1,1)
%                         plot(moment_arm_profile(:,2),'LineWidth',2)
%                         if muscles(1,i) == 33
%                             hold on
%                             plot(obj.theta_motion(xx,2)*(180/pi),'LineWidth',2)
%                         end
%                         grid on
%                         title(obj.musc_obj{muscles(1,i)}.muscle_name(4:end))
%                         ylabel([obj.joint_obj{muscles(2,i)}.name(4:end),' Angle (deg)'])
%                         xlabel('Stride Percent')
%                         subplot(4,1,2)
%                         plot(moment_arm_profile(:,3),'LineWidth',2)
%                         grid on
%                         ylabel('Moment Arm Length (mm)')
%                         xlabel('Stride Percent')
%                         subplot(4,1,3)
%                         plot(moment_arm_profile(:,2),moment_arm_profile(:,3),'LineWidth',2)
%                         grid on
%                         ylabel('Moment Arm Length (mm)')
%                         xlabel([obj.joint_obj{muscles(2,i)}.name(4:end),' Angle (deg)'])
%                         subplot(4,1,4)
%                         anglenorm = (moment_arm_profile(:,2)-min(moment_arm_profile(:,2)))/(max(moment_arm_profile(:,2))-min(moment_arm_profile(:,2)));
%                         momentnorm = (moment_arm_profile(:,3)-min(moment_arm_profile(:,3)))/(max(moment_arm_profile(:,3))-min(moment_arm_profile(:,3)));
%                         plot(anglenorm,'LineWidth',2)
%                         hold on
%                         plot(momentnorm,'LineWidth',2)
%                         grid on
%                         ylabel('Normalized Parameter')
%                         xlabel('Stride Percent')
%                         %legend({'Normalized Angle','Normalized Moment Arm Length'},'Location','southoutside')
%                         set(gcf,'Position',[600 50 500 900])
%                         saveas(gcf,['G:\My Drive\Rat\BiomimeticsPaper','\whole_',obj.musc_obj{muscles(1,i)}.muscle_name(4:end),'_',datestr(datetime('now'),'yyyymmdd'),'.png'])
% %%
%                         for k = 1:3
%                             relevant_muscles = [];
%                             for ii = 1:38
%                                 attachment_bodies = cell2mat(obj.musc_obj{ii}.pos_attachments(:,3));
%                                 if joint == 1 
%                                     if attachment_bodies(1) == 1
%                                         relevant_muscles = [relevant_muscles;ii];
%                                     end
%                                 elseif joint == 2
%                                     if attachment_bodies(end) == 3
%                                         relevant_muscles = [relevant_muscles;ii];
%                                     end
%                                 elseif joint == 3
%                                     if attachment_bodies(end) == 4
%                                         relevant_muscles = [relevant_muscles;ii];
%                                     end
%                                 end
%                             end
%                             figure
%                             for l=1:length(relevant_muscles)
%                                 if l == 1
%                                     [moment_arm_profile,~] = compute_muscle_moment_arm(obj,obj.musc_obj{relevant_muscles(l)},axis,k,0,0);
%                                     anglenorm = (moment_arm_profile(:,2)-min(moment_arm_profile(:,2)))/(max(moment_arm_profile(:,2))-min(moment_arm_profile(:,2)));
%                                     momentnorm = (moment_arm_profile(:,3)-min(moment_arm_profile(:,3)))/(max(moment_arm_profile(:,3))-min(moment_arm_profile(:,3)));
%                                     deriv = diff(momentnorm);
%                                     if (momentnorm(50)-momentnorm(1)) < 0
%                                         if .5*(momentnorm(40)+momentnorm(60)) > .15
%                                             if momentnorm(1) < .55
%                                                 subplotter = 5;
%                                             else
%                                                 subplotter = 4;
%                                             end
%                                         else
%                                             subplotter = 3;
%                                         end
%                                     else
%                                         subplotter = 2;
%                                     end
%                                     subplot(5,1,1)
%                                     plot(anglenorm,'LineWidth',2)
%                                     subplot(5,1,subplotter)
%                                     plot(momentnorm,'LineWidth',2)
%                                     hold on
%                                 else
%                                     [moment_arm_profile,~] = compute_muscle_moment_arm(obj,obj.musc_obj{relevant_muscles(l)},axis,k,0,0);
%                                     momentnorm = (moment_arm_profile(:,3)-min(moment_arm_profile(:,3)))/(max(moment_arm_profile(:,3))-min(moment_arm_profile(:,3)));
%                                     deriv = diff(momentnorm);
%                                     if (momentnorm(50)-momentnorm(1)) < 0
%                                         if .5*(momentnorm(40)+momentnorm(60)) > .15
%                                             if momentnorm(1) < .55
%                                                 subplotter = 5;
%                                             else
%                                                 subplotter = 4;
%                                             end
%                                         else
%                                             subplotter = 3;
%                                         end
%                                     else
%                                         subplotter = 2;
%                                     end
%                                     subplot(5,1,subplotter)
%                                     plot(momentnorm,'LineWidth',2)
%                                     hold on
%                                 end
%                                 grid on
%                                 ylabel('Normalized Parameter')
%                                 xlabel('Stride Percent')
%                                 %legend({'Normalized Angle','Normalized Moment Arm Length'},'Location','southoutside')
%                             end
%                        end
                        %%
            end
        end
    end
end