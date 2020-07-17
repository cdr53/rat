[obj,sim_file,joints,bodies,joint_limits,joint_profile,sdata,passive_tension] = design_synergy("G:\My Drive\Rat\SynergyControl\Animatlab\SynergyWalking\SynergyWalking_EMGwalking_Standalone.asim");

%%
initForcePos_rel = [0;-38.242;-9.6]./1000;
anchorPos = [50.061;-139.07;18.55]./1000;
samp = 5000;
theta = obj.theta_motion(samp,:);

initForcePos = obj.att_pos_on_demand([0 0 0],[{initForcePos_rel},{''},{3},{''}]);
temp1 = find(contains({sdata(:).name},'ForceSensorInfo'));
temp2 = find(contains(sdata(temp1).data_cols,'Ib'));
temp3 = find(contains(sdata(temp1).data_cols,'ml'));
fsData = sdata(temp1).data(:,temp2);
fsDataL = sdata(temp1).data(:,temp3);
clear temp* fX fY fZ

close all
figure('Position',[962,45,958,950])
for ii = floor(linspace(1,18529,50))%floor(linspace(1,9000,50))%3785
    movedForcePos = obj.att_pos_on_demand(obj.theta_motion(ii,:),[{initForcePos_rel},{''},{3},{''}]);

    fVec = movedForcePos-anchorPos;
    fVecnorm = fVec./norm(fVec);

    fX(ii) = fsData(ii)*fVecnorm(1);
    fY(ii) = fsData(ii)*fVecnorm(2);
    fZ(ii) = fsData(ii)*fVecnorm(3);
    
    pospel = obj.organism_position*1000;
    posfem = obj.att_pos_on_demand(obj.theta_motion(ii,:),[{[0;0;0]},{''},{2},{''}])'*1000;
    postib = obj.att_pos_on_demand(obj.theta_motion(ii,:),[{[0;0;0]},{''},{3},{''}])'*1000;
    posfot = obj.att_pos_on_demand(obj.theta_motion(ii,:),[{[0;0;0]},{''},{4},{''}])'*1000;
    postoe = obj.att_pos_on_demand(obj.theta_motion(ii,:),[{[20.452;-6.26;-1.398]./1000},{''},{4},{''}])'*1000;
    limbLine = [pospel;posfem;postib;posfot;postoe];
    plot3(limbLine(:,1),limbLine(:,2),limbLine(:,3))
    hold on
    plot3(movedForcePos(1)*1000,movedForcePos(2)*1000,movedForcePos(3)*1000,'r.','MarkerSize',20)
    % For plotting individual x,y,z lines
    xMat = [movedForcePos'*1000;movedForcePos'*1000+[fX(ii),0,0]*15];
    yMat = [movedForcePos'*1000;movedForcePos'*1000+[0,fY(ii),0]*15];
    zMat = [movedForcePos'*1000;movedForcePos'*1000+[0,0,fZ(ii)]*15];
    plot3(xMat(:,1),xMat(:,2),xMat(:,3),'k','LineWidth',2)
    plot3(yMat(:,1),yMat(:,2),yMat(:,3),'b','LineWidth',2)
    plot3(zMat(:,1),zMat(:,2),zMat(:,3),'g','LineWidth',2)
    % For plotting the combined vector
%     fMat = [movedForcePos'*1000;movedForcePos'*1000+[fX(ii),fY(ii),fZ(ii)]*10];
%     plot3(fMat(:,1),fMat(:,2),fMat(:,3))
    % For xy and yz planes
%     xyMat = [movedForcePos'*1000;movedForcePos'*1000+[fX(ii),fY(ii),0]*10];
%     yzMat = [movedForcePos'*1000;movedForcePos'*1000+[0,fY(ii),fZ(ii)]*10];
%     plot3(xyMat(:,1),xyMat(:,2),xyMat(:,3),'k','LineWidth',3)
    for jj = 1:size(obj.musc_obj{4}.pos_attachments,1)
        muscMat(jj,:) = obj.musc_obj{4}.pos_attachments{jj,4}(ii,:)*1000;
    end
    plot3(muscMat(:,1),muscMat(:,2),muscMat(:,3),'m')
    %plot3(yzMat(:,1),yzMat(:,2),yzMat(:,3),'b')
    grid on
    pbaspect([1 1 1])
    xlabel('X');ylabel('Y');zlabel('Z');
    xlim([-50 40]);ylim([-180 -90]);zlim([0 25]);
    view([0 90])
    title(num2str(obj.theta_motion_time(ii,1)),'FontSize',20)
    pause(.2)
    hold off
end