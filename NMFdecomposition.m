function [r2scores,recompiled,W,H] = NMFdecomposition(k,forces,to_plot)    
    if size(forces,1)>size(forces,2)
        forces = forces';
    end
    forces(forces<0)=0;
    m = size(forces,1);
    n = size(forces,2);
    %k = 4;
    gif_plot = 0;
    
    W = rand(m,k);
    backupW = W;
    %H = rand(k,n);
    lamH = .01;
    lamW = .01;
    %% Plotting GIFs
    if gif_plot
        close all
        activationfig = figure('color','white','Position',[100 500 700 500]);
        weightingfig = figure('color','white','Position',[900 500 800 500]);
        recompfig = figure('color','white','Position',[50 50 1000 600]);
        count = 0;
        act_filename = ['G:\My Drive\Rat\SynergyControl\OutputFigures\Gifs\',datestr(datetime('now'),'yyyymmdd'),'_','cprocess.gif'];
        wgt_filename = ['G:\My Drive\Rat\SynergyControl\OutputFigures\Gifs\',datestr(datetime('now'),'yyyymmdd'),'_','wgtprocess.gif'];
        recom_filename = ['G:\My Drive\Rat\SynergyControl\OutputFigures\Gifs\',datestr(datetime('now'),'yyyymmdd'),'_','recprocess.gif'];
    end
    %%
    for i = 1:10e2
        % Multiplicative update algorithm for NMF in Berry 2006, "Algorithms and applications..."
% %         H = H.*(W'*forces)./(W'*W*H+10^-9);
% %         holder = H*H';
% %         W = W.*(forces*H')./(W*holder+10^-9);
        % ACLS algorithm for NMF from Langville 2014, "Algorithms , Initializations,..."
        % forces bit better than multiplicative update because it results in a less noisy H array
        % Additional degrees of freedom w lambda values, which affect sparsity of W and H. Ultimately little effect for this work
        % since the input arrays are not as sparse as, say, arrays for speech detection. Generally, keep lambdas [0,1]
            H = linsolve(W'*W+lamH*eye(k),W'*forces);
            H(H<0) = 0;
            W = linsolve(H*H'+lamW*eye(k),H*forces')';
            W(W<0) = 0;
            % Sometimes, the solution to the local minimum equations will result in synergies being unused. To prevent this, we "bump" the offending synergy
            % with a random vector and continue the minimization
            if any(mean(W)==0)
                % find the W columns that are zero
                zW = find(mean(W)==0);
                % for each one, fill it with a random vector
                for jj = 1:length(zW)
                    W(:,zW(jj)) = rand(m,1);
                end
            end
        %%
        if mod(i,20) == 0 && gif_plot
            count = count + 1;
            relW = W./max(W);
            bigH = H'.*max(W);
            figure(activationfig)
%             plot(linspace(0,100,size(H,2)),H','LineWidth',2)
            plot(linspace(0,100,size(H,2)),bigH,'LineWidth',2)
            if count == 1
                ymax_act = 1.1*max(max(bigH));
            end
            ylim([0 ymax_act])
            drawnow
            title({'Synergy Activation Level';['NNMF Iteration: ',num2str(i)]},'FontSize',16)
            
            clf(weightingfig)
            figure(weightingfig)
            ha = tight_subplot(5,1,.05,.12,.1);
            for k = 1:size(W,2)
                axes(ha(k));
                holder = relW(:,k);
                %subplot(size(W,2),1,k)
                bar(holder)
                if k == 1
                    title({'Relative Activation of Individual Muscles';['NNMF Iteration: ',num2str(i)]},'FontSize',16)
%                     ymax = 1.1*max(W);
                elseif k == 5
                    xlabel('Muscle #')
                end
                xlim([0 39])
%                 ylim([0 ymax(k)])
                ylim([0 1.1])
                ylabel('Activation')
            end
                
            clf(recompfig)
            figure(recompfig)
            subplot(2,1,1)
            plot(forces')
            subplot(2,1,2)
            recompiled = (relW*bigH')';
            plot(recompiled)
            
            for j = 1:3
               switch j
                   case 1
                       fighand = activationfig;
                       filename = act_filename;
                   case 2
                       fighand = weightingfig;
                       filename = wgt_filename;
                   case 3
                       fighand = recompfig;
                       filename = recom_filename;
               end
              % Capture the plot as an image 
              frame = getframe(fighand); 
              im = frame2im(frame); 
              [imind,cm] = rgb2ind(im,256); 
              % Write to the GIF File 
              if count == 1 
                  imwrite(imind,cm,filename,'gif', 'Loopcount',inf); 
              else 
                  imwrite(imind,cm,filename,'gif','WriteMode','append'); 
              end
            end
        end
        %%
    end
    
    % r^2 value representing Pearson correlation factor
    r2scores = zeros(size(W,1),1);
    % VAF anonymous function as described by Torres-Olviedo 2006, based on uncentered Pearson correlation coefficient
    % Seems to be much more lenient about correlation verification than the r^2 values
    VAF = @(x,y) (1/length(x))*sum((x./sqrt((1/length(x))*sum(x.^2))).*(y./sqrt((1/length(y))*sum(y.^2))));
    recompiled = (W*H)';
    for i = 1:size(W,1)
        X = recompiled(:,i);
        Y = forces(i,:)';
        rho = corr(X,Y,'Type','Pearson')^2;
        %rho = VAF(X,Y);
        r2scores(i,1) = rho;
    end
    
    if any(mean(W)==0)
        keyboard
    end
    
     if to_plot
        plotWH(forces,W,H,0);
     end
end