function sim_eqn = sum_of_sines_maker(coefficients,project_file)
    % project_file is a boolean that determines hwether or not the output function is for a sim or project file
    % The equations for these two different file types are quite different
    if project_file
        % This section allows the user to create a sum of sines equation in the format used by the .aproj files from Animatlab
        % For automated simulations, most injections occur in the .asim file, instead, which uses a different format
        syms a b c t
        eqn = sym(zeros(length(coefficients)/3,1));
        count = 0;
            for i=1:3:length(coefficients)
                count = count + 1;
                a = coefficients{i,2};
                if a < 0
                    disp(['Warning: an ''a'' variable is negative. Animatlab doesn''t like this.',... 
                        'Consider reducing number of sine waves such that you don''t need negative ''a'' values'])
                end
                b = coefficients{i+1,2};
                c = coefficients{i+2,2};
                eqn(count,1) = vpa(a*sin(b*t+c),5);
            end
            sim_eqn = vpa(sum(eqn(1:end)),5);
    else
        % This section creates sum of sines equations formatted for .asim files
        % These equations are more compact than the .aproj counterparts and store sine algebra in a "tail" refered here as the plus_trail
        sim_eqn = [];
        plus_trail = [];

        for j=1:3:length(coefficients)
            a = coefficients{j,2};
            b = coefficients{j+1,2};
            c = coefficients{j+2,2};
            if a < 0
                astring = ['0,',num2str(abs(a)),','];
            else
                astring = [num2str(a),','];
            end
            if b < 0
                bstring = ['0,',num2str(abs(b)),','];
                sinstring = '-,sin,*,';
            else
                bstring = [num2str(b),','];
                sinstring = 'sin,*,';
            end
            if c < 0
                cstring = [num2str(abs(c)),',-,'];
            else
                cstring = [num2str(c),',+,'];
            end
            sim_eqn = [sim_eqn,astring,bstring,'t,*,',cstring,sinstring];
            if j > 1
                if a > 0
                    plus_trail = [plus_trail,'+,'];
                else
                    plus_trail = [plus_trail,'-,'];
                end
            end
        end
        sim_eqn = [sim_eqn,plus_trail(1:end-1)];   
    end   
end