function Path_plot(datafile)
%   datafile : load the file of simulation data
%
    load(['data\' datafile '.mat']);
    fprintf('Avg escape time : %ld\n',mean(His.escape_t));
    fprintf('Max escape time : %ld\n',max(His.escape_t));    
    Scene_plot(scene_num);   
    for i=1:length(His.pops)
        x = His.pops(i).px;
        y = His.pops(i).py;
        index = (x>0);
        plot(x(index),y(index),'-');
    end       
end