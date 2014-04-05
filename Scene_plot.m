function fig = Scene_plot(s)

    Data = Scenes(s);
    
    Area = Data.Area;
    obstacles = Data.obs';
    lights = Data.lights';
    Goal = Data.Goal;
    fig = figure();
    set(gcf, 'PaperPositionMode','manual');
    set(gca,'unit','pixels');    
    set(gcf,'Position',[100 50 1080 800]);
    set(gca,'Position',[100 100 Area*1.5]);    
    hold on;

    for v_light=lights
        r = rectangle('Position',[v_light.pos(1)-5 v_light.pos(2)-5 10 10],'Curvature',[1,1]);
        set(r,'FaceColor','r');
    end
    
    for v_obs=obstacles
        if( v_obs.type == 1)         % wall
            r = rectangle('Position',[v_obs.x-v_obs.sigma_x v_obs.y-v_obs.sigma_y 2*v_obs.sigma_x 2*v_obs.sigma_y]);
            set(r,'FaceColor',[0.4 0.4 0.4],'LineStyle','none');
        elseif(v_obs.type == 2)      % rectangle
            r = rectangle('Position',[v_obs.x-v_obs.sigma_x v_obs.y-v_obs.sigma_y 2*v_obs.sigma_x 2*v_obs.sigma_y]);
            set(r,'LineWidth',1);
        elseif(v_obs.type == 3)      % circle
            r = rectangle('Position',[v_obs.x-v_obs.sigma v_obs.y-v_obs.sigma 2*v_obs.sigma 2*v_obs.sigma],'Curvature',[1,1]);
            set(r,'LineWidth',1);
        end
    end    
    if(Goal(1)==11)
        r = rectangle('Position',[Goal(1)-1 Goal(2)-15 , 8 , 30]);
        set(r,'FaceColor','r');
    else
        r = rectangle('Position',[Goal(1)-15 Goal(2)-1 , 30 , 8]);
        set(r,'FaceColor','r');        
    end        
    xlim([-5,Area(1)])
    ylim([-5,Area(2)])
    
end