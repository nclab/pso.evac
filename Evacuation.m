function Evacuation(scene_num , outfile , plot_type)
%   Evacuation simulator
%   scene_num : indicate the scene number for loading
%   outfile   : save simulation data
%   plot_type : indicate the type of display
%   0 -> None 
%   1 -> plot auto
%   2 -> plot with pause
%   3 -> record film

%   Load scene parameters
    Data = Scenes(scene_num);    
    Area = Data.Area;
    Goal = Data.Goal;
    Areas = Area.^2;
    obstacles = Data.obs';    
    escape_count = 0;
    
    lights = Data.lights';
    light_size = length(lights);    
    
%   PSO parameters
    MaxGen = 3000;
    popsize = Data.Pops;
    stepsize = 5;    
    
%   PSO parameters
    w = 1;
    c1 = 0.2;
    c2 = 1;
%   Approach parameters
    p_sigma = 5;
    C_obs = 0.1;    

%   save .AVI file & plot figure
    if(plot_type>0)
        fig = figure();
        set(gca,'unit','pixels');
        set(gcf,'Position',[100 50 1080 800]);
        set(gca,'Position',[100 100 Area*1.5]);
        box on;
        hold on;
    end
    if(plot_type == 3)
        m = VideoWriter([outfile '.avi']);
        open(m);
    end        
    
%   Data logger
    init_struct = struct('px',[],'py',[]);
    His.pops = repmat(init_struct , popsize , 1 );
    His.escape = zeros(1,popsize);
    His.escape_t = MaxGen*ones(1,popsize);
    
%%%% simulation start %%%%

    %   ---- particle init ----
    disp('Initializing...');
    init_particle = struct('pos',zeros(1,2),'dir',zeros(1,2),'speed',0,'fitness',0,'LBF',inf,'LBP',zeros(1,2),'GBF',inf,'GBP',zeros(1,2),'penalty',1,'escape',0,'t',0);
    pops = repmat(init_particle , popsize , 1 );
    
    %   init position & direction
    for i=1:popsize
        rpos = rand(1,2).*(Area-20)+10;
        while(init_pos_check(rpos(1) , rpos(2) , i))
            rpos = rand(1,2).*(Area-20)+10;
        end
        pops(i).pos = rpos;
        r = 2*pi*rand();
		pops(i).dir = [cos(r) sin(r)];
    end
    
    %   init penalty & fitness    
    for i=1:popsize
        pops(i).penalty = cost(i , pops(i).pos);
        evaluation(i);
    end
    if(plot_type==1)
        plot_frame;
        pause(0.1);
    elseif(plot_type==2)
        plot_frame;
        pause;
    elseif(plot_type==3)
        plot_frame;
    end
        
    %   ---- evoluation start ----
    disp('Simulating...');
    for g=1:MaxGen
        if(light_size>0)
            instruction();
        end
        for i=1:popsize
            if(pops(i).escape)
                continue;
            end
            position_update(i);
            evaluation(i);
            escape(i);
        end
        % record the history
        for i=1:popsize
            His.pops(i).px(g) = pops(i).pos(1);
            His.pops(i).py(g) = pops(i).pos(2);
        end
        
        if(plot_type==1)
            plot_frame
            pause(0.2);
        elseif(plot_type==2)
            plot_frame
            pause;
        elseif(plot_type==3)
            plot_frame
        end           
        if(escape_count==popsize)
            break;
        end
    end
    %   ---- evoluation end ----

%%%%  output data %%%%
    for i=1:popsize
        His.escape(i) = pops(i).escape;
        His.escape_t(i) = pops(i).t;        
    end
    His.maxgen = g;
    save(['data\' outfile '.mat'] , 'His' , 'scene_num');
    if(exist('fig','var'))
        close(fig);
    end
    if(plot_type==3)
        close(m);
    end
    
    disp('Simulation end.');
%%%%  subfunction routines %%%%
    function fit = fitness(pos)
        fit = exp(sum(((pos - Goal).^2)./Areas));
    end
    % evaluation function 
    function evaluation(p)
        pops(p).fitness = C_obs*pops(p).penalty + fitness(pops(p).pos);

        pops(p).speed = stepsize*(1-pops(p).penalty);
        pops(p).GBF = pops(p).GBF*1.05;
        % update local best & global best in view
        if(pops(p).fitness < pops(p).LBF)
            pops(p).LBP = pops(p).pos;
            pops(p).LBF = pops(p).fitness;
        end
        if(pops(p).fitness < pops(p).GBF)
            pops(p).GBP = pops(p).pos;
            pops(p).GBF = pops(p).fitness;
        end                    
        for indiv=[1:p-1 p+1:popsize]
            if(pops(p).fitness < pops(indiv).GBF)
                if(see(pops(p).pos , pops(indiv).pos))
                    pops(indiv).GBP = pops(p).pos;
                    pops(indiv).GBF = pops(p).fitness;
                end
            end
        end
    end

    function escape(p)
        if(fitness(pops(p).pos)<1.001)
            escape_count = escape_count + 1;
            pops(p).pos = -Area;
            pops(p).escape = 1;
            pops(p).t = g;
        end        
    end

    function position_update(p)
        v1 = pops(p).LBP - pops(p).pos;
        if(any(v1)), v1 = v1 / norm(v1); end
        v2 = pops(p).GBP - pops(p).pos;
        if(any(v2)), v2 = v2 / norm(v2); end
        if(~any(v1-v2))
            new_d = w.*pops(p).dir;
        else
            new_d = w.*pops(p).dir + c1*rand().*v1 + c2*rand().*v2;
        end        
        new_d = new_d/norm(new_d);        
        new_p = pops(p).pos + pops(p).speed.*new_d;        
        % penalty due to the obs
        new_penalty = cost(p , new_p);
        if ( accept_test(new_penalty , pops(p).penalty) )
            pops(p).dir = new_d;
            pops(p).pos = new_p;
            pops(p).penalty = new_penalty;
        else
            local_search(p , new_d , pops(p).penalty);
        end
    end

    %local search
    function local_search(p , new_dir , new_penalty)        
        angle = 20;
        speed = pops(p).speed;
        for t=1:18
            r = angle*(rand()-0.5)*(pi/180);
            v_dir(1) = new_dir(1)*cos(r) - new_dir(2)*sin(r);
            v_dir(2) = new_dir(1)*sin(r) + new_dir(2)*cos(r);
            v_pos = pops(p).pos + speed*v_dir;
            v_cost = cost(p , v_pos);
            if (accept_test(v_cost , new_penalty))
                pops(p).pos = v_pos;
                pops(p).dir = v_dir;
                pops(p).speed = speed;
                pops(p).penalty = v_cost;           
                break;
            else
                angle = angle+20;
                speed = speed*0.9;
            end          
        end
    end

    function maxcost = cost(p , position)
        maxcost = 0;
        % 	static obstacles
        for obs = obstacles
            if( obs.type <= 2)        % rectangle
                cost = min(...
                    exp( -sum((position(1) - obs.x).^2)/((obs.sigma_x + p_sigma)^2) ),...
                    exp( -sum((position(2) - obs.y).^2)/((obs.sigma_y + p_sigma)^2) )...
                );
            elseif(obs.type == 3)     % circle
                cost = exp( -sum((position - [obs.x obs.y]).^2)/((obs.sigma + p_sigma)^2) );
            end
            maxcost = max(maxcost , cost);
        end
        % 	dinamic obstacles
        for indiv=([1:p-1 p+1:popsize])
            if(~pops(indiv).escape)
                cost = exp( -sum((position - pops(indiv).pos).^2)/((2*p_sigma)^2) );
                maxcost = max(maxcost , cost);            
            end
        end        
    end

    function bool=accept_test(new_penalty , old_penalty)
        bool = 0;
        if(new_penalty < old_penalty)
            bool=1;
        elseif(rand > new_penalty*exp(1))
            bool=1;
        end
    end

    % check the collision while initializing 
    function a=init_pos_check(x,y,i)
        a = 0;
        for obs=obstacles
            if(obs.type < 3)
                if( abs(x-obs.x)<obs.sigma_x+p_sigma && abs(y-obs.y)<obs.sigma_y+p_sigma)
                    a = 1;
                    return;
                end
            elseif(obs.type == 3)
                if(norm([x-obs.x , y-obs.y])<obs.sigma+p_sigma)
                    a = 1;
                    return;
                end
            end
        end
        for p=1:i-1
            if(norm([x-pops(p).pos(1) , y-pops(p).pos(2)]) < 2*p_sigma)
                a = 1;
                return;
            end
        end
    end
    
    % check the view of particle
    function a=see(Pa , Pb)
        a = 1;
        for obs = obstacles
            if(obs.type==2)
                if( Pa(1)<obs.x-obs.sigma_x && Pb(1)<obs.x-obs.sigma_x), continue; end;
                if( Pa(1)>obs.x+obs.sigma_x && Pb(1)>obs.x+obs.sigma_x), continue; end;
                if( Pa(2)<obs.y-obs.sigma_y && Pb(2)<obs.y-obs.sigma_y), continue; end;
                if( Pa(2)>obs.y+obs.sigma_y && Pb(2)>obs.y+obs.sigma_y), continue; end;                
                if( cross_test(Pa , Pb , obs.points) ), continue; end;
                a = 0;
                return;                
            elseif(obs.type==3)
                p1 = Pb(2)-Pa(2);
                p2 = Pa(1)-Pb(1);
                p3 = Pb(1)*Pa(2)-Pa(1)*Pb(2);
                if( (abs(p1*obs.x+p2*obs.y+p3)/norm([p1, p2])) > obs.sigma ), continue; end;
                A = Pa - [obs.x obs.y];
                B = Pa - Pb;
                if( acosd(dot(A,B)/norm(A)/norm(B)) > 90) , continue; end;
                A = Pb - [obs.x obs.y];
                B = Pb - Pa;
                if( acosd(dot(A,B)/norm(A)/norm(B)) > 90) , continue; end;
                a = 0;
                return;
            end
        end
    end
   
    function instruction()
        for light = lights
            for indiv=1:popsize
                if(light.fit < pops(indiv).GBF)
                    if(see(light.pos , pops(indiv).pos))
                        pops(indiv).GBP = light.pos;
                        pops(indiv).GBF = light.fit;
                    end
                end
            end
        end
    end

    function plot_frame()
        cla();
        for v_light=lights
            r = rectangle('Position',[v_light.pos(1)-3 v_light.pos(2)-3 6 6],'Curvature',[1,1]);
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
            r = rectangle('Position',[Goal(1)-1 Goal(2)-10 , 5 , 20]);
            set(r,'FaceColor','r');
        else
            r = rectangle('Position',[Goal(1)-10 Goal(2)-1 , 20 , 5]);
            set(r,'FaceColor','r');        
        end       
        
        for l=1:popsize
            if(pops(l).escape)
                continue;
            end
            plot(pops(l).LBP(1),pops(l).LBP(2),'g+');
            plot(pops(l).GBP(1),pops(l).GBP(2),'rs');
            plot( [pops(l).pos(1) pops(l).GBP(1)] , [pops(l).pos(2) pops(l).GBP(2)] , 'b:');            
            rectangle('Position',[pops(l).pos(1)-p_sigma pops(l).pos(2)-p_sigma 2*p_sigma 2*p_sigma],'Curvature',[1,1])                
            line([pops(l).pos(1) pops(l).pos(1)+pops(l).dir(1)*10],[pops(l).pos(2) pops(l).pos(2)+pops(l).dir(2)*10])
        end
        xlim([-5,Area(1)])
        ylim([-5,Area(2)])                

        if(plot_type==3)
            F = getframe(gca);            
            writeVideo(m,F);
        end
    end
end

function cross=cross_test(Pa,Pb, points)
    p1 = Pb(2)-Pa(2);
    p2 = Pa(1)-Pb(1);
    p3 = Pb(1)*Pa(2)-Pa(1)*Pb(2);
    sign = zeros(1,size(points,1));
    for p=1:size(points,1)
        sign(p) = ( p1*points(p,1) + p2*points(p,2) + p3 );
    end
    if(all(sign>0) || all(sign<0))
        cross = 1;
    else
        cross = 0;
    end
end

