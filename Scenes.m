function D = Scenes(s)
    if(s==1)
        [A,G,P, O,L] = scene1();
    elseif(s==2)
        [A,G,P, O,L] = scene2();
    elseif(s==3)
        [A,G,P, O,L] = scene3();
    elseif(s==4)
        [A,G,P, O,L] = scene4();
    else
        return;
    end
    
    % default alpha = 1
    init_struct = struct('type',0,'x',0,'y',0,'sigma',0,'sigma_x',0,'sigma_y',0,'alpha',1,'points',zeros(4,2));    
    obs = repmat(init_struct , size(O,1) , 1 );
    for i=1:size(O,1)
        obs(i).type = O(i,1);
        obs(i).x = O(i,2);
        obs(i).y = O(i,3);
        if(O(i,1) < 3 )
            obs(i).sigma_x = O(i,4);
            obs(i).sigma_y = O(i,5);
            obs(i).points(1,:) = [obs(i).x-obs(i).sigma_x obs(i).y-obs(i).sigma_y];
            obs(i).points(2,:) = [obs(i).x-obs(i).sigma_x obs(i).y+obs(i).sigma_y];
            obs(i).points(3,:) = [obs(i).x+obs(i).sigma_x obs(i).y-obs(i).sigma_y];
            obs(i).points(4,:) = [obs(i).x+obs(i).sigma_x obs(i).y+obs(i).sigma_y];
        elseif(O(i,1) == 3 )
            if( O(i,4) ~= O(i,5))
                disp('Nonequal size of a circle');
                return;
            end
            obs(i).sigma = O(i,4);
        else
            disp('Error obs type');
            return;
        end
    end

    init_struct = struct('pos',[],'fit',0);
    lights = repmat(init_struct , size(L,1) , 1 );
    for i=1:size(L,1)
        lights(i).pos = [L(i,1) L(i,2)];
        lights(i).fit = exp(sum((lights(i).pos - G).^2)/sum(A.^2))/exp(1);
    end
    
    D.Area = A;
    D.Goal = G;
    D.Pops = P;
    D.obs = obs;
    D.lights = lights;
end

function [Area, Goal, Pops, obs , light]=scene1()    
    Area = [600 400];
    Goal = [30 11];
    Pops = 50;
    % obs data array
    % type , center_x , center_y , half_width , half_hight
    % type = 1 -> wall
    % type = 2 -> retangle
    % type = 3 -> circle
    obs = [
        1   ,   300 ,   5   ,   300 ,   5 ;
        1   ,   300 ,   395 ,   300 ,   5;
        1   ,   5   ,   200 ,   5   ,   200;
        1   ,   595 ,   200 ,   5   ,   200;
                        
        2   ,   120   ,   320 ,   70   ,   40;
        2   ,   480   ,   80 ,   70   ,   40;
        
        2   ,   150   ,   80 ,   70   ,   40;
        2   ,   450   ,   320 ,   70   ,   40;
                
        2   ,   230   ,   180 ,   50   ,   40;
        2   ,   370   ,   220 ,   50   ,   40;
        
        2   ,   100   ,   200 ,   40   ,   40;
        2   ,   500   ,   200 ,   40   ,   40;
        
        3   ,   330   ,   90 ,   50   ,   50;
        3   ,   270   ,   310 ,   50   ,   50;

    ];

    % light point, the only light is exit
    light = [
        30  , 11;
    ];
end

function [Area, Goal,Pops, obs , light]=scene2()    
    Area = [600 400];
    Goal = [300 11];
    Pops = 50;
    % obs data array
    % type , center_x , center_y , half_width , half_hight
    % type = 1 -> wall
    % type = 2 -> retangle
    % type = 3 -> circle
    obs = [
        1   ,   300 ,   5   ,   300 ,   5 ;
        1   ,   300 ,   395 ,   300 ,   5;
        1   ,   5   ,   200 ,   5   ,   200;
        1   ,   595 ,   200 ,   5   ,   200;
                        
        2   ,   120   ,   320 ,   70   ,   40;
        2   ,   480   ,   80 ,   70   ,   40;
        
        2   ,   150   ,   80 ,   70   ,   40;
        2   ,   450   ,   320 ,   70   ,   40;
                
        2   ,   230   ,   180 ,   50   ,   40;
        2   ,   370   ,   220 ,   50   ,   40;
        
        2   ,   100   ,   200 ,   40   ,   40;
        2   ,   500   ,   200 ,   40   ,   40;
        
        3   ,   330   ,   90 ,   50   ,   50;
        3   ,   270   ,   310 ,   50   ,   50;
    ];

    % light point, the only light is exit
    light = [
        300  , 11;
    ];
end

function [Area, Goal,Pops, obs , light]=scene3()
    Area = [600 400];
    Goal = [30 11];
    Pops = 50;
    % obs data array
    % type , center_x , center_y , half_width , half_hight
    % type = 1 -> wall
    % type = 2 -> retangle
    % type = 3 -> circle
    obs = [
        1   ,   300 ,   5   ,   300 ,   5 ;
        1   ,   300 ,   395 ,   300 ,   5;
        1   ,   5   ,   200 ,   5   ,   200;
        1   ,   595 ,   200 ,   5   ,   200;
                        
        2   ,   140   ,   320 ,   70   ,   40;
        2   ,   470   ,   80 ,   70   ,   40;
        
        2   ,   140   ,   80 ,   70   ,   40;
        2   ,   470   ,   320 ,   70   ,   40;
                
        2   ,   200   ,   200 ,   50   ,   40;
        2   ,   410   ,   200 ,   50   ,   40;
        
        2   ,   110   ,   200 ,   40   ,   40;
        2   ,   500   ,   200 ,   40   ,   40;
        
        3   ,   300   ,   90 ,   50   ,   50;
        3   ,   300   ,   310 ,   50   ,   50;
    ];
    % light point, the only light is exit
    light = [
        30  , 11;
    ];
end

function [Area, Goal,Pops, obs , light]=scene4()    
    Area = [600 400];
    Goal = [30 11];
    Pops = 50;
    % obs data array
    % type , center_x , center_y , half_width , half_hight
    % type = 1 -> wall
    % type = 2 -> retangle
    % type = 3 -> circle
    obs = [
        1   ,   300 ,   5   ,   300 ,   5 ;
        1   ,   300 ,   395 ,   300 ,   5;
        1   ,   5   ,   200 ,   5   ,   200;
        1   ,   595 ,   200 ,   5   ,   200;
                        
        2   ,   120   ,   320 ,   70   ,   40;
        2   ,   480   ,   80 ,   70   ,   40;
        
        2   ,   150   ,   80 ,   70   ,   40;
        2   ,   450   ,   320 ,   70   ,   40;
                
        2   ,   230   ,   180 ,   50   ,   40;
        2   ,   370   ,   220 ,   50   ,   40;
        
        2   ,   100   ,   200 ,   40   ,   40;
        2   ,   500   ,   200 ,   40   ,   40;
        
        3   ,   330   ,   90 ,   50   ,   50;
        3   ,   270   ,   310 ,   50   ,   50;
    ];

    % light point, the first light is exit
    light = [
        30  , 11;
        11  , 110;
        230  , 11;
        11 ,  270;
        400 , 11;
        11 , 380;
        570 , 11;
    ];
end
