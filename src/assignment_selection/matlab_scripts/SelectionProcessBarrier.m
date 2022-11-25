function ObjectOrders = SelectionProcessBarrier(trgts,botbases,basket,BrickNo,a1,c1)%n=number of objects,trgts=taget objects, botbases = robots base locations,basket=tower coordinates, direction = "inwards"or"outwards", method = "solo"or "group"
    %Author: Steven Craig    Date:25/11/2022
    % SelectionProcessBarrier selects the bricks that should be picked up
    % by the robot arm.
    %It outputs this selection in the ObjectOrders matrix.
    %The matrix contains the distance corresponding to each brick
    %IT does this by running the search and selecting the nearest block at
    %each stage AND is withing its barrier space.
    %
    % Call the fucntion by runnung a command simmilar to this
    % ObjectOrder = SelectionProcess(BrickLoactions,RobotBasesLoactons,EndLocation,BrickNumber)
    % 
    % Each input in the form
    % BrickLoactions [x1,y1;x2,y2;...] - the x,y quardiats for each brick
    % RobotBasesLoactons [x1,y1;x2,y2;...] - x,y quardinats represeting
    % where the bace of the bots are
    % EndLocation [x1,y1] - x,y quardinats represerting the drop off
    % location
    % BrickNumber[a,b,c,...] - The intiget represetning each brick
    %
    % 
    % ObjectOrder = [dist, name, dist , name, ....;...]
    n = length(BrickNo);
    objs = zeros(n,2);
    objs(1:n,1:2) = trgts; 
    botnumbers = length(botbases)/2; %number of robots used
    firstlocations = zeros(n,2*botnumbers); %array of distances from robot to obj
    normaldistances = zeros(n,2*botnumbers); %array of distacnes from baskrt to obj
    objectangels = zeros(n,2*botnumbers); %angle from bot to object
    closestobj = zeros(botnumbers,1);
    for i=1:n %fills in object distance for each robot
        %gets the distance from robot base to each brick.
        %get the distance form the basket to each brick.
        for x = 1:botnumbers
            normaldistances(i,(2*x)-1) = norm(objs(i,:)-basket);
            normaldistances(i,2*x) = i;
            botbase = [botbases((2*x)-1) botbases(2*x)];
            firstlocations(i,(2*x)-1) = norm(objs(i,:)-botbase);
            firstlocations(i,2*x) = i;
            objectangles(i,(2*x)-1) = atan2(objs(i,2)-botbase(2),objs(i,1)-botbase(1));
            objectangles(i,2*x) = i;
        end
        
    end
    %Assigns the first brick to be picked up (closet to robot).
    for x = 1:botnumbers %gets the closest first brick and raplaces normal distance for that brick with this value
        closestobj(x) = min(firstlocations(:,(2*x)-1));
        for i = 1:n
            if closestobj(x) == firstlocations(i,(2*x)-1)
                normaldistances(i,(2*x)-1) = closestobj(x);
            end
        end
    end
    normaldistances = sortrows(normaldistances,2);
    botdist = zeros(botnumbers,1); %array for distances from obj to robots
    botnum = zeros(botnumbers,1);
    CompareArray = zeros(i,3*botnumbers);
    for i = 1:n %checks which robot is close to the obj and gives it the obj (sets value for other robots to 0)
        for x = 1:botnumbers
            botdist(x) = firstlocations(i,(2*x)-1);
            botnum(x) = x; %which bot the distance belongs to
        end
        w = 1:14;
        %This gets the intersect of the line from robotbase to brick and
        %the barrier.
        %If the distance to this intersect is less than the distance to the
        %brick then the brick is out of bounds and taken off that robots
        %pickup list.
        %This is done by setting the brick distance to zero.
        for x = 1:botnumbers
            botbase = [botbases((2*x)-1) botbases(2*x)];
            a = tan(objectangles(i,(2*x)-1));
            c =(botbase(2)-botbase(1)*tan(objectangles(i,(2*x)-1)));
            intersectx = ((c1-c)/(a-a1));
            intersecty = abs((c-c1)/(a-a1));
            %plot(w,a.*w + c);
            plot(w,1*w);
            hold on
            CompareArray(i,(3*x)-2)=norm([intersectx,intersecty]-botbase);
            CompareArray(i,(3*x)-1) = normaldistances(i,(2*x)-1);
            CompareArray(i,3*x) = i;
            if norm([intersectx,intersecty]-botbase) < firstlocations(i,(2*x)-1)
                normaldistances(i,(2*x)-1) = 0;
            end
        end
    end
    ObjectOrders = normaldistances;
    scatter(objs(:,1),objs(:,2))
end
