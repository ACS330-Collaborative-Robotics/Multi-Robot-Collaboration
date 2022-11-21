function ObjectOrders = SelectionProcess(trgts,botbases,basket,brickNo)
    % SelectionProcess selects the bricks that should be reached by each
    % arm and outputs it in the object order matrix
    % It does this by running a serch and designating the robots to grab
    % the nearest block to them at each stage. 
    % 
    % Call the fucntion by runnung a command simmilar to this
    % 
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

    n = length(brickNo);
    objs = zeros(n+1,2);
    objs(1:n,1:2) = trgts; 
    botnumbers = length(botbases)/2; %number of robots used
    firstlocations = zeros(n,2*botnumbers); %array of distances from robot to obj
    normaldistances = zeros(n,2*botnumbers); %array of distacnes from baskrt to obj
    closestobj = zeros(botnumbers,1);
    for i=1:n %fills in object distance for each robot
        for x = 1:botnumbers
            normaldistances(i,(2*x)-1) = norm(basket-objs(i,:));
            normaldistances(i,2*x) = i;
            botbase = [botbases((2*x)-1) botbases(2*x)];
            firstlocations(i,(2*x)-1) = norm(botbase-objs(i,:));
            firstlocations(i,2*x) = i; 
        end
    end
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
    
    for i = 1:n %checks which robot is close to the obj and gives it the obj (sets value for other robots to 0)
        for x = 1:botnumbers
            botdist(x) = firstlocations(i,(2*x)-1);
            botnum(x) = x; %which bot the distance belongs to
        end
        for x = 1:botnumbers
            if botdist(x) ~= min(botdist)
               normaldistances(i,(2*x)-1) = 0;
            end
        end
    end
    ObjectOrders = normaldistances';
    ObjectOrders(2,:) = brickNo;ObjectOrders(3,:) = brickNo;ObjectOrders = ObjectOrders';
    % scatter(objs(:,1),objs(:,2))
end