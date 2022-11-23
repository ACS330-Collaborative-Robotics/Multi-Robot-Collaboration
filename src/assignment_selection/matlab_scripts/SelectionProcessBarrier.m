function ObjectOrders = SelectionProcess(n,trgts,botbases,basket,a1,c1)%n=number of objects,trgts=taget objects, botbases = robots base locations,basket=tower coordinates, direction = "inwards"or"outwards", method = "solo"or "group"
    objs = zeros(n,2);
    objs(1:n,1:2) = trgts; 
    botnumbers = length(botbases)/2; %number of robots used
    firstlocations = zeros(n,2*botnumbers); %array of distances from robot to obj
    normaldistances = zeros(n,2*botnumbers); %array of distacnes from baskrt to obj
    objectangels = zeros(n,2*botnumbers); %angle from bot to object
    closestobj = zeros(botnumbers,1);
    for i=1:n %fills in object distance for each robot
        for x = 1:botnumbers
            normaldistances(i,(2*x)-1) = norm(basket-objs(i,:));
            normaldistances(i,2*x) = i;
            botbase = [botbases((2*x)-1) botbases(2*x)];
            firstlocations(i,(2*x)-1) = norm(botbase-objs(i,:));
            firstlocations(i,2*x) = i;
            objectangles(i,(2*x)-1) = atan2(botbase(2)-objs(i,2),botbase(1)-objs(i,1));
            objectangles(i,2*x) = i;
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
            botbase = [botbases((2*x)-1) botbases(2*x)];
            a = tan(objectangles(i,(2*x)-1));
            c =(botbase(2)-botbase(1)*tan(objectangles(i,(2*x)-1)));
            intersectx = ((c1-c)/(a-a1));
            intersecty = ((c-c1)/(a-a1));
            if norm(botbase-[intersectx,intersecty]) < normaldistances(i,(2*x)-1)
                normaldistances(i,(2*x)-1) = 0; %you are here
            end
        end
    end
    ObjectOrders = normaldistances;
    scatter(objs(:,1),objs(:,2))
end