    %% defining random bricks and arm base locations
%the aim of this software is to develop basic brick selection logic

n=16; %number of bricks
testpts(1:n,1:2) =10*rand(n,2); testpts(1:n,1)=testpts(1:n,1)+2;
%random points x 2-12 y 0-10
scatter(testpts(:,1),testpts(:,2))
base1=[0,5]; %arm 1 base 
base2=[14,5];%arm 2 base 
basket=[7,5];%block plasing area
hold on
scatter(base1(1,1),base1(1,2),150,'d');scatter(base2(1,1),base2(1,2),150,'d');%plot arm bases
scatter(testpts(:,1),testpts(:,2))%plot bricks
scatter(basket(1,1),basket(1,2),40,"red");
%% selection logic
%the arms will place a red/green circle on the bricks they have just
%selected. They can both select one brick per time tick
%% method 1:nearest
brickdist1=zeros(n,2);
brickdist2=zeros(n,2);
brickangle1 = 1:n;
brickdist1x = 1:n;
brickdist1y = 1:n;
brickdistfirst1 = zeros(n,2);
brickdistfirst2 = zeros(n,2);
for i=1:n
    brickdistfirst1(i,1)= norm(base1(1,:)-testpts(i,:));%distance between base and point
    brickdistfirst2(i,1)= norm(base2(1,:)-testpts(i,:));%distance between bease and point
    brickdist1(i,1)= norm(basket(1,:)-testpts(i,:));%distance between arm 1 and point
    brickdist2(i,1)= norm(basket(1,:)-testpts(i,:));%distance between arm 2 and point
    brickdist1(i,2)= i;%taking note of which block it is
    brickdist2(i,2)=i;
    brickdistfirst1(i,2)= i;%taking note of which block it is
    brickdistfirst2(i,2)=i;
    brickdist1x(i) = brickdist1(i,1).*cos(brickangle1(i)); %getting x displacement for IK
    brickdist1y(i) = brickdist1(i,1).*sin(brickangle1(i)); %getting y displacement for Ik
    brickangle1(i)= atan2(testpts(i,2)-base1(1,2),testpts(i,1)-base1(1,1));%angle between arm one and points
end
base1brickorder = zeros(n,2);
base2brickorder = zeros(n,2);

closestobjfirst1 = min(brickdistfirst1(:,1));
closestobjfirst2 = min(brickdistfirst2(:,1));

for i = 1:n %makes closest brick from start that bricks value
     if closestobjfirst1 == brickdistfirst1(i,2)
         base1brickorder(i,1) = closestobjfirst1;
     end 
     if closestobjfirst2 == brickdistfirst2(i,2)
         base2brickorder(i,1) = closestobjfirst2;
     end
end
base1brickorder = sortrows(brickdist1,2);%ordered by brick number
base2brickorder = sortrows(brickdist2,2);
for i = 1:n %removes bricks from the lists that will be more efficient on the other (closest distance)
    if base1brickorder(i,1) == base2brickorder(i,1)
        if brickdistfirst1(i,1) < brickdistfirst2(i,1)
            base2brickorder(i,1) = 0;
        end 
        if brickdistfirst1(i,1) > brickdistfirst2(i,1)
            base1brickorder(i,1) = 0;
        end 
    end
end
base1brickorder = sortrows(base1brickorder,1);%sequence of next closest brick
base2brickorder = sortrows(base2brickorder,1);
%outputs are base1brickorder or base2brickorder. If the value next to the
%block is zero, the other arm grabbed it, if the value is non zero, the
%first column is the distance and the second column is the block number

%% display section
%use base1brickorder and base2brick order to display a sequential set of figures to highlight which 
%brick it has picked at which step (highlight arm1 pick red and arm2 pick blue)