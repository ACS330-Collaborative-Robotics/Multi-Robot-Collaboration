%This provides the elements needed to run SelectionProcessBarrier
clear
n=16; %number of bricks
brickNo = (1:n);
testpts(1:n,1:2) =10*rand(n,2); testpts(1:n,1)=testpts(1:n,1)+2;
%random points x 2-12 y 0-10
base1=[0,5]; %arm 1 base  
base2=[14,5];%arm 2 base 
basket=[7,5];%block plasing area
dists = SelectionProcessBarrier(testpts,[base1,base2],basket,brickNo,1,0);
% disp(testpts)
% disp([base1,base2])
% disp(basket)
