%author: Steven Craig
%
%inputs: BotNum (scalar value), BotPos(x1,y2,x2,y2,...), radius(scalar
%value)
%outptut: row 1 = x values. row2 = y values that make upper curve
%row 3 = y values that make lower curve
%row 4 = second robot x values, etc....
function ZoneCoordinates = BotZoneCreation(BotNum,BotPos,radius)
ZoneCoordinates = zeros(3*BotNum,(2*radius)+1);
for x = 1:BotNum
    xarea = BotPos((2*x)-1)-radius:BotPos((2*x)-1)+radius;
    TopHalf = BotPos(2*x) + sqrt(radius^2 - (xarea - BotPos((2*x)-1)).^2);
    BottomHalf = BotPos(2*x) - sqrt(radius^2 - (xarea - BotPos((2*x)-1)).^2);
    ZoneCoordinates((3*x)-2,:) = xarea;
    ZoneCoordinates((3*x)-1,:) = TopHalf;
    ZoneCoordinates((3*x),:) = BottomHalf;
    plot(xarea,TopHalf)
    plot(xarea,BottomHalf)
end
    