import math
import matplotlib.pyplot as plt
def NoiseAddition(PathPointsx,PathPointsy,xgoal,ygoal,xobj,yobj):
    ObjAngle = math.atan2(ygoal-PathPointsy,xobj-xgoal)
    ObjAngle = math.degrees(ObjAngle)
    fig = plt.figure()
    ax = plt.axes()
    ax.plot([xgoal,PathPointsx,xobj],[ygoal,PathPointsy,yobj])
    ax.set_xlabel(ObjAngle)
    plt.show()
    return ObjAngle

print(NoiseAddition(5,5,5,10,-10,10))