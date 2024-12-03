import math
class DHMatrix6Dof:
    #DH parameters
    
    # a is distance between the origin of frame n and n-1 along Xn
    A = [0,50,0,0,0,0]
    
    # alpha is anangle from  Zn-1 to Zn along Xn
    Alphas = [math.radians(90),math.radians(0),math.radians(90),
              math.radians(90),math.radians(90),math.radians(0)]
    
    # d is the distance from Xn-1 to Xn along the Zn-1 direction.
    D = [50,0,0,50,0,0]
    
    # Theta is anangle from  Xn-1 to Xn along Zn-1
    Theta1 = math.radians(0)
    Theta2 = math.radians(0)
    Theta3 = math.radians(90)
    Theta4 = math.radians(-180)
    Theta5 = math.radians(90)
    Theta6 = math.radians(0)
    
    Thetas = [Theta1,Theta2,Theta3,Theta4,Theta5,Theta6]