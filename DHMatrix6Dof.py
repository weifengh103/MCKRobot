import numpy as np
class DHMatrix6Dof:
    #DH parameters
    
    # a is distance between the origin of frame n and n-1 along Xn
    A = [0,50,0,0,0,0]
    
    # alpha is anangle from  Zn-1 to Zn along Xn
    Alphas = [np.radians(90),np.radians(0),np.radians(90),
              np.radians(90),np.radians(90),np.radians(0)]
    
    # d is the distance from Xn-1 to Xn along the Zn-1 direction.
    D = [50,0,0,50,0,0]
    
    # Theta is anangle from  Xn-1 to Xn along Zn-1
    Theta1 = np.radians(0)
    Theta2 = np.radians(0)
    Theta3 = np.radians(90)
    Theta4 = np.radians(-180)
    Theta5 = np.radians(90)
    Theta6 = np.radians(0)
    
    Thetas = [Theta1,Theta2,Theta3,Theta4,Theta5,Theta6]