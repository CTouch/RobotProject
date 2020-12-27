import numpy as np

# input rot: rotation of each axis
# return Tn: 
def forward(rot):
    # all the DH param
    a = np.array([[0, 0.225, 0, 0, 0, 0]], dtype=np.float32).T
    d = np.array([[0.284, 0, 0, 0.2289, 0, 0.055]], dtype=np.float32).T
    alpha = np.array([[np.pi/2, 0, np.pi/2, np.pi/2, np.pi/2, 0]], dtype=np.float32).T
    theta = np.array([[0, np.pi/2, 0, np.pi, np.pi/2, 0]], dtype=np.float32).T + rot

    # initiate Tn
    Tn = np.identity(4)

    for i in range(0, 6):
        t = np.array([
            [np.cos(theta[i]), -np.cos(alpha[i])*np.sin(theta[i]), np.sin(alpha[i])*np.sin(theta[i]), a[i]*np.cos(theta[i])],
            [np.sin(theta[i]), np.cos(alpha[i])*np.cos(theta[i]), -np.sin(alpha[i])*np.cos(theta[i]), a[i]*np.sin(theta[i])],
            [0, np.sin(alpha[i]), np.cos(alpha[i]), d[i]],
            [0, 0, 0, 1]
        ], dtype=np.float32)
        Tn = np.dot(Tn, t)

    # eliminate the computation error
    Tn[abs(Tn) < 1e-4] = 0

    # to match the coordinate with gazebo
    Q = np.array([
        [1, 0, 0, 0],
        [0, 0, 1, 0],
        [0, -1, 0, 0],
        [0, 0, 0, 1]
    ])
    Tn_gazebo = np.dot(Tn, Q)

    # extract information from Tn
    dn = np.array([Tn[0:3, 3]]).T
    Rn = Tn[0:3, 0:3]
    Rn_gazebo = Tn_gazebo[0:3, 0:3]

    # calculate the RPY
    # Y = np.around(np.arctan2(Rn_gazebo[1,0], Rn_gazebo[0,0]), decimals=4)
    # P = np.around(np.arctan2(-Rn_gazebo[2,0], np.hypot(Rn_gazebo[2,0], Rn_gazebo[2,2])), decimals=4)
    # R = np.around(np.arctan2(Rn_gazebo[2,1], Rn_gazebo[2,2]), decimals=4)
    P = np.arctan2(-Rn_gazebo[2,0], np.hypot(Rn_gazebo[0,0], Rn_gazebo[1,0]))
    if abs(P) is not np.pi/2:
        Y = np.arctan2(Rn_gazebo[1,0]/np.cos(P), Rn_gazebo[0,0]/np.cos(P))
        R = np.arctan2(Rn_gazebo[2,1]/np.cos(P), Rn_gazebo[2,2]/np.cos(P))
    elif P == np.pi/2:
        Y = 0
        R = np.arctan2(Rn_gazebo[0,1], Rn_gazebo[1,1])
    elif P == -np.pi/2:
        Y = 0
        R = -np.arctan2(Rn_gazebo[0,1], Rn_gazebo[1,1])

    RPY = np.around([R, P, Y], decimals=4)

    return Tn, dn, Rn, RPY


def main():

    rot = np.array([[0.5*np.pi, 0*np.pi, 0*np.pi, 0*np.pi, 0.25*np.pi, 0*np.pi]], dtype=np.float32).T

    Tn, dn, Rn, RPY = forward(rot)



    print("rotation: ", np.around(rot.T, decimals=4), "\nTn: ", np.around(Tn, decimals=4), "\ndn: ", np.around(dn, decimals=4))

    print("RPY: ", RPY)

if __name__ == "__main__":
    main()


#######################
# Author: Yangshuo He #
# Date: 2020.10.8     #
# Func: Forward       #
# Ver: 1.0.0          #
#######################