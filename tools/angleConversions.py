import numpy as np

def Quaternion2Euler(q):
    # phi = np.arctan2(2*(q[0]*q[1] + q[2]*q[3]),(1 - 2*(q[1]**2 + q[2]**2)))
    # theta = np.arcsin(2*(q[0]*q[2] - q[3]*q[1]))
    # psi = np.arctan2(2*(q[0]*q[3] + q[1]*q[2]),(1 - 2*(q[2]**2 + q[3]**2)))
    phi = np.arctan2(2*(q[0]*q[1] + q[2]*q[3]),(q[0]**2 + q[3]**2 - q[1]**2 - q[2]**2))
    theta = np.arcsin(2*(q[0]*q[2] - q[3]*q[1]))
    psi = np.arctan2(2*(q[0]*q[3] + q[1]*q[2]),(q[0]**2 +q[1]**2 - q[2]**2 - q[3]**2))
    return float(phi),float(theta),float(psi)

def Euler2Quaternion(phi, theta, psi):
    cy = np.cos(psi * 0.5)
    sy = np.sin(psi * 0.5)
    cp = np.cos(theta * 0.5)
    sp = np.sin(theta * 0.5)
    cr = np.cos(phi * 0.5)
    sr = np.sin(phi * 0.5)

    e0 = cy * cp * cr + sy * sp * sr
    e1 = cy * cp * sr - sy * sp * cr
    e2 = cy * sp * cr + sy * cp * sr
    e3 = sy * cp * cr - cy * sp * sr
    return e0,e1,e2,e3

def Quaternion2Rotation(e): #body to inertial
    R = np.array([[(e[0]**2 + e[1]**2 - e[2]**2 - e[3]**2),(2*(e[1]*e[2] - e[0]*e[3])), \
                    (2*(e[1]*e[3] + e[0]*e[2]))], \
                    [(2*(e[1]*e[2] + e[0]*e[3])), (e[0]**2 - e[1]**2 + e[2]**2 - e[3]**2), \
                    (2*(e[2]*e[3] - e[0]*e[1]))], \
                    [(2*(e[1]*e[3] - e[0]*e[2])), (2*(e[2]*e[3] + e[0]*e[1])), \
                    (e[0]**2 - e[1]**2 - e[2]**2 + e[3]**2)]])
    return R.squeeze()

def Euler2Rotation(phi, theta, psi):
    # cy = np.cos(psi * 0.5)
    # sy = np.sin(psi * 0.5)
    # ct = np.cos(theta * 0.5)
    # st = np.sin(theta * 0.5)
    # cp = np.cos(phi * 0.5)
    # sp = np.sin(phi * 0.5)
    cy = np.cos(psi)
    sy = np.sin(psi)
    ct = np.cos(theta)
    st = np.sin(theta)
    cp = np.cos(phi)
    sp = np.sin(phi)
    R = np.array([[(ct*cy),(ct*sy),(-st)],
                  [(sp*st*cy - cp*sy),(sp*st*sy + cp*cy),(sp*ct)],
                  [(cp*st*cy + sp*sy),(cp*st*sy - sp*cy),(cp*ct)]])
    return R
