import numpy as np
import matplotlib.pyplot as plt
import math as m
import matplotlib.pyplot as plt

# checks for any Nan in the line
def isNan(delta_x, delta_y, yaw, x, y):
  if(np.isnan(float(delta_x)) or np.isnan(float(delta_y)) or np.isnan(float(yaw)) or np.isnan(float(x)) or np.isnan(float(y))):
    return True
  return False

def parseVOestimates():
  vox = []
  voy = []
  vo_yaw = []
  with open("vo_pose_current.txt", "r") as file:
    line = file.readline()
    while(line):
      data = line.split(",")
      x, y, yaw = data
      vox.append(float(x))
      voy.append(float(y))
      vo_yaw.append(float(yaw))
      line = file.readline()

  return vox, voy, vo_yaw

# parses the input file
def parseFile():
  del_l = []
  gtx = []
  gty = []
  theta_vector = []
  # open the file
  with open("pose_data_current.txt","r") as file:
    line = file.readline()
    first_line = True
    R = None
    t = None
    H = None
    while(line):
      data = line.split(",")
      delta_x, delta_y, yaw, x, y = data
      if isNan(delta_x, delta_y, yaw, x, y):
        break;
      delta_x = float(delta_x)
      delta_y = float(delta_y)
      yaw = float(yaw)
      x = float(x)
      y = float(y)

      if first_line:
        first_line = False
        R = np.array([[np.cos(yaw), -np.sin(yaw)],
                      [np.sin(yaw), np.cos(yaw)]])
        R = R.T
        t = np.array([[x],[y]])
        t = -np.dot(R,t)

        H = np.array([[R[0,0], R[0,1], t[0]],
                      [R[1,0], R[1,1], t[1]],
                      [0     , 0     , 1  ]])
        x = 0
        y = 0
      else:
        state = np.array([[x],[y],[1]])
        state = np.dot(H, state)
        x = float(state[0])
        y = float(state[1])

      del_l.append(delta_x)
      del_l.append(delta_y)
      theta_vector.append(yaw)
      gtx.append(x)
      gty.append(y)

      # print("delta_x : {}, delta_y: {}, yaw : {}, x : {}, y : {}".format(delta_x, delta_y, yaw, x, y))
      line = file.readline()

  # convert to numpy array
  del_l = np.array(del_l)
  return del_l, theta_vector, gtx, gty

# returns A2 as described in the LAGO paper
def calcA2(numEdges):
  A2 = np.zeros((2 * numEdges,2 * numEdges))

  j = 0;
  for i in range(2*numEdges):
    A2[i][j] = 1;
    if(j + 2 < 2*numEdges):
      A2[i][j+2] = -1;
    j+=1;

  return A2

# returns R star as defined in the LAGO paper
def calcRstar(theta_vector):
  theta_size = len(theta_vector)
  mat_size = 2 * theta_size

  Rstar = np.eye(mat_size)

  # populate Rstar
  for i in range(theta_size):
    k = i + 1;
    Rstar[2*k - 2, 2*k - 2] = m.cos(theta_vector[i])
    Rstar[2*k - 2, 2*k - 1] = -m.sin(theta_vector[i])
    Rstar[2*k - 1, 2*k - 2] = m.sin(theta_vector[i])
    Rstar[2*k - 1, 2*k - 1] = m.cos(theta_vector[i])

  return Rstar

# returns P_del_l as defined in the LAGO paper
def calcPDelL(numEdges):
  P =  0.1 * np.eye(numEdges * 2)
  return P

# returns P_del_g as defined in the LAGO paper
def calcPDelG(numEdges, Rstar):
  P_del_l = calcPDelL(numEdges)
  P_del_g = np.dot(np.dot(Rstar, P_del_l), Rstar.T)
  return P_del_g

# returns del_g
def calcDelG(del_l, Rstar):
  del_g = np.dot(Rstar, del_l)
  return del_g

def computeOptimalPoseConfig(A2, P_del_g, del_g):
  print("P_del_g is: {}".format(P_del_g))
  A2Pinv = np.dot(A2, np.linalg.inv(P_del_g))
  invA2PinvA2T = np.linalg.inv(np.dot(A2Pinv, A2.T))
  A2PinvDelg = np.dot(A2Pinv, del_g)
  p_star = np.dot(invA2PinvA2T, A2PinvDelg)
  return p_star

def getCordinatesFrompstar(p_star):
  lago_x = []
  lago_y = []
  i = 0
  while(i < len(p_star) - 1):
    lago_x.append(p_star[i])
    lago_y.append(p_star[i+1])
    i+=2

  return lago_x, lago_y

def parseVirtualDel(vox, voy, vo_yaw):
  virtual_del_l = []
  virtual_theta_vector = []
  for i in range(0, len(vox)- 1):
    # forma a vector
    dvo = np.array([[vox[i + 1] - vox[i]],
                    [voy[i + 1] - voy[i]]])
    # form the rotation matrix
    Rot = np.array([[np.cos(vo_yaw[i]), -np.sin(vo_yaw[i])],
                    [np.sin(vo_yaw[i]), np.cos(vo_yaw[i])]])

    dvo_relative = np.dot(Rot.T, dvo)
    dx = float(dvo_relative[0])
    dy = float(dvo_relative[1])
    virtual_del_l.append(dx)
    virtual_del_l.append(dy)
    virtual_theta_vector.append(vo_yaw[i])

  virtual_del_l = np.array(virtual_del_l)
  return virtual_del_l, virtual_theta_vector

def transformVO(vox, voy, vo_yaw):
  first_line = True
  R = None
  t = None
  H = None
  vox_new = []
  voy_new = []
  vo_yaw_new = []
  ref_yaw = None

  for i in range(len(vox)):
    x = 0
    y = 0
    yaw = 0
    if first_line:
      first_line = False
      R = np.array([[np.cos(0), -np.sin(0)],
                    [np.sin(0), np.cos(0)]])
      R = R.T
      t = np.array([[vox[i]],[voy[i]]])
      t = -np.dot(R,t)

      H = np.array([[R[0,0], R[0,1], t[0]],
                    [R[1,0], R[1,1], t[1]],
                    [0     , 0     , 1  ]])
      x = 0
      y = 0
      ref_yaw = float(vo_yaw[i])
    else:
      state = np.array([[vox[i]],[voy[i]],[1]])
      state = np.dot(H, state)
      x = float(state[0])
      y = float(state[1])
      yaw = vo_yaw[i] - ref_yaw

    vox_new.append(x)
    voy_new.append(y)
    vo_yaw_new.append(yaw)

  return vox_new, voy_new, vo_yaw_new



def plot_traj(gtx, gty, lago_x, lago_y, vox, voy):
  fig, axs = plt.subplots(1)
  axs.set_aspect('equal')
  axs.plot(gtx,gty,'r-', label = "ground truth")
  axs.plot(lago_x, lago_y, 'b-', label = "lago estimates")
  axs.plot(vox, voy, 'm-', label = "vo estimates")
  plt.legend()
  plt.show()

if __name__ == "__main__":
  #extract data
  vox, voy, vo_yaw = parseVOestimates()
  vox, voy, vo_yaw = transformVO(vox, voy, vo_yaw)
  del_l, theta_vector, gtx, gty = parseFile()
  numEdges = del_l.shape[0] / 2

  # precompute values
  Rstar = calcRstar(theta_vector)
  A2 = calcA2(numEdges)
  P_del_l = calcPDelL(numEdges)
  P_del_g = calcPDelG(numEdges, Rstar)
  del_g = calcDelG(del_l, Rstar)

  # print debug messages
  print("del_l : {}".format(del_l))
  print("numEdges : {}".format(numEdges))
  print("Rstar : {}".format(Rstar))
  print("A2 : {}".format(A2))
  print("P_del_l : {}".format(P_del_l))
  print("P_del_g : {}".format(P_del_g))
  print("del_g : {}".format(del_g))

  # compute poses values
  p_star = computeOptimalPoseConfig(A2, P_del_g, del_g)
  lago_x, lago_y = getCordinatesFrompstar(p_star)
  print("p_star : {}".format(p_star))
  plot_traj(gtx, gty, lago_x, lago_y, vox, voy)

  """
  refining wont work

  # as we have already gotten raw lago estimates
  # lets generate virtual del_l
  virtual_del_l, virtual_theta_vector = parseVirtualDel(vox, voy, vo_yaw)
  virtual_numEdges = virtual_del_l.shape[0] / 2
  # precompute virtual values
  virtual_Rstar = calcRstar(virtual_theta_vector)
  virtual_A2 = calcA2(virtual_numEdges)
  virtual_P_del_l = calcPDelL(virtual_numEdges)
  virtual_P_del_g = calcPDelG(virtual_numEdges, virtual_Rstar)
  virtual_del_g = calcDelG(virtual_del_l, virtual_Rstar)

  # compute refined poses values
  virtual_p_star = computeOptimalPoseConfig(virtual_A2, virtual_P_del_g, virtual_del_g)
  refined_x, refined_y = getCordinatesFrompstar(virtual_p_star)
  """
