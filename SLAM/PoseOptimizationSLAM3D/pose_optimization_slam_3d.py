"""
3D (x, y, z, qw, qx, qy, qz) pose optimization SLAM
author: Ryohei Sasaki(@rsasaki0109)
Ref:
- [A Compact and Portable Implementation of Graph\-based SLAM](https://www.researchgate.net/publication/321287640_A_Compact_and_Portable_Implementation_of_Graph-based_SLAM)
- [GitHub \- furo\-org/p2o: Single header 2D/3D graph\-based SLAM library](https://github.com/furo-org/p2o)
- [GitHub \- AtsushiSakai/PythonRobotics
/SLAM/PoseOptimizationSLAM](https://github.com/AtsushiSakai/PythonRobotics/blob/master/SLAM/PoseOptimizationSLAM/pose_optimization_slam.py)
"""

import sys
import time
import numpy as np
from scipy import sparse
from scipy.sparse import linalg
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

est_traj_fig = plt.figure()
ax = est_traj_fig.add_subplot(111, projection='3d')

def skew_symmetric(v):

    return np.array(
        [[0, -v[2], v[1]],
         [v[2], 0, -v[0]],
         [-v[1], v[0], 0]]
    )

def robust_coeff(squared_error, delta):

    if (squared_error < 0): 
        return 0
    sqre = np.sqrt(squared_error)
    if (sqre < delta):
        return 1 # no effect
    return delta / sqre # linear


class Optimizer3D:

    def __init__(self):
        self.verbose = False
        self.animation = False
        self.p_lambda = 1e-7
        self.init_w = 1e10
        self.stop_thre = 1e-3
        self.robust_delta = 1
        self.dim = 6  # state dimension

    def optimize_path(self, nodes, consts, max_iter, min_iter):

        graph_nodes = nodes[:]
        prev_cost = sys.float_info.max

        est_traj_fig = plt.figure()
        ax = est_traj_fig.add_subplot(111, projection='3d')

        for i in range(max_iter):
            start = time.time()
            cost, graph_nodes = self.optimize_path_one_step(
                graph_nodes, consts)
            elapsed = time.time() - start
            if self.verbose:
                print("step ", i, " cost: ", cost, " time:", elapsed, "s")

            # check convergence
            if (i > min_iter) and (prev_cost - cost < self.stop_thre):
                if self.verbose:
                    print("converged:", prev_cost
                          - cost, " < ", self.stop_thre)
                    break
            prev_cost = cost

            if self.animation:
                plt.cla()
                plot_nodes(nodes, ax, color="-b")
                plot_nodes(graph_nodes, ax)
                plt.pause(1.0)

        return graph_nodes

    def optimize_path_one_step(self, graph_nodes, constraints):

        indlist = [i for i in range(self.dim)]
        numnodes = len(graph_nodes)
        bf = np.zeros(numnodes * self.dim)
        tripletList = TripletList()

        for con in constraints:
            ida = con.id1
            idb = con.id2
            assert 0 <= ida and ida < numnodes, "ida is invalid"
            assert 0 <= idb and idb < numnodes, "idb is invalid"
            pa = graph_nodes[ida]
            pb = graph_nodes[idb]
            r, Ja, Jb = self.calc_error(
                pa, pb, con.t)
 
            info_mat = con.info_mat * robust_coeff(r.reshape(self.dim,1).T @ con.info_mat @ r.reshape(self.dim,1), self.robust_delta)

            trJaInfo = Ja.transpose() @ info_mat
            trJaInfoJa = trJaInfo @ Ja
            trJbInfo = Jb.transpose() @ info_mat
            trJbInfoJb = trJbInfo @ Jb
            trJaInfoJb = trJaInfo @ Jb

            for k in indlist:
                for m in indlist:
                    tripletList.push_back(
                        ida * self.dim + k, ida * self.dim + m, trJaInfoJa[k, m])
                    tripletList.push_back(
                        idb * self.dim + k, idb * self.dim + m, trJbInfoJb[k, m])
                    tripletList.push_back(
                        ida * self.dim + k, idb * self.dim + m, trJaInfoJb[k, m])
                    tripletList.push_back(
                        idb * self.dim + k, ida * self.dim + m, trJaInfoJb[m, k])
                        
            bf[ida * self.dim: (ida + 1) * self.dim ] += trJaInfo @ r
            bf[idb * self.dim: (idb + 1) * self.dim ] += trJbInfo @ r
        
        for k in indlist:
            tripletList.push_back(k, k, self.init_w)

        for i in range(self.dim * numnodes):
            tripletList.push_back(i, i, self.p_lambda)

        mat = sparse.coo_matrix((tripletList.data, (tripletList.row, tripletList.col)),
                                shape=(numnodes * self.dim, numnodes * self.dim))

        x = linalg.spsolve(mat.tocsr(), -bf)

        out_nodes = []

        for i in range(len(graph_nodes)):

            u_i = i * self.dim
            
            q_before = Quaternion(graph_nodes[i].qw, graph_nodes[i].qx, graph_nodes[i].qy, graph_nodes[i].qz)
            rv_before = RotVec(quaternion = q_before)
            rv_after = RotVec(ax = rv_before.ax + x[u_i + 3], ay = rv_before.ay + x[u_i + 4], az = rv_before.az + x[u_i + 5])
            q_after = rv_after.toQuaternion()

            pos = Pose3D(
                graph_nodes[i].x  + x[u_i],
                graph_nodes[i].y  + x[u_i + 1],
                graph_nodes[i].z  + x[u_i + 2],
                q_after.qw,
                q_after.qx,
                q_after.qy,
                q_after.qz
            )
            out_nodes.append(pos)

        cost = self.calc_global_cost(out_nodes, constraints)

        return cost, out_nodes

    def calc_global_cost(self, nodes, constraints):

        cost = 0.0
        for c in constraints:
            diff = self.error_func(nodes[c.id1], nodes[c.id2], c.t)
            info_mat = c.info_mat * robust_coeff(diff.reshape(self.dim,1).T @ c.info_mat @ diff.reshape(self.dim,1), self.robust_delta)
            cost += diff.transpose() @ info_mat @ diff

        return cost

    def error_func(self, pa, pb, t):

        ba = pb.ominus(pa)
        q = t.rv().toQuaternion().conjugate().quat_mult(ba.rv().toQuaternion(), out = 'Quaternion')
        drv = RotVec(quaternion = q)
        error = np.array([ba.x - t.x,
                          ba.y - t.y,
                          ba.z - t.z,
                          drv.ax[0],
                          drv.ay[0],
                          drv.az[0]])
        return error

    def dQuat_dRV(self, rv):

        u1 = rv.ax; u2 = rv.ay; u3 = rv.az
        v = np.sqrt(u1**2 + u2**2 + u3**2)
        if v < 1e-6:
            dqu = 0.25 * np.array(
            [[ -u1, -u2, -u3],
             [ 2.0, 0.0, 0.0],
             [ 0.0, 2.0, 0.0],
             [ 0.0, 0.0, 2.0]]
            )
            return dqu

        vd = v*2
        v2 = v**2
        v3 = v**3

        S = np.sin(v/2.0); C = np.cos(v/2.0)
        dqu = np.array(
            [[ -u1 * S/vd                       , -u2*S/vd                     , -u3*S/vd],
             [ S/v + u1*u1*C/(2*v2) - u1*u1*S/v3, u1*u2*(C/(2*v2)-S/v3)        , u1*u3*(C/(2*v2)-S/v3)],
             [ u1*u2*(C/(2*v2)-S/v3)            , S/v+u2*u2*C/(2*v2)-u2*u2*S/v3, u2*u3*(C/(2*v2)-S/v3)],
             [ u1*u3*(C/(2*v2)-S/v3)            , u2*u3*(C/(2*v2)-S/v3)        , S/v+u3*u3*C/(2*v2)-u3*u3*S/v3]]
            )

        return dqu

    def dR_dRV(self, rv):

        q = rv.toQuaternion()
        qw = q.qw;qx = q.qx; qy = q.qy;qz = q.qz
        dRdqw = 2 * np.array(
            [[ qw, -qz,  qy],
             [ qz,  qw, -qx],
             [-qy,  qx,  qw]]
        )
        dRdqx = 2 * np.array(
            [[ qx,  qy,  qz],
             [ qy, -qx, -qw],
             [ qz,  qw, -qx]]
        )
        dRdqy = 2 * np.array(
            [[-qy,  qx,  qw],
             [ qx,  qy,  qz],
             [-qw,  qz, -qy]]
        )
        dRdqz = 2 * np.array(
            [[-qz, -qw,  qx],
             [ qw, -qz, -qy],
             [ qz,  qy,  qz]]
        )
        dqdu = self.dQuat_dRV(rv)
        dux = dRdqw * dqdu[0,0] + dRdqx * dqdu[1, 0] + dRdqy * dqdu[2, 0] + dRdqz * dqdu[3, 0]
        duy = dRdqw * dqdu[0,1] + dRdqx * dqdu[1, 1] + dRdqy * dqdu[2, 1] + dRdqz * dqdu[3, 1]
        duz = dRdqw * dqdu[0,2] + dRdqx * dqdu[1, 2] + dRdqy * dqdu[2, 2] + dRdqz * dqdu[3, 2]
        return dux, duy, duz
    
    def dRV_dQuat(self, q):
        
        qw = q.qw[0]
        qx = q.qx[0]
        qy = q.qy[0]
        qz = q.qz[0]

        if 1 - qw**2 < 1e-7:
            ret = np.array(
            [[ 0.0, 2.0, 0.0, 0.0],
             [ 0.0, 0.0, 2.0, 0.0],
             [ 0.0, 0.0, 0.0, 2.0]]
            )
            return ret
        
        c = 1/(1 - qw**2)
        d = np.arccos(qw)/(np.sqrt(1-qw**2))
        ret = 2.0 * np.array(
            [[ c*qx*(d*qw-1),   d, 0.0, 0.0],
             [ c*qy*(d*qw-1), 0.0,   d, 0.0],
             [ c*qz*(d*qw-1), 0.0, 0.0,   d]]
            )
        return ret

    def QMat(self, q):

        qw = q.qw; qx = q.qx; qy = q.qy; qz = q.qz
        Q = np.array(
            [[ qw, -qx, -qy, -qz],
             [ qx,  qw, -qz,  qy],
             [ qy,  qz,  qw, -qx],
             [ qz, -qy,  qx,  qw]]
            )
        return Q

    def QMatBar(self, q):

        qw = q.qw; qx = q.qx; qy = q.qy; qz = q.qz
        Q = np.array(
            [[ qw, -qx, -qy, -qz],
             [ qx,  qw,  qz, -qy],
             [ qy, -qz,  qw,  qx],
             [ qz,  qy, -qx,  qw]]
            )
        return Q

    def calc_error(self, pa, pb, t):

        e0 = self.error_func(pa, pb, t)
        Ja = np.identity(6); Jb = np.identity(6) 

        rva_inv = pa.rv().inverted()
        rotPaInv = rva_inv.toRotationMatrix()

        Ja[:3,:3] = -rotPaInv
        Jb[:3,:3] = rotPaInv

        dRux, dRuy, dRuz = self.dR_dRV(rva_inv)

        cvec = np.array([[pb.x - pa.x],[pb.y - pa.y],[pb.z - pa.z]])

        Ja[0:3,3:4] = -dRux @ cvec
        Ja[0:3,4:5] = -dRuy @ cvec
        Ja[0:3,5:6] = -dRuz @ cvec

        # rotation part: qdiff = qc-1 * qa-1 * qb
        qainv = rva_inv.toQuaternion()
        qcinv = t.rv().inverted().toQuaternion()
        qb = pb.rv().toQuaternion()
        qinvca = qcinv.quat_mult(qainv, out = 'Quaternion')
        qdiff  = qinvca.quat_mult(qb, out = 'Quaternion')

        Ja[3:6,3:6] = -self.dRV_dQuat(qdiff) @ self.QMat(qcinv) @ self.QMatBar(qb) @ self.dQuat_dRV(rva_inv)
        Jb[3:6,3:6] =  self.dRV_dQuat(qdiff) @ self.QMat(qcinv) @ self.QMat(qainv) @ self.dQuat_dRV(pb.rv())

        return e0, Ja, Jb

class Quaternion:

    def __init__(self, qw, qx, qy, qz):
        self.qw = qw; self.qx = qx; self.qy = qy; self.qz = qz  
    
    def conjugate(self):
        return Quaternion(self.qw, -self.qx, -self.qy, -self.qz)
    
    def to_numpy(self):
        return np.array([self.qw, self.qx, self.qy, self.qz]).reshape(4,1)

    def quat_mult(self, q, out='np'):
        v = np.array([self.qx, self.qy, self.qz]).reshape(3, 1)
        sum_term = np.zeros([4,4])
        sum_term[0,1:]   = -v[:,0]
        sum_term[1:, 0]  = v[:,0]
        sum_term[1:, 1:] = skew_symmetric(v)
        sigma = self.qw * np.eye(4) + sum_term
        q_new = sigma @ q.to_numpy()
        if out == 'np':
            return q_new
        elif out == 'Quaternion':
            q_obj = Quaternion(*q_new)
            return q_obj
    

class RotVec:

    def __init__(self, ax=0., ay=0., az=0., quaternion=None):
        if quaternion is None:
            self.ax = ax; self.ay = ay; self.az = az
        else:
            x = quaternion.qx; y = quaternion.qy; z = quaternion.qz
            norm_im = np.sqrt(x**2 + y**2 + z**2)
            if (norm_im < 1e-7):
                self.ax = 2*x; self.ay = 2*y; self.az = 2*z
            else:
                th = 2 * np.arctan2(norm_im, quaternion.qw)
                th = self.pi2pi(th)
                self.ax = x / norm_im * th
                self.ay = y / norm_im * th
                self.az = z / norm_im * th

    def inverted(self):

        return RotVec(-self.ax, -self.ay, -self.az)

    def toRotationMatrix(self):

        q = self.toQuaternion()
        q_vec =  np.array([q.qx, q.qy, q.qz]).reshape(3,1)
        qw = q.qw
        mat = (qw**2 - q_vec.T @ q_vec) * np.eye(3) + \
               2 * q_vec @ q_vec.T - 2 * qw * skew_symmetric(q_vec.reshape(-1,))
        return mat.T
    
    def toQuaternion(self):

        v = np.sqrt(self.ax**2 + self.ay**2 + self.az**2)
        if (v < 1e-6):
            return Quaternion(1, 0, 0, 0)
        else:
            return Quaternion(np.cos(v/2), np.sin(v/2)*self.ax/v,
                            np.sin(v/2)*self.ay/v, np.sin(v/2)*self.az/v)

    def pi2pi(self, rad):

        val = np.fmod(rad, 2.0 * np.pi)
        if val > np.pi:
            val -= 2.0 * np.pi
        elif val < -np.pi:
            val += 2.0 * np.pi

        return val
    

class TripletList:

    def __init__(self):
        self.row = []
        self.col = []
        self.data = []

    def push_back(self, irow, icol, idata):
        self.row.append(irow)
        self.col.append(icol)
        self.data.append(idata)

class Pose3D:

    def __init__(self, x, y, z, qw, qx, qy, qz):
        self.x = x; self.y = y; self.z = z
        self.qw = qw; self.qx = qx; self.qy = qy; self.qz = qz
    
    def pos(self):
        v = np.array([self.x, self.y, self.z]).reshape(3,1)
        return v
    
    def rv(self):
        q = Quaternion(self.qw, self.qx, self.qy, self.qz)
        return RotVec(quaternion = q)

    def ominus(self, base):
        t = base.rv().toRotationMatrix().T @ (self.pos() - base.pos())
        q = base.rv().toQuaternion().conjugate().quat_mult(self.rv().toQuaternion(), out = 'Quaternion')
        return Pose3D(t[0][0], t[1][0], t[2][0], q.qw, q.qx, q.qy, q.qz)


class Constrant3D:

    def __init__(self, id1, id2, t, info_mat):
        self.id1 = id1
        self.id2 = id2
        self.t = t
        self.info_mat = info_mat

def plot_nodes(nodes, ax, color ="-r", label = ""):

    x, y, z = [], [], []
    for n in nodes:
        x.append(n.x); y.append(n.y); z.append(n.z)
    x = np.array(x)
    y = np.array(y)
    z = np.array(z)
    max_range = np.array([x.max()-x.min(), y.max()-y.min(), z.max()-z.min()]).max() / 2.0
    mid_x = (x.max()+x.min()) * 0.5
    mid_y = (y.max()+y.min()) * 0.5
    mid_z = (z.max()+z.min()) * 0.5
    ax.set_xlim(mid_x - max_range, mid_x + max_range)
    ax.set_ylim(mid_y - max_range, mid_y + max_range)
    ax.set_zlim(mid_z - max_range, mid_z + max_range)    
    ax.plot(x, y, z, color, label=label)

def load_data(fname):

    nodes, consts = [], []

    for line in open(fname):
        sline = line.split()
        tag = sline[0]

        if tag == "VERTEX_SE3:QUAT":
            #data_id = int(sline[1]) # unused
            x = float(sline[2])
            y = float(sline[3])
            z = float(sline[4])
            qx = float(sline[5])
            qy = float(sline[6])
            qz = float(sline[7])
            qw = float(sline[8])
         
            nodes.append(Pose3D(x, y, z, qw, qx, qy, qz))
        elif tag == "EDGE_SE3:QUAT":
            id1 = int(sline[1])
            id2 = int(sline[2])
            x = float(sline[3])
            y = float(sline[4])
            z = float(sline[5])
            qx = float(sline[6])
            qy = float(sline[7])
            qz = float(sline[8])
            qw = float(sline[9])
            c1  = float(sline[10])
            c2  = float(sline[11])
            c3  = float(sline[12])
            c4  = float(sline[13])
            c5  = float(sline[14])
            c6  = float(sline[15])
            c7  = float(sline[16])
            c8  = float(sline[17])
            c9  = float(sline[18])
            c10 = float(sline[19])
            c11 = float(sline[20])
            c12 = float(sline[21])
            c13 = float(sline[22])
            c14 = float(sline[23])
            c15 = float(sline[24])
            c16 = float(sline[25])
            c17 = float(sline[26])
            c18 = float(sline[27])
            c19 = float(sline[28])
            c20 = float(sline[29])
            c21 = float(sline[30])
            t = Pose3D(x, y, z, qw, qx, qy, qz)
            info_mat = np.array([[c1,  c2,  c3,  c4,  c5,  c6],
                                 [c2,  c7,  c8,  c9, c10, c11],
                                 [c3,  c8, c12, c13, c14, c15],
                                 [c4,  c9, c13, c16, c17, c18],
                                 [c5, c10, c14, c17, c19, c20],
                                 [c6, c11, c15, c18, c20, c21]
                                 ])
            consts.append(Constrant3D(id1, id2, t, info_mat))

    print("n_nodes:", len(nodes))
    print("n_consts:", len(consts))

    return nodes, consts



def main():
    
    print("start!!")

    fnames = ["parking-garage.g2o"]

    max_iter = 20
    min_iter = 3

    # parameter setting
    optimizer = Optimizer3D()
    optimizer.p_lambda = 1e-6
    optimizer.verbose = True
    optimizer.animation = True
    
    for f in fnames:
        print(f)

        nodes, consts = load_data(f)

        start = time.time()
        final_nodes = optimizer.optimize_path(nodes, consts, max_iter, min_iter)
        print("elapsed_time", time.time() - start, "sec")

        # plot
        plt.cla()
        est_traj_fig = plt.figure()
        ax = est_traj_fig.add_subplot(111, projection='3d')
        plot_nodes(nodes, ax, color="-b", label="before")
        plot_nodes(final_nodes, ax, label="after")
        plt.show()

    print("done!!")

if __name__ == '__main__':
    main()
