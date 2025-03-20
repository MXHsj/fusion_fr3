import numpy as np
import panda_py
from panda_py import controllers
import panda_py.constants
from spatialmath import SO3

import rospy
from std_msgs.msg import Float64MultiArray

import logging
# logging.basicConfig(level=logging.INFO)

CAM_T_PROBE = np.array([[-0.9997596514674911, 0.01895868516356237, -0.011009430251793509, 0.003494285383665675],
                        [-0.019020413904005103, -0.9998038049115955, 0.005529515278539474, 0.04646526225586623],
                        [-0.010902437916377784, 0.005737590187891416, 0.9999241055731759, 0.17898964143239388],
                        [0, 0, 0, 1]])

n_vec_calib_esti = np.array([0.34137974, 0.01360224, 0.93982703])
n_vec_calib_true = np.array([0.0, 0.0, 1.0])
calib_rot_axis = np.cross(n_vec_calib_esti, n_vec_calib_true)
calib_rot_ang = np.arccos(np.clip(np.dot(n_vec_calib_esti, n_vec_calib_true), -1.0, 1.0))

calib_rot_R = SO3.AngleAxis(calib_rot_ang, calib_rot_axis)

joint_pos_start = panda_py.constants.JOINT_POSITION_START
joint_pos_start[-1] = np.pi/2

class TrackVectorRT():
  
    def __init__(self):
    
        self.robot_ip = '172.16.0.2'

        self.a_des_base = np.array([0.0, 0.0, 1.0])    # desired ee approach vector (z-axis) w.r.t base
        self.a_des_base_last = np.array([0.0, 0.0, 1.0])
        self.a_des_base_hat = np.array([0.0, 0.0, 0.0])

        self.Kp1 = 1.25
        self.Kp2 = 0.5          # 0.50
        self.Kp_manip = 0.05     # 0.20

        self.epsilon = 1e-6

        self.panda = panda_py.Panda('172.16.0.2')
        self.panda.move_to_joint_position(joint_pos_start)
        
        self.ctrl = controllers.IntegratedVelocity()

        self.n_des_base = self.panda.get_pose()[:3, 0]    # desired ee normal vector (x-axis) w.r.t base

        self.panda.teaching_mode(True)
        input('press enter to start control')
        self.panda.teaching_mode(False)
        self.panda.start_controller(self.ctrl)

        rospy.init_node('track_vector_rt', anonymous=True)
        rospy.Subscriber('/asee2/normal_vector', Float64MultiArray, self.normal_vector_cb)
        self.hz = 100
        self.rate = rospy.Rate(self.hz)
        self.last_msg_time = rospy.get_time()

    def normal_vector_cb(self, msg:Float64MultiArray):
        self.a_des_base = np.array(msg.data)
        self.transform_normal_vector_to_base()
        self.compensate_norm_error()
        print(f'normal vector w.r.t base: {self.a_des_base}')

        self.a_des_base_hat = (self.a_des_base-self.a_des_base_last) / (rospy.get_time()-self.last_msg_time)

        self.last_msg_time = rospy.get_time()
        self.a_des_base_last = self.a_des_base.copy()

    def transform_normal_vector_to_base(self):
        O_T_PROBE = self.panda.get_pose()
        O_T_CAM = O_T_PROBE @ np.linalg.pinv(CAM_T_PROBE)
        norm_base = O_T_CAM @ np.concatenate((self.a_des_base, np.array([1.0]))).T

        self.a_des_base = norm_base[:3]
        self.a_des_base = self.a_des_base / np.linalg.norm(self.a_des_base)

    def compensate_norm_error(self):
        self.a_des_base = calib_rot_R * self.a_des_base
        self.a_des_base = self.a_des_base[:,0]
        # print(self.a_des_base)
        self.a_des_base = self.a_des_base / np.linalg.norm(self.a_des_base)

    def onUpdate(self):

        while not rospy.is_shutdown():

            T_0 = self.panda.get_pose()

            # ========== compute twist to align z-axis ==========
            a_cur_base = T_0[:3, 2]
            a_cur_base = a_cur_base / np.linalg.norm(a_cur_base)
            e_a = np.cross(a_cur_base, self.a_des_base)

            # print(f'approach vector error: {e_a}')

            # TODO: add feedforward term: a_cur x a^hat_des
            # omega_a = -self.Kp1 * e_a
            omega_a = -self.Kp1 * e_a + 0.05*np.cross(a_cur_base, self.a_des_base_hat)

            # ========== compute twist to align x-axis ==========
            n_cur_base = T_0[:3, 0]
            n_cur_base = n_cur_base / np.linalg.norm(n_cur_base)
            e_n = np.cross(n_cur_base, self.n_des_base)

            # print(f'normal vector error: {e_a}')

            omega_n = -self.Kp2 * np.dot(e_n, a_cur_base)

            # ========== control joint velocity ==========
            omega_total = omega_a + omega_n

            v = np.array([0.0, 0.0, 0.0])
            twist = np.concatenate((v, omega_total))

            model = self.panda.get_model()
            J = model.zero_jacobian(frame=panda_py.libfranka.Frame.kEndEffector, 
                                    q=self.panda.q, 
                                    EE_T_K=self.panda.get_state().EE_T_K, 
                                    F_T_EE=self.panda.get_state().F_T_EE)
            J = np.array(J).reshape(7, 6).T
            J_pinv = np.linalg.pinv(J)

            # null-space projector
            N = np.eye(7) - J_pinv @ J

            # singularity avoidance
            manip = np.sqrt(np.linalg.det(J @ J.T))
            grad_manip = np.zeros(7)
            for i in range(7):
                q_perturb = self.panda.q.copy()
                q_perturb[i] += self.epsilon
                J_perturb = model.zero_jacobian(frame=panda_py.libfranka.Frame.kEndEffector, 
                                                q=q_perturb, 
                                                EE_T_K=self.panda.get_state().EE_T_K, 
                                                F_T_EE=self.panda.get_state().F_T_EE)
                J_perturb = np.array(J_perturb).reshape(7, 6).T
                manip_perturb = np.sqrt(np.linalg.det(J_perturb @ J_perturb.T))
                grad_manip[i] = (manip_perturb - manip) / self.epsilon
                
            dq_manip = self.Kp_manip * grad_manip

            dq = J_pinv @ twist

            # print(f'dq_manip: \n{dq_manip}')
            dq_total = dq + N @ dq_manip

            self.ctrl.set_control(dq_total)

            self.rate.sleep()

if __name__ == '__main__':
    control = TrackVectorRT()
    control.onUpdate()