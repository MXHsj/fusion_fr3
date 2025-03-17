import numpy as np
import panda_py
from panda_py import controllers
from spatialmath import SE3, SO3

import logging
# logging.basicConfig(level=logging.INFO)

robot_ip = '172.16.0.2'

a_des_base = np.array([0, 0, 1])    # desired ee approach vector (z-axis) w.r.t base
a_des_base = a_des_base / np.linalg.norm(a_des_base)

n_des_base = np.array([1, 0, 0])    # desired ee normal vector (x-axis) w.r.t base
n_des_base = n_des_base / np.linalg.norm(n_des_base)

Kp1 = 1.15
Kp2 = 0.85
Kp_manip = 0.0    # 0.5

epsilon = 1e-6

panda = panda_py.Panda('172.16.0.2')
panda.move_to_start()
ctrl = controllers.IntegratedVelocity()

panda.teaching_mode(True)
input('press enter to start control')
panda.teaching_mode(False)
panda.start_controller(ctrl)

with panda.create_context(frequency=1000) as ctx:
  while ctx.ok():

    T_0 = panda.get_pose()

    # ========== compute twist to align z-axis ==========
    a_cur_base = T_0[:3, 2]
    a_cur_base = a_cur_base / np.linalg.norm(a_cur_base)
    e_a = np.cross(a_cur_base, a_des_base)

    # print(f'approach vector error: {e_a}')

    # TODO: add feedforward term: a_cur x a^hat_des
    omega_a = -Kp1 * e_a

    # ========== compute twist to align x-axis ==========
    n_cur_base = T_0[:3, 0]
    n_cur_base = n_cur_base / np.linalg.norm(n_cur_base)
    e_n = np.dot(n_cur_base, n_des_base)

    # print(f'normal vector error: {e_a}')

    omega_n = -Kp2 * e_n * a_cur_base

    # ========== control joint velocity ==========
    omega_total = omega_a + omega_n

    v = np.array([0.0, 0.0, 0.0])
    twist = np.concatenate((v, omega_total))

    model = panda.get_model()
    J = model.zero_jacobian(frame=panda_py.libfranka.Frame.kEndEffector, 
                            q=panda.q, 
                            EE_T_K=panda.get_state().EE_T_K, 
                            F_T_EE=panda.get_state().F_T_EE)
    J = np.array(J).reshape(7, 6).T
    J_pinv = np.linalg.pinv(J)

    # null-space projector
    N = np.eye(7) - J_pinv @ J

    # singularity avoidance
    manip = np.sqrt(np.linalg.det(J @ J.T))
    grad_manip = np.zeros(7)
    for i in range(7):
      q_perturb = panda.q.copy()
      q_perturb[i] += epsilon
      J_perturb = model.zero_jacobian(frame=panda_py.libfranka.Frame.kEndEffector, 
                                      q=q_perturb, 
                                      EE_T_K=panda.get_state().EE_T_K, 
                                      F_T_EE=panda.get_state().F_T_EE)
      J_perturb = np.array(J_perturb).reshape(7, 6).T
      manip_perturb = np.sqrt(np.linalg.det(J_perturb @ J_perturb.T))
      grad_manip[i] = (manip_perturb - manip) / epsilon
    dq_manip = Kp_manip * grad_manip

    dq = J_pinv @ twist

    print(f'dq_manip: \n{dq_manip}')
    dq_total = dq + N @ dq_manip

    ctrl.set_control(dq_total)
