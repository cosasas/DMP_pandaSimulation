import numpy as np

threshold = 0.08   # constant, threshold for link function// ToDo den uparxei sto kwdika 
global u, u2
u2 = 0
u = np.zeros((9,))
thresh = False


def check(rho,u_psp):
    if rho < .01:
        return u_psp
    else:
        return u + u_psp


def getJointStates(robot, bullet_client, j):
	joint_states = bullet_client.getJointStates(robot,list(range(0,j))) #range(bullet_client.getNumJoints(robot)))
	joint_positions = [state[0] for state in joint_states]
	joint_velocities = [state[1] for state in joint_states]
	joint_torques = [state[3] for state in joint_states]
	return joint_positions, joint_velocities, joint_torques

def getMotorJointStates(robot, bullet_client, j):
    joint_states = bullet_client.getJointStates(robot, list(range(0,j)))  #range(bullet_client.getNumJoints(robot)))
    joint_infos = [bullet_client.getJointInfo(robot, i) for i in  list(range(0,j))]       #range(bullet_client.getNumJoints(robot))]
    joint_states = [j for j, i in zip(joint_states, joint_infos) if i[3] > -1]
    joint_positions = [state[0] for state in joint_states]
    joint_velocities = [state[1] for state in joint_states]
    joint_torques = [state[3] for state in joint_states]
    return joint_positions, joint_velocities, joint_torques

def setJointPosition(robot, position, bullet_client, kp=1.0, kv=0.3):
	num_joints = bullet_client.getNumJoints(robot)
	zero_vec = [0.0] * num_joints
	if len(position) == num_joints:
		bullet_client.setJointMotorControlArray(robot, range(num_joints), bullet_client.POSITION_CONTROL,
			targetPositions=position, targetVelocities=zero_vec,
			positionGains=[kp] * num_joints, velocityGains=[kv] * num_joints)
	else:
		print("Not setting torque. "
			  "Expected torque vector of "
			  "length {}, got {}".format(num_joints, len(torque)))

def multiplyJacobian(jacobian, vector):
	result = [0.0, 0.0, 0.0]
	for c in range(len(vector)):
		for r in range(3):
			result[r] += jacobian[r][c] * vector[c]
	return result




def link(robot, bullet_client, v, obstacle_radius):
    
    for j in range(bullet_client.getNumJoints(robot)):
        bullet_client.changeDynamics(robot, j, linearDamping=0, angularDamping=0)
        info = bullet_client.getLinkState(robot, j)
        info2 = bullet_client.getLinkState(robot, j+1)
        if j < 7:       # den 8elw ton gripper gia arxi mporw na paikse me auto argotera
            p1 = info[0]            #begin of the arm
            p2 = info2[0]           #end of the arm

            vec_line = np.subtract(p2, p1)
        # calculate minimum distance from arm segment to obstacle
        # the vector of our line
        # the vector from the obstacle to the first line point
            vec_ob_line = np.subtract(v, p1)
        # calculate the projection normalized by length of arm segment

            projection = (np.dot(vec_ob_line, vec_line) / np.sum((vec_line)**2))
            if projection < 0:         
        # then closest point is the start of the segment
                closest = p1
            elif projection > 1:
        # then closest point is the end of the segment
                closest = p2
            else:
                closest = p1 + projection * vec_line
        # calculate distance from obstacle vertex to the closest point
            dist = np.sqrt(np.sum((np.subtract(v, closest))**2))
        # account for size of obstacle
            rho = dist - obstacle_radius # monodiastatos ari8mos
            thresh = False
            if rho < threshold : #threshold:0.12
                eta = .02
                thresh = True
                drhodx = np.subtract(v, closest) / rho  # partial derivate of rho
                Fpsp = (eta * (1.0/rho - 1.0/threshold) *1.0/rho**2 * drhodx)

                #numJoints = bullet_client.getNumJoints(robot)
                zero_vec = [0.0] * 9 #numJoints

                pos, vel, torq = getJointStates(robot, bullet_client, 9)
                #mpos, mvel, mtorq = getMotorJointStates(robot, bullet_client, 9)

                #result = bullet_client.getLinkState(robot, 11, computeLinkVelocity=1, computeForwardKinematics=1) #kukaEndEffectorIndex, computeLinkVelocity=1, computeForwardKinematics=1)
                #link_trn, link_rot, com_trn, com_rot, frame_pos, frame_rot, link_vt, link_vr = result

                # calculate the jacobian for this point (closest)
                jac_t, jac_r = bullet_client.calculateJacobian(robot, j, closest, pos, zero_vec, zero_vec)#com_trn, pos, zero_vec, zero_vec)

                # calculate the inertia matrix for the
                # point subjected to the potential space
                Mxpsp = bullet_client.calculateMassMatrix(robot,pos)
                y=np.array([np.array(xi) for xi in Mxpsp])

                kl = np.transpose(y[0])

                #Mxpsp_inv = np.dot(Jpsp,np.dot(np.linalg.pinv(Mq), Jpsp.T))
                Jpsp = np.linalg.pinv(jac_t)
                second_part = np.dot(kl[0], Fpsp)

                u_psp = np.dot((Jpsp), second_part)

                u2 = check(rho, u_psp)
                thresh = 1 

                return u2, thresh

    return info[0] , thresh 
