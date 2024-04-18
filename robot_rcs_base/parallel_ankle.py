import numpy as np
from math import asin, sqrt


#################### working range ####################
# motor: -60~30 deg # -1.0472~0.5236 rad # forwad input
# ankle(roll): -25~25 deg, -0.43663~0.4363 rad
# ankle(pitch): -60~30 deg, -1.0472~0.5236 rad
#######################################################


def single_radian_rotation_matrix(angle_radian):
    return np.array([[np.cos(angle_radian), 0, np.sin(angle_radian)],
                     [0, 1, 0],
                     [-np.sin(angle_radian), 0, np.cos(angle_radian)]])


def radian_to_rotation_matrix(roll, pitch):
    roll_matrix = np.array([[1, 0, 0],
                            [0, np.cos(roll), -np.sin(roll)],
                            [0, np.sin(roll), np.cos(roll)]])
    pitch_matrix = np.array([[np.cos(pitch), 0, np.sin(pitch)],
                             [0, 1, 0],
                             [-np.sin(pitch), 0, np.cos(pitch)]])

    return np.dot(pitch_matrix, roll_matrix)


class ParallelAnkle:
    def __init__(self, type):
        # params and configuration
        if type == 'left':
            self.type = 1  # 1 stands for left lateral
            self.initial_ra1 = np.array([0, 25, 260])  # 初始化位置参数、连杆长度、踝关节旋转角度和力矩、旋量
            self.initial_ra2 = np.array([0, -25, 205])
            self.initial_rb1 = np.array([-53.95, 25, 270.68])
            self.initial_rb2 = np.array([-53.95, -25, 215.68])
            self.initial_rc1 = np.array([-53.95, 25, 10.68])
            self.initial_rc2 = np.array([-53.95, -25, 10.68])
            self.length_bar1 = 55
            self.length_bar2 = 55
            self.length_rod1 = 260
            self.length_rod2 = 205
            self.initial_offset = 11.2
        elif type == 'right':
            self.type = 2  # 2 stands for right lateral
            self.initial_ra1 = np.array([0, 25, 205])
            self.initial_ra2 = np.array([0, -25, 260])
            self.initial_rb1 = np.array([-53.95, 25, 215.68])
            self.initial_rb2 = np.array([-53.95, -25, 270.68])
            self.initial_rc1 = np.array([-53.95, 25, 10.68])
            self.initial_rc2 = np.array([-53.95, -25, 10.68])
            self.length_bar1 = 55
            self.length_bar2 = 55
            self.length_rod1 = 205
            self.length_rod2 = 260
            self.initial_offset = 11.2

    def left_fk_nnfit(self, joint_position_l, joint_position_r):
        b_a = np.array([
            4.1611978856994787, 4.28741375744369, -3.2329482724835192,
            3.2295728584301391, 3.365863656368993, -3.4259786092261972,
            0.303635264829151, 1.0357905491413777, 1.5231961933374518,
            1.0498917678443165, -0.58362691644491116, 1.7484159415008411,
            2.1113609630711081, 1.0747125999655283, 3.5508508673796273,
            -5.6997289141674745, -0.36377691922178579, 3.8197442841339533,
            2.9072686042308167, 4.4845259887083584, 3.0252680728927888,
            -4.4380907596964221, 3.4572240063471118, 2.2951870935425682,
            2.1190189047177417, 1.9493717780635309, 3.61377945932344,
            -1.9231356367313859, 1.1284908864145931, -1.4455242366115082,
            0.2762312104770232, -2.2592374860789031, -0.9919894744965122,
            -2.1345868264164469, -4.1318070917760625, 1.3700001439411873,
            1.9476927947853835, -0.47108463478070545, 5.9982905586314432,
            -4.4738324363998334
        ])

        c_a = np.array([
            -0.029794316585132038, 0.01074737121138849, -0.058219617245258738,
            1.320440270982024, -0.812445434526058, 0.85385039357651027,
            -0.019969054245140574, 0.0079475300319278418, -0.0056757536194365658,
            0.0021853594840013433, -0.022823271077105488, -0.011099589162747449,
            -0.14441661402650102, 0.0055976007721327955, 1.9685634137657466,
            -0.4000332665044134, -0.02085288266606752, 0.00960177457386128,
            0.63899221309889553, -0.67205744539590417, -1.8269641153308076,
            -2.6893496524019556, 0.36562432032970271, -0.24121790972856549,
            0.261320393324544, 0.064926999361325163, -1.0585982747652551,
            0.16535903667133425, 0.21883920552105279, -0.10727643303014929,
            0.29247156508798017, 0.012678685702453714, 0.4383921027264408,
            -0.069479006307976557, 1.2393962580759492, 0.040974566668789285,
            0.00022042510584849409, -0.00010354397897001635, 0.47070224310908043,
            0.098821292850216055
        ])

        a = np.array([
            -5.9760158537189847, -4.1249691705803508, 3.2066607341023725,
            -3.5720327546704826, -2.4546116983536455, 1.8517622251157368,
            4.404202269458251, -0.87384290494696482, -0.85240856674642762,
            0.065454587653003743, 0.058225544026714843, 1.1254504621582928,
            1.3112153092199585, -0.86300243857635028, 2.7755539240786171,
            -5.4439882066443595, -2.500743964487075, 4.6395292436252324,
            3.3321784177238882, 6.1120838188927777
        ])

        b = np.zeros(20)
        xp1 = np.array([-0.1639267685074135, 0.61135792158094282])
        ankle_position = np.zeros(2)

        d = (joint_position_l - -1.2256) * 1.0158007811508 + -1.0
        d1 = (joint_position_r - -1.2261) * 1.01606397138764 + -1.0

        for k in range(20):
            b[k] = 2.0 / (np.exp(-2.0 * (a[k] + (b_a[k] * d + b_a[k + 20] * d1))) + 1.0) - 1.0

        for k in range(2):
            d = 0.0
            for i in range(20):
                d += c_a[k + (i << 1)] * b[i]
            ankle_position[k] = ((xp1[k] + d) - -1.0) / (-1.01860676223578 * float(k) + 2.29184332958999) + (
                        -0.61086999999999991 * float(k) + -0.43633)

        # (ankle_position_roll, ankle_position_pitch)
        return ankle_position[0], ankle_position[1]

    def right_fk_nnfit(self, joint_position_l, joint_position_r):
        b_a = np.array([
            5.8015802628833963, 5.5325401941629586, -3.1505991231686941,
            -1.6915759867734201, -0.66507918822980661, -0.67238047065067741,
            -1.6663152753691004, -1.8304042900954436, -0.864526210917737,
            0.538361899407137, -0.77791921527080321, 1.2916881014140098,
            -3.3447817087599736, 1.7969729102051304, -3.4977469290567171,
            -3.613223353210175, 3.5411609071599148, -2.7773046317734376,
            3.8705604825857307, -2.0056279141110647, 1.2754353671617,
            1.140460934393515, 0.13051461255442884, -1.4773611913026237,
            -2.0003509287100094, -1.9455181279387057, 3.6493474874193628,
            0.66617788014938006, 0.48943252027258788, 0.46615879246825032,
            1.5161007739518868, -0.54082471251291553, -4.000751452938653,
            0.45733541304994479, -1.5828175080949114, -1.6496482114185858,
            2.04459900329226, -3.8725970521498554, -3.2058205465663394,
            -3.6934113164268672
        ])

        c_a = np.array([
            -0.52055268433455837, -0.013064426993101631, 0.66391725663750567,
            0.019889562027824745, 0.8069642301199792, 0.13739004104960595,
            -0.025791917069473709, -0.18778517593056288, -0.41929815057738729,
            1.0149687569993062, 0.46353307812404443, -1.1364794900870865,
            0.015683098719050922, 0.0077122779793951284, 0.73268754746115372,
            0.13786361779282369, -7.2012919550334438, -0.91596425278923554,
            -1.9496074237077998, 0.92922246644371409, 0.45437945232945731,
            0.13841680375736659, 3.1938362166761407, -0.30901667502786556,
            0.0016089429365942402, -0.00033021148932226209, -0.32109445292991867,
            0.1154325220587684, 1.0674143383026604, -0.37680019193500852,
            -0.89626271414466752, 0.31228596323969338, -0.15727994506526946,
            0.077378763634186221, 0.55143815685033315, -0.42124328495441182,
            1.0468702194073807, -1.1538345882850085, -0.548632191810631,
            0.54129620778662479
        ])

        a = np.array([
            -6.1083164296326329, -5.8301729310806731, 3.4229839770536832,
            3.0223018402152344, 1.4640724379919925, 1.4469175835095069,
            0.9314251848390066, 1.117096416081601, 0.92261500183104561,
            -0.037504437646810745, -0.14710242985866709, 0.97160335969268163,
            -1.8245064590468378, 0.88123673337471642, -2.9476764717238542,
            -3.0327050448250619, 4.5034889371670168, -6.4125243423026648,
            4.3845371782546385, -5.7719103289008293
        ])

        b = np.zeros(20)
        xp1 = np.array([1.06957653882809, 2.0739829450581366])
        ankle_position = np.zeros(2)

        d = (joint_position_l - -1.2258) * 1.01620852598953 + -1.0
        d1 = (joint_position_r - -1.2259) * 1.01565634251994 + -1.0

        for k in range(20):
            b[k] = 2.0 / (np.exp(-2.0 * (a[k] + (b_a[k] * d + b_a[k + 20] * d1))) + 1.0) - 1.0

        for k in range(2):
            d = 0.0
            for i in range(20):
                d += c_a[k + (i << 1)] * b[i]
            ankle_position[k] = ((xp1[k] + d) - -1.0) / (-1.01860676223578 * float(k) + 2.29184332958999) + (
                        -0.61086999999999991 * float(k) + -0.43633)

        # (ankle_position_roll, ankle_position_pitch)
        return ankle_position[0], ankle_position[1]

    def forward(self,
                joint_position_up,
                joint_position_down,
                joint_velocity_up=0,
                joint_velocity_down=0,
                joint_torque_up=0,
                joint_torque_down=0):
        # ankle position
        if self.type == 1:
            joint_position_l = joint_position_up
            joint_position_r = joint_position_down
            joint_velocity_l = joint_velocity_up
            joint_velocity_r = joint_velocity_down
            joint_torque_l = joint_torque_up
            joint_torque_r = joint_torque_down
            ankle_position_roll, ankle_position_pitch = self.left_fk_nnfit(joint_position_l, joint_position_r)
        elif self.type == 2:
            joint_position_r = joint_position_up
            joint_position_l = joint_position_down
            joint_velocity_r = joint_velocity_up
            joint_velocity_l = joint_velocity_down
            joint_torque_r = joint_torque_up
            joint_torque_l = joint_torque_down
            ankle_position_roll, ankle_position_pitch = self.right_fk_nnfit(joint_position_l, joint_position_r)

        rotation_matrix = radian_to_rotation_matrix(ankle_position_roll, ankle_position_pitch)
        target_rc1 = np.dot(rotation_matrix, self.initial_rc1)
        target_rc2 = np.dot(rotation_matrix, self.initial_rc2)
        target_ra1 = self.initial_ra1
        target_ra2 = self.initial_ra2

        # 根据角度得到旋转矩阵
        single_rotation_matrix_l = single_radian_rotation_matrix(joint_position_l)
        single_rotation_matrix_r = single_radian_rotation_matrix(joint_position_r)

        # 踝关节目标位置参数
        target_rb1 = target_ra1 + np.dot(single_rotation_matrix_l, (self.initial_rb1 - self.initial_ra1))
        target_rb2 = target_ra2 + np.dot(single_rotation_matrix_r, (self.initial_rb2 - self.initial_ra2))

        # bar和rod的向量表示
        r_bar1 = target_rb1 - target_ra1
        r_bar2 = target_rb2 - target_ra2
        r_rod1 = target_rc1 - target_rb1
        r_rod2 = target_rc2 - target_rb2

        # 旋量中的方向向量
        s11 = np.array([0, 1, 0])
        s21 = np.array([0, 1, 0])

        # Jx
        Jx = np.array([r_rod1.tolist() + (np.cross(target_rc1, r_rod1)).tolist(),
                       r_rod2.tolist() + (np.cross(target_rc2, r_rod2)).tolist()])

        # J_theta
        J_theta = np.array([[np.dot(s11, np.cross(r_bar1, r_rod1)), 0],
                            [0, np.dot(s21, np.cross(r_bar2, r_rod2))]])

        # G_matrix
        G_matrix = np.array([[0, 0, 0, np.cos(ankle_position_pitch), 0, -np.sin(ankle_position_pitch)],
                             [0, 0, 0, 0, 1, 0]]).T

        # ankle velocity
        joint_velocity = np.array([joint_velocity_l, joint_velocity_r])
        ankle_velocity_roll, ankle_velocity_pitch = np.linalg.inv(
            np.dot(np.linalg.inv(J_theta), np.dot(Jx, G_matrix))).dot(joint_velocity)

        # ankle torque
        joint_torque = np.array([joint_torque_l, joint_torque_r])
        ankle_torque_roll, ankle_torque_pitch = np.linalg.inv(
            np.linalg.inv(np.dot(np.linalg.inv(J_theta), np.dot(Jx, G_matrix))).T).dot(joint_torque)

        return ankle_position_pitch, \
            ankle_position_roll, \
            ankle_velocity_pitch, \
            ankle_velocity_roll, \
            ankle_torque_pitch, \
            ankle_torque_roll

    def inverse(self,
                ankle_position_pitch,
                ankle_position_roll,
                ankle_velocity_pitch=0,
                ankle_velocity_roll=0,
                ankle_torque_pitch=0,
                ankle_torque_roll=0):
        # limit pitch, roll angle
        if ankle_position_pitch < -1.0472:
            ankle_position_pitch = -1.0472
        elif ankle_position_pitch > 0.5236:
            ankle_position_pitch = 0.5236

        if ankle_position_roll < -0.4363:
            ankle_position_roll = -0.4363
        elif ankle_position_roll > 0.4363:
            ankle_position_roll = 0.4363

        rotation_matrix = radian_to_rotation_matrix(ankle_position_roll, ankle_position_pitch)

        # 旋转后的关节点
        target_rc1 = np.dot(rotation_matrix, self.initial_rc1)
        target_rc2 = np.dot(rotation_matrix, self.initial_rc2)
        target_ra1 = self.initial_ra1
        target_ra2 = self.initial_ra2

        # 得到计算公式的元素
        interm_a1 = target_rc1 - target_ra1
        a1 = interm_a1[0]
        interm_a2 = target_rc2 - target_ra2
        a2 = interm_a2[0]
        interm_b1 = target_ra1 - target_rc1
        b1 = interm_b1[2]
        interm_b2 = target_ra2 - target_rc2
        b2 = interm_b2[2]

        # 计算二阶范数
        norm_1 = np.linalg.norm(target_rc1 - target_ra1, ord=2)
        norm_2 = np.linalg.norm(target_rc2 - target_ra2, ord=2)

        c1 = (self.length_rod1 ** 2 - self.length_bar1 ** 2 - norm_1 ** 2) / (2 * self.length_bar1)
        c2 = (self.length_rod2 ** 2 - self.length_bar2 ** 2 - norm_2 ** 2) / (2 * self.length_bar2)

        # joint position
        joint_position_l = asin((b1 * c1 + sqrt(b1 ** 2 * c1 ** 2 - (a1 ** 2 + b1 ** 2) * (c1 ** 2 - a1 ** 2))) \
                                / (a1 ** 2 + b1 ** 2)) - np.deg2rad(self.initial_offset)
        joint_position_r = asin((b2 * c2 + sqrt(b2 ** 2 * c2 ** 2 - (a2 ** 2 + b2 ** 2) * (c2 ** 2 - a2 ** 2))) \
                                / (a2 ** 2 + b2 ** 2)) - np.deg2rad(self.initial_offset)

        single_rotation_matrix_l = single_radian_rotation_matrix(joint_position_l)
        single_rotation_matrix_r = single_radian_rotation_matrix(joint_position_r)

        # 踝关节目标位置参数
        target_rb1 = target_ra1 + np.dot(single_rotation_matrix_l, (self.initial_rb1 - self.initial_ra1))
        target_rb2 = target_ra2 + np.dot(single_rotation_matrix_r, (self.initial_rb2 - self.initial_ra2))

        # bar和rod的向量表示
        r_bar1 = target_rb1 - target_ra1
        r_bar2 = target_rb2 - target_ra2
        r_rod1 = target_rc1 - target_rb1
        r_rod2 = target_rc2 - target_rb2

        # 旋量中的方向向量
        s11 = np.array([0, 1, 0])
        s21 = np.array([0, 1, 0])

        # 雅可比矩阵的组成部分
        # Jx
        Jx = np.array([r_rod1.tolist() + (np.cross(target_rc1, r_rod1)).tolist(),
                       r_rod2.tolist() + (np.cross(target_rc2, r_rod2)).tolist()])

        # J_theta
        J_theta = np.array([[np.dot(s11, np.cross(r_bar1, r_rod1)), 0],
                            [0, np.dot(s21, np.cross(r_bar2, r_rod2))]])

        # G_matrix
        G_matrix = np.array([[0, 0, 0, np.cos(ankle_position_pitch), 0, -np.sin(ankle_position_pitch)],
                             [0, 0, 0, 0, 1, 0]]).T

        # joint velocity
        ankle_velocity = np.array([ankle_velocity_roll, ankle_velocity_pitch])
        joint_velocity_l, joint_velocity_r = np.dot(np.linalg.inv(J_theta), np.dot(Jx, G_matrix)).dot(ankle_velocity)

        # joint torque
        ankle_torque = np.array([ankle_torque_roll, ankle_torque_pitch])
        joint_torque_l, joint_torque_r = np.linalg.inv(np.dot(np.linalg.inv(J_theta), np.dot(Jx, G_matrix))).T.dot(
            ankle_torque)
        if self.type == 1:
            return joint_position_l, \
                joint_position_r, \
                joint_velocity_l, \
                joint_velocity_r, \
                joint_torque_l, \
                joint_torque_r
        else:
            return joint_position_r, \
                joint_position_l, \
                joint_velocity_r, \
                joint_velocity_l, \
                joint_torque_r, \
                joint_torque_l            

# testing demo
# left_ankle = ankle('left')
# ankle_position_roll, ankle_position_pitch, ankle_velocity_roll, ankle_velocity_pitch, ankle_torque_roll, ankle_torque_pitch = left_ankle.inverse(0.1, -0.4, 1.1, 0.3, 1.1, 0.3)

# right_ankle = ankle('right')
# joint_position_l, joint_position_r, joint_velocity_l, joint_velocity_r, joint_torque_l, joint_torque_r = right_ankle.inverse(0.1, -0.4, 1.1, 0.3, 1.1, 0.3)
