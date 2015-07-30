#!/usr/bin/env python

from threading import Thread
import rospy
import math
from phantomx_gazebo.phantomx import PhantomX
from geometry_msgs.msg import Twist


jleg = ['j_c1', 'j_thigh', 'j_tibia']
suffix = ['f', 'm', 'r']
sides = ['l', 'r']
joints = []
for j in jleg:
    for s in suffix:
        for side in sides:
            z = j + "_" + side + s
            joints.append(z)

phantomx_joints = joints


class WJFunc:
    """Walk Joint Function"""
    def __init__(self):
        self.offset = 0
        self.scale = 1
        self.in_offset = 0
        self.in_scale = 1

    def get(self, x):
        """x between 0 and 1"""
        f = math.sin(self.in_offset + self.in_scale * x)
        return self.offset + self.scale * f

    def clone(self):
        z = WJFunc()
        z.offset = self.offset
        z.scale = self.scale
        z.in_offset = self.in_offset
        z.in_scale = self.in_scale
        return z

    def mirror(self):
        z = self.clone()
        z.offset *= -1
        z.scale *= -1
        return z

    def __str__(self):
        return 'y = {} + {} * sin({} + {} * x)'.format(
            self.offset, self.scale, self.in_offset, self.in_scale)


class WFunc:
    """Walk Function"""
    def __init__(self, **kwargs):
        self.parameters = {}

        self.parameters['swing_scale'] = 1
        self.parameters['vx_scale'] = 0.5
        self.parameters['vy_scale'] = 0.5
        self.parameters['vt_scale'] = 0.4

        for k, v in kwargs.items():
            self.parameters[k] = v

        self.joints = phantomx_joints
        self.generate()

    def generate(self):
        # f1=THIGH1=ANKLE1=L=R in phase
        self.pfn = {}  # phase joint functions
        self.afn = {}  # anti phase joint functions

        f1 = WJFunc()
        f1.in_scale = math.pi
        f1.scale = -self.parameters['swing_scale']

        f2 = f1.clone()
        f2.scale = 0

        f3 = f1.clone()
        f3.scale *= -1

        f4 = f2.clone()
        f3.scale *= -1

        zero = WJFunc()
        zero.scale = 0

        self.set_func('j_thigh', f1, f2)
        self.set_func('j_tibia', f3, f4)
        self.set_func('j_c1', zero, zero)

        self.show()

    def set_func(self, joint, fp, fa):
        for leg in ['lf', 'rm', 'lr']:
            j = joint + '_' + leg
            self.pfn[j] = fp
            self.afn[j] = fa

        for leg in ['rf', 'lm', 'rr']:
            j = joint + '_' + leg
            self.pfn[j] = fa
            self.afn[j] = fp

    # def generate_right(self):
    #     # Mirror from left to right and antiphase right
    #     l=[ v[:-2] for v in self.pfn.keys()]
    #     for j in l:
    #         self.pfn[j+"_r"]=self.afn[j+"_l"].mirror()
    #         self.afn[j+"_r"]=self.pfn[j+"_l"].mirror()

    def get(self, phase, x, velocity):
        """x between 0 and 1"""
        angles = {}
        for j in self.pfn.keys():
            if phase:
                v = self.pfn[j].get(x)
                angles[j] = v
            else:
                angles[j] = self.afn[j].get(x)
        self.apply_velocity(angles, velocity, phase, x)
        return angles

    def show(self):
        for j in self.pfn.keys():
            print j, 'p', self.pfn[j], 'a', self.afn[j]

    def apply_velocity(self, angles, velocity, phase, x):
        pass

        # VX
        v = velocity[0] * self.parameters['vx_scale']
        d = (x * 2 - 1) * v
        if phase:
            angles['j_c1_lf'] -= d
            angles['j_c1_rm'] += d
            angles['j_c1_lr'] -= d
            angles['j_c1_rf'] -= d
            angles['j_c1_lm'] += d
            angles['j_c1_rr'] -= d
        else:
            angles['j_c1_lf'] += d
            angles['j_c1_rm'] -= d
            angles['j_c1_lr'] += d
            angles['j_c1_rf'] += d
            angles['j_c1_lm'] -= d
            angles['j_c1_rr'] += d

        # VY
        # v=velocity[1]*self.parameters["vy_scale"]
        # d=(x)*v
        # d2=(1-x)*v
        # if v>=0:
        #     if phase:
        #         angles["j_thigh1_l"]-=d
        #         angles["j_ankle2_l"]-=d
        #         angles["j_thigh1_r"]+=d
        #         angles["j_ankle2_r"]+=d
        #     else:
        #         angles["j_thigh1_l"]-=d2
        #         angles["j_ankle2_l"]-=d2
        #         angles["j_thigh1_r"]+=d2
        #         angles["j_ankle2_r"]+=d2
        # else:
        #     if phase:
        #         angles["j_thigh1_l"]+=d2
        #         angles["j_ankle2_l"]+=d2
        #         angles["j_thigh1_r"]-=d2
        #         angles["j_ankle2_r"]-=d2
        #     else:
        #         angles["j_thigh1_l"]+=d
        #         angles["j_ankle2_l"]+=d
        #         angles["j_thigh1_r"]-=d
        #         angles["j_ankle2_r"]-=d

        # VT
        v = velocity[2] * self.parameters['vt_scale']
        d = (x * 2 - 1) * v
        if phase:
            angles['j_c1_lf'] += d
            angles['j_c1_rm'] += d
            angles['j_c1_lr'] += d
            angles['j_c1_rf'] -= d
            angles['j_c1_lm'] -= d
            angles['j_c1_rr'] -= d
        else:
            angles['j_c1_lf'] -= d
            angles['j_c1_rm'] -= d
            angles['j_c1_lr'] -= d
            angles['j_c1_rf'] += d
            angles['j_c1_lm'] += d
            angles['j_c1_rr'] += d


class Walker:
    """Class for making PhantomX walk"""
    def __init__(self, darwin):
        self.darwin = darwin
        self.running = False

        self.velocity = [0, 0, 0]
        self.walking = False
        self.func = WFunc()

        # self.ready_pos=get_walk_angles(10)
        self.ready_pos = self.func.get(True, 0, [0, 0, 0])

        self._th_walk = None

        self._sub_cmd_vel = rospy.Subscriber(
            darwin.ns + "cmd_vel", Twist, self._cb_cmd_vel, queue_size=1)

    def _cb_cmd_vel(self, msg):
        """Catches cmd_vel and update walker speed"""
        print 'cmdvel', msg
        vx = msg.linear.x
        vy = msg.linear.y
        vt = msg.angular.z
        self.start()
        self.set_velocity(vx, vy, vt)

    def init_walk(self):
        """If not there yet, go to initial walk position"""
        rospy.loginfo('Going to walk position')
        if self.get_dist_to_ready() > 0.02:
            self.darwin.set_angles_slow(self.ready_pos)

    def start(self):
        if not self.running:
            self.running = True
            self.init_walk()
            self._th_walk = Thread(target=self._do_walk)
            self._th_walk.start()
            self.walking = True

    def stop(self):
        if self.running:
            self.walking = False
            rospy.loginfo('Waiting for stopped')
            while not rospy.is_shutdown() and self._th_walk is not None:
                rospy.sleep(0.1)
            rospy.loginfo('Stopped')
            self.running = False

    def set_velocity(self, x, y, t):
        self.velocity = [x, y, t]

    def _do_walk(self):
        """Main walking loop

        Smoothly update velocity vectors and apply corresponding angles.
        """
        r = rospy.Rate(100)
        rospy.loginfo('Started walking thread')
        func = self.func

        # Global walk loop
        n = 50
        p = True
        i = 0
        self.current_velocity = [0, 0, 0]
        while (not rospy.is_shutdown() and
               (self.walking or i < n or self.is_walking())):
            if not self.walking:
                self.velocity = [0, 0, 0]
            if not self.is_walking() and i == 0:
                # Do not move if nothing to do and already at 0
                self.update_velocity(self.velocity, n)
                r.sleep()
                continue
            x = float(i) / n
            angles = func.get(p, x, self.current_velocity)
            self.update_velocity(self.velocity, n)
            self.darwin.set_angles(angles)
            i += 1
            if i > n:
                i = 0
                p = not p
            r.sleep()
        rospy.loginfo('Finished walking thread')

        self._th_walk = None

    def is_walking(self):
        e = 0.02
        for v in self.current_velocity:
            if abs(v) > e:
                return True
        return False

    def rescale(self, angles, coef):
        z = {}
        for j, v in angles.items():
            offset = self.ready_pos[j]
            v -= offset
            v *= coef
            v += offset
            z[j] = v
        return z

    def update_velocity(self, target, n):
        a = 3 / float(n)
        b = 1 - a
        t_and_v = zip(target, self.current_velocity)
        self.current_velocity = [a * t + b * v for (t, v) in t_and_v]

    def get_dist_to_ready(self):
        angles = self.darwin.get_angles()
        return get_distance(self.ready_pos, angles)


def interpolate(anglesa, anglesb, coefa):
    z = {}
    joints = anglesa.keys()
    for j in joints:
        z[j] = anglesa[j] * coefa + anglesb[j] * (1 - coefa)
    return z


def get_distance(anglesa, anglesb):
    d = 0
    joints = anglesa.keys()
    if len(joints) == 0:
        return 0
    for j in joints:
        d += abs(anglesb[j] - anglesa[j])
    d /= len(joints)
    return d


if __name__ == '__main__':
    rospy.init_node('walker')
    rospy.sleep(1)

    rospy.loginfo('Instantiating Robot Client')
    robot = PhantomX()
    rospy.loginfo('Instantiating Robot Walker')
    walker = Walker(robot)

    rospy.loginfo('Walker Ready')
    while not rospy.is_shutdown():
        rospy.sleep(1)
