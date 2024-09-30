#!/usr/bin/env python3
g=9.80665
import PID
import rospy
from geometry_msgs.msg import PoseStamped, Quaternion, Twist, Vector3Stamped, TwistStamped, TransformStamped
from mavros_msgs.msg import State, AttitudeTarget
from mavros_msgs.srv import CommandBool, CommandBoolRequest, SetMode, SetModeRequest
from nav_msgs.msg import Odometry
import math
import numpy as np
from pyquaternion import Quaternion as qt

def euler_from_quaternion(x, y, z, w):
        """
        Convert a quaternion into euler angles (roll, pitch, yaw)
        roll is rotation around x in radians (counterclockwise)
        pitch is rotation around y in radians (counterclockwise)
        yaw is rotation around z in radians (counterclockwise)
        """
        t0 = +2.0 * (w * x + y * z)
        t1 = +1.0 - 2.0 * (x * x + y * y)
        roll_x = math.atan2(t0, t1)
     
        t2 = +2.0 * (w * y - z * x)
        t2 = +1.0 if t2 > +1.0 else t2
        t2 = -1.0 if t2 < -1.0 else t2
        pitch_y = math.asin(t2)
     
        t3 = +2.0 * (w * z + x * y)
        t4 = +1.0 - 2.0 * (y * y + z * z)
        yaw_z = math.atan2(t3, t4)
     
        return [roll_x, pitch_y, yaw_z] # in radians


class Drone():

    def __init__(self):
        self.mpc_z_p=1
        self.mpc_xy_p=0.95
        self.mpc_z_vel_p_acc=4
        self.mpc_xy_vel_p_acc=1.8
        self.mpc_z_vel_i_acc=2
        self.mpc_xy_vel_i_acc=0.4
        self.mpc_z_vel_d_acc=0
        self.mpc_xy_vel_d_acc=0.2
        self.mpc_phi_p=1.0
        self.mpc_theta_p=1.0
        self.mpc_psi_p=1.0
        self.mass=1.535




        self.pos_control_obj=[PID.PID(self.mpc_xy_p, 0, 0), PID.PID(self.mpc_xy_p, 0, 0), PID.PID(self.mpc_z_p, 0, 0)]
        self.vel_control_obj=[PID.PID(self.mpc_xy_vel_p_acc, self.mpc_xy_vel_i_acc, 0), PID.PID(self.mpc_xy_vel_p_acc, self.mpc_xy_vel_i_acc, 0), PID.PID(self.mpc_z_vel_p_acc, self.mpc_z_vel_i_acc,0)]
        self.vel_control2_obj=[PID.PID(0, 0, self.mpc_xy_vel_d_acc), PID.PID(0, 0, self.mpc_xy_vel_d_acc), PID.PID(0, 0, self.mpc_z_vel_d_acc)]
        self.att_control_obj=[PID.PID(self.mpc_phi_p, 0, 0), PID.PID(self.mpc_theta_p, 0, 0), PID.PID(self.mpc_psi_p, 0, 0)]



        self.vel_pub=rospy.Publisher("/mavros/setpoint_velocity/cmd_vel_unstamped", Twist, queue_size=10)
        self.initial_altitude=2.0
        self.state_sub = rospy.Subscriber("mavros/state", State, callback = self.state_cb)
        self.currentPos = rospy.Subscriber(name="mavros/global_position/local", data_class=Odometry, queue_size=10, callback=self.pose_cb)
        self.local_pos_pub = rospy.Publisher("mavros/setpoint_position/local", PoseStamped, queue_size=10)
        self.accel_pub=rospy.Publisher("mavros/setpoint_accel/accel", Vector3Stamped, queue_size=10)
        self.vel_sub=rospy.Subscriber("/mavros/local_position/velocity_local", TwistStamped, queue_size=10, callback=self.vel_callback)
        self.att_pub=rospy.Publisher("/mavros/setpoint_raw/attitude", AttitudeTarget, queue_size=10)

       

        rospy.wait_for_service("/mavros/cmd/arming")
        self.arming_client = rospy.ServiceProxy("mavros/cmd/arming", CommandBool)

        rospy.wait_for_service("/mavros/set_mode")
        self.set_mode_client = rospy.ServiceProxy("mavros/set_mode", SetMode)

        self.current_state = State()

        self.pose_cmd=PoseStamped()
        self.accel_msg=Vector3Stamped()
        self.att_msg=AttitudeTarget()

        self.offb_set_mode = SetModeRequest()
        self.offb_set_mode.custom_mode = 'OFFBOARD'

        self.arm_cmd = CommandBoolRequest()
        self.arm_cmd.value = True

        self.xyz_act=[0, 0, 0]
        self.actual_psi=0.0
        self.desired_psi=0.0
        self.vxyz_cmd=[0, 0, 0]
        self.vel_msg=Twist()
        self.axyz_cmd=[0, 0, 0]
        self.vxyz_act=[0, 0, 0]
        self.vang_act=[0, 0, 0]
        self.phi_cmd=0.0
        self.phi_act=0.0
        self.theta_act=0.0
        self.theta_cmd=0.0
        self.a_cmd=[0, 0, 0]
        self.thrust=0.0
        self.body_rate_cmd=[0, 0, 0]

        self.mode=0 #0=default mode, 1=vicon mode

        self.vicon_xyz=[0,0,0]
        self.vicon_attitude=[0,0,0]
        self.conv_matrix=[]

        self.rate = rospy.Rate(20)
        self.initialise()
        self.arm()

    def vicon_callback(self,vicon_msg):
        self.vicon_xyz=[vicon_msg.transform.translation.x, vicon_msg.transform.translation.y,vicon_msg.transform.translation.z]
        self.vicon_attitude=euler_from_quaternion(vicon_msg.transform.rotation.x, vicon_msg.transform.rotation.y, vicon_msg.transform.rotation.z, vicon_msg.transform.rotation.w)

    def vel_callback(self, msg):
        self.vxyz_act=[msg.twist.linear.x, msg.twist.linear.y, msg.twist.linear.z]
        self.vang_act=[msg.twist.angular.x, msg.twist.angular.y, msg.twist.angular.z]

    def pose_update(self, x, y, z):
        self.pose_cmd.pose.position.x= x
        self.pose_cmd.pose.position.y = y
        self.pose_cmd.pose.position.z = z



    def state_cb(self, msg):

        self.current_state = msg

    def initialise(self):
        self.pose_update(0, 0, self.initial_altitude)
        
        while not self.current_state.connected:
            pass


        for i in range(100):
            self.local_pos_pub.publish(self.pose_cmd)
            self.rate.sleep()

        while(not self.current_state.mode=="OFFBOARD"):
            if(self.set_mode_client.call(self.offb_set_mode).mode_sent == True):
                rospy.loginfo("OFFBOARD enabled")
            self.rate.sleep()
    
    def arm(self):
        while not self.current_state.armed:
            if(self.arming_client.call(self.arm_cmd).success == True):
                    pass
            self.local_pos_pub.publish(self.pose_cmd)
            self.rate.sleep()
    


    def go_to_pose(self, x, y, z, psi=0.0):
        self.pose_update(x,y,z)
        self.pose_cmd.pose.orientation=Quaternion(0.0, 0.0, math.sin(0.5*psi), math.cos(0.5* psi))
        self.desired_psi=psi
        while not self.check_waypoint_reached():
            self.local_pos_pub.publish(self.pose_cmd)
            self.rate.sleep()



    def pose_cb(self, msg):
        self.xyz_act=[msg.pose.pose.position.x, msg.pose.pose.position.y, msg.pose.pose.position.z]
        q0, q1, q2, q3 = (
            msg.pose.pose.orientation.w,
            msg.pose.pose.orientation.x,
            msg.pose.pose.orientation.y,
            msg.pose.pose.orientation.z,
        )


        self.actual_psi = math.atan2((2 * (q0 * q3 + q1 * q2)),
                    (1 - 2 * (pow(q2, 2) + pow(q3, 2))))
        self.theta_act=math.asin(2*(q0 *q2 - q3*q1))
        self.phi_act=math.atan2((2 * (q0 * q1 + q3 * q2)),
                    (1 - 2 * (pow(q2, 2) + pow(q1, 2))))
        
        
    def check_waypoint_reached(self, pos_tol=0.3, head_tol=0.01):
    
        
        self.local_pos_pub.publish(self.pose_cmd)

        xyz_temp=self.xyz_act if self.mode==0 else self.vicon_xyz

        dx = abs(self.pose_cmd.pose.position.x - xyz_temp[0])
        dy = abs(self.pose_cmd.pose.position.y - xyz_temp[1])
        dz = abs(self.pose_cmd.pose.position.z - xyz_temp[2])

        dMag = math.sqrt(pow(dx, 2) + pow(dy, 2) + pow(dz, 2))

        cosErr = math.cos(self.actual_psi) - math.cos(self.desired_psi)

        sinErr = math.sin(self.actual_psi) - math.sin(self.desired_psi)

        dHead = math.sqrt(pow(cosErr, 2) + pow(sinErr, 2))

        if dMag < pos_tol and dHead < head_tol:
            return 1
        else:
            return 0
    
    def check_position_reached(self, pos_tol=0.1):
        
        self.local_pos_pub.publish(self.pose_cmd)

        self.xyz_temp=self.xyz_act if self.mode==0 else self.vicon_xyz

        dx = abs(self.pose_cmd.pose.position.x - self.xyz_temp[0])
        dy = abs(self.pose_cmd.pose.position.y - self.xyz_temp[1])
        dz = abs(self.pose_cmd.pose.position.z - self.xyz_temp[2])

        dMag = math.sqrt(pow(dx, 2) + pow(dy, 2) + pow(dz, 2))

        if dMag < pos_tol:
            return 1
        else:
            return 0
        
    def check_yaw_reached(self, head_tol=0.01):
        
        self.local_pos_pub.publish(self.pose_cmd)

        cosErr = math.cos(self.actual_psi) - math.cos(self.desired_psi)

        sinErr = math.sin(self.actual_psi) - math.sin(self.desired_psi)

        dHead = math.sqrt(pow(cosErr, 2) + pow(sinErr, 2))

        if dHead < head_tol:
            return 1
        else:
            return 0
        
    def check_velocity_reached(self, vel_tol=0.02):
        
        self.local_pos_pub.publish(self.pose_cmd)

        dx = abs(self.vxyz_cmd[0] - self.vxyz_act[0])
        dy = abs(self.vxyz_cmd[1] - self.vxyz_act[1])
        dz = abs(self.vxyz_cmd[2] - self.vxyz_act[2])

        dMag = math.sqrt(pow(dx, 2) + pow(dy, 2) + pow(dz, 2))
       # print(dMag)
        if dMag < vel_tol:
            return 1
        else:
            return 0
        

    def publish_velocity(self,vx,vy,vz):
        self.vxyz_cmd=[vx, vy, vz]
        self.vel_msg.linear.x=vx
        self.vel_msg.linear.y=vy
        self.vel_msg.linear.z=vz
        self.vel_pub.publish(self.vel_msg)
        self.rate.sleep()

    def publish_acceleration(self, ax, ay, az):
        self.accel_msg.vector.x=ax
        self.accel_msg.vector.y=ay
        self.accel_msg.vector.z=az
        self.accel_pub.publish(self.accel_msg)
        self.rate.sleep()

    def pos_controller(self, x, y, z):
        self.pose_update(x,y,z)
        v=[0, 0, 0]
        xyz_sp=[x,y,z]
        for i in range(3): self.pos_control_obj[i].SetPoint=xyz_sp[i]
            
        while not self.check_position_reached():
            for i in range(3): 
                self.pos_control_obj[i].update(self.xyz_temp[i])
                v[i]=self.pos_control_obj[i].output

            print("xyz= ", self.xyz_act)
            # self.vel_controller(v[0],v[1],v[2])
            self.publish_velocity(v[0],v[1],v[2]) 


    def desired_attitude(self,a):
        self.phi_cmd=1/g * (a[0]* math.sin(self.desired_psi)- a[1]* math.cos(self.desired_psi))    
        self.theta_cmd=1/g * (a[0]* math.cos(self.desired_psi)+ a[1]* math.sin(self.desired_psi))  
        self.conv_matrix=np.array([[math.cos(self.theta_act), 0.0, -math.cos(self.phi_act)* math.sin(self.theta_act)],
                                  [0.0, 1.0, math.sin(self.phi_act)],
                                  [math.sin(self.theta_act), 0, math.cos(self.theta_act)* math.cos(self.phi_act)]])
        # print(self.conv_matrix)
        self.attitude_controller()

    def attitude_controller(self):
        att_sp=[self.phi_cmd, self.theta_cmd, self.desired_psi]
        att_act=[self.phi_act, self.theta_act, self.actual_psi]
        att_d=np.array([0.0, 0.0, 0.0])
        for i in range(3): 
            self.att_control_obj[i].SetPoint=att_sp[i]
            self.att_control_obj[i].update(att_act[i])
            att_d[i]=self.att_control_obj[i].output


        self.body_rate_cmd=np.array(np.matmul(self.conv_matrix, att_d.T)).T
        self.att_msg.body_rate.x, self.att_msg.body_rate.y, self.att_msg.body_rate.z=self.body_rate_cmd
        self.att_msg.type_mask=128
        self.att_msg.thrust=self.thrust
        self.att_pub.publish(self.att_msg)     
        #print(self.att_msg)
        self.rate.sleep()   


    def vel_controller(self, vx, vy, vz):
        v_sp=[vx, vy, vz]
        self.vxyz_cmd=v_sp
        v_sp_2=[0, 0, 0]
        a=[0, 0, 0]
        for i in range(3):
            self.vel_control2_obj[i].SetPoint=v_sp_2[i]
            self.vel_control_obj[i].SetPoint=v_sp[i]
        # while not self.check_velocity_reached():
            # for i in range(3):
            self.vel_control_obj[i].update(self.vxyz_act[i])
            self.vel_control2_obj[i].update(self.vxyz_act[i])
            a[i]=self.vel_control_obj[i].output+self.vel_control2_obj[i].output
        self.a_cmd=a
        self.thrust=self.mass*(g+a[2])
            # print(self.vxyz_act)
        self.desired_attitude(a)
        # self.publish_acceleration(a[0], a[1], a[2])

            
def main():
    rospy.init_node("drone")
    drone=Drone()
    
    while not rospy.is_shutdown():
        drone.desired_psi=0
        drone.pos_controller(5,5,16)
        print("vxyz= ", drone.xyz_act, "\nheading= ", drone.actual_psi)
        drone.rate.sleep()
        rospy.spin()

if __name__=="__main__":
    main()

