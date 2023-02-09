import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Pose2D
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
from nav_msgs.msg import Odometry
import math


class ControlTurtle(Node):

    def __init__(self):
        super().__init__('control_turtle')
        self.init_variables()
        self.init_publishers()
        self.init_subscribers()        

    def init_variables(self):
        self.x = -2.0
        self.x_error = 0.0
        self.x_goal = 1.70      
        self.y = -0.5 
        self.y_error = 0.0
        self.y_goal = -0.50
        self.k_omega = 1.5 
        self.theta = 0.0 
        self.p = 0.0
        self.alpha = 0.0
        self.v_max = 0.25
        
        rows = 8
        cols = 2
 
        self.mat = [[0]*cols for _ in range(rows)]
        # editing the individual elements
        self.mat[0][0], self.mat[0][1] = 1.70,-0.50
        self.mat[1][0], self.mat[1][1] = 1.70,1.55
        self.mat[2][0], self.mat[2][1] = 0.55,1.55
        self.mat[3][0], self.mat[3][1] = 0.55,-0.55
        self.mat[4][0], self.mat[4][1] = -0.55,-0.55
        self.mat[5][0], self.mat[5][1] = -0.55,1.65
        self.mat[6][0], self.mat[6][1] = -1.40,1.65
        self.mat[7][0], self.mat[7][1] = -2.00,-0.50
        
        self.isNew = False
        self.line_count = 0
        print(f'matrix with dimension {rows} x {cols} is {self. mat}')

        self.euler_x = 0.0
        self.euler_y = 0.0
        self.euler_z = 0.0


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
        
            return roll_x, pitch_y, yaw_z # in radians


    def init_publishers(self):
        self.publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def init_subscribers(self):
        self.subscription = self.create_subscription(Odometry,'/odom', self.listener_callback, 10)
        self.subscription_pose = self.create_subscription(Pose2D,'goal',self.goal_callback,10)
        # Prevenir possíveis warnings de variáveis não utilizadas
        self.subscription 
        self.subscription_pose

    def listener_callback(self, msg):
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y
        ori_x = msg.pose.pose.orientation.x
        ori_y = msg.pose.pose.orientation.y
        ori_z = msg.pose.pose.orientation.z
        ori_w = msg.pose.pose.orientation.w
        
        t0 = +2.0 * (ori_w * ori_x + ori_y * ori_z)
        t1 = +1.0 - 2.0 * (ori_x * ori_x + ori_y * ori_y)
        self.euler_x = math.atan2(t0, t1)
    
        t2 = +2.0 * (ori_w * ori_y - ori_z * ori_x)
        t2 = +1.0 if t2 > +1.0 else t2
        t2 = -1.0 if t2 < -1.0 else t2
        self.euler_y = math.asin(t2)
    
        t3 = +2.0 * (ori_w * ori_z + ori_x * ori_y)
        t4 = +1.0 - 2.0 * (ori_y * ori_y + ori_z * ori_z)
        self.theta = math.atan2(t3, t4)

        # self.euler_x, self.euler_y, self.euler_z = self.euler_from_quaternion(ori_x,ori_y,ori_z,ori_w)

        #self.get_logger().info('Value: "%f"' % self.x )


    def goal_callback(self, msg):
        self.x_goal = msg.x
        self.y_goal = msg.y

    def timer_callback(self):
        msg = Twist()

        if(self.isNew):
            self.x_goal = self.mat[self.line_count][0]
            self.y_goal = self.mat[self.line_count][1]
            self.line_count = self.line_count + 1
            if self.line_count == 8:
                self.line_count = 0
            self.isNew = False
            
        # Calculando primeiramente os erros de localização do robô
        self.x_error = self.x_goal - self.x
        self.y_error = self.y_goal - self.y
        self.get_logger().info('X erro: "%f"' % self.x_error)
        self.get_logger().info('Y erro: "%f"' % self.y_error)
        self.get_logger().info('X Goal: "%f"' % self.x_goal)
        self.get_logger().info('Y Goal: "%f"' % self.y_goal)
        if not(abs(self.x_error) < 0.15 and abs(self.y_error) < 0.15):
            self.p = math.sqrt(pow(self.x_error, 2) + pow(self.y_error, 2))
            self.alpha = math.atan2(self.y_error,self.x_error) - (self.theta)
        else:
            self.p = 0.0
            self.alpha = 0.0
            self.isNew = True
        msg.linear.x = self.v_max * math.tanh(self.p)
        msg.linear.y = 0.0
        msg.angular.z = self.k_omega * self.alpha
        if msg.linear.x > 0.25:
            msg.linear.x = 0.25
        self.publisher.publish(msg)


def main(args=None):
    rclpy.init(args=args)

    turtle_object = ControlTurtle()

    rclpy.spin(turtle_object)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    turtle_object.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
