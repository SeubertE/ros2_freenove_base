#!/usr/bin/env python

""" interface2driver

This is a port of the interface2driver.py from EE447 Lab 2 into ROS 2. Certain
interfaces have been disabled or neglected as they were removed from our
version of our robot.

"""
import queue
import rclpy
from rclpy.node import Node
from freenove_interfaces.msg import ADC as ADCMsg, Buzzer as BuzzerMsg, LED as LEDMsg, LineTracker as LineTrackerMsg, Motor as MotorMsg, Servo as ServoMsg
from freenove_interfaces.srv import Ultrasonic
from .driver4hardware import *

__author__ = "Eugene Seubert"
__email__ = "seuberte@uw.edu"

class interface2driver(Node):
    def __init__(self):
        super().__init__('interface2driver')
        #======================
        # Subscriber 
        #======================
        # Motor initialize
        self.motor = Motor()
        self.motor.setMotorModel(0,0,0,0)
        #self.motor_sub = rospy.Subscriber(TOPIC["motor_topic"],motor_msg,
        #                    self.motor_callback, queue_size=3)        
        self.motor_sub = self.create_subscription(MotorMsg, 'motor_topic', 
            self.motor_callback, 10)

        # Servo initialize
        self.servo = Servo()
        #self.servo_sub =  rospy.Subscriber(TOPIC["servo_topic"],servo_msg,
        #                    self.servo_callback, queue_size=3)
        self.servo_sub = self.create_subscription(ServoMsg, 'servo_topic', 
            self.servo_callback, 10)

        # Buzzer initialize
        self.buzzer = Buzzer()
        #self.buzzer_sub =  rospy.Subscriber(TOPIC["buzzer_topic"],buzzer_msg,
        #                    self.buzzer_callback, queue_size=3)
        self.buzzer_sub = self.create_subscription(BuzzerMsg, 'buzzer_topic', 
            self.buzzer_callback, 10)
        
        # LED iniialize
        self.led = Led()
        #self.led_sub = rospy.Subscriber(TOPIC["led_topic"], led_msg,
        #                    self.led_callback, queue_size=3)
        self.led_sub = self.create_subscription(LEDMsg, 'led_topic', 
            self.led_callback, 10)

        #======================
        # Publisher
        #======================
        # Line tracking sensor initialize
        self.line = Line_Tracking()
        #self.line_tracking_pub = rospy.Publisher(TOPIC["line_tracking_topic"], 
        #                            line_tracking_msg, queue_size=3)
        self.line_tracking_pub = self.create_publisher(LineTrackerMsg, 
            'line_tracking_topic', 10)

        # ADC initialize
        self.adc = Adc()
        #self.adc_pub =  rospy.Publisher(TOPIC["adc_topic"],adc_msg, queue_size=3) 
        self.adc_pub = self.create_publisher(ADCMsg, 'adc_topic', 10)

        #======================
        # service 
        #======================
        # Ultrasonic        
        #self.ultrasonic = Ultrasonic()
        #self.ultrasonic_run = rospy.Service(TOPIC["ultrasonic_topic"],ultrasonic_srv,
        #                        self.ultrasonic_callback)
        #self.ultrasonic = self.create_service(ultrasonic_topic, 'ultrasonic_srv', self.ultrasonic_callback, 10)

        #Set up rate for update publishers 
        # Update 30 times per second
        #self.rate = rospy.Rate(30)
        timer_freq = 30
        timer_period = 1/timer_freq
        self.timer = self.create_timer(timer_period, self.timer_callback)

    # callback motors
    def motor_callback(self, msg):
        self.motor.setMotorModel(msg.left_upper_wheel,
                            msg.left_lower_wheel,
                            msg.right_upper_wheel,
                            msg.right_lower_wheel)

    # callback servo
    def servo_callback(self, msg):   
        self.servo.setServoPwm('0',msg.horizontal)
        self.servo.setServoPwm('1',msg.vertical)

    # callback for buzzer
    def buzzer_callback(self,msg):
        self.buzzer.run(msg.command)

    # callback for ultrasonic
    # if request is activate==1, run ultrasonic algorithm
    #def ultrasonic_callback(self,srv):
    #    if srv.activate:
    #        self.ultrasonic.get_distance()
    #        return self.ultrasonic.get_distance()

    # LED callback
    def led_callback(self, msg):
        self.led.changeLED(msg.led, msg.r, msg.g, msg.b)
    
    # regularly updating publishers
    def timer_callback(self):
        line_msg = LineTrackerMsg()
        line_msg.left, line_msg.mid, line_msg.right = self.line.signal()
        self.line_tracking_pub.publish(line_msg)

        volt_msg = ADCMsg()
        volt_msg.left=self.adc.recvADC(0)
        volt_msg.right=self.adc.recvADC(1)
        volt_msg.power=self.adc.recvADC(2)*3
        self.adc_pub.publish(volt_msg)
    
    # This will be invoked before actual shutdown occurs
    # prevent motor keep running
    def end(self):
        self.motor.setMotorModel(0,0,0,0)

def main(args=None):
    rclpy.init(args=args)
    print("start interface")
    
    interface_for_driver = interface2driver()

    rclpy.spin(interface_for_driver)

    interface2driver.end()
    interface_for_driver.destroy_node()
    rclpy.shutdown()

    #rospy.init_node('interface2driver', anonymous=True)
    #TOPIC ={}
    #TOPIC["motor_topic"] = rospy.get_param("~motor_topic",'/car/hardware/motor')
    #TOPIC["servo_topic"] = rospy.get_param("~servo_topic",'/car/hardware/servo')
    #TOPIC["buzzer_topic"] = rospy.get_param("~buzzer_topic",'/car/hardware/buzzer')
    #TOPIC["led_topic"] = rospy.get_param("~led_topic",'/car/hardware/led')
    #TOPIC["line_tracking_topic"] = rospy.get_param("~line_tracking_topic",'/car/hardware/line_tracking')
    #TOPIC["adc_topic"] = rospy.get_param("~adc_topic",'/car/hardware/adc')
    #TOPIC["ultrasonic_topic"] = rospy.get_param("~ultrasonic_topic",'/car/hardware/ultrasonic')
    #run = interface2driver(TOPIC)  
    # Keeping this node is activated
    #while not rospy.is_shutdown():        
    #    run.update()        
    # prevent motor keep running
    #run.end()   
    
    print("\nexit interface")

if __name__ == '__main__':
    main()
