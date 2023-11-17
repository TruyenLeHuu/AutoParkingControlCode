import rospy
import time
import sys
from std_msgs.msg import String
from parameter import Parameters

class CarParking():
    def __init__(self):
        self.parking_params = Parameters()

        self.state = 0
        self.is_parking = 0

        self.steering_angle = 0
        self.heading_angle = 0
        self.brake_pressure = 0

        self.ros_init()

    def run(self):
        while (1):
            """ Start parking """
            while (self.is_parking == 1):
                """ State 1: steer to max steering angle """
                if (self.state == 1):
                    if (abs(self.steering_angle - self.parking_params.MAX_STEERING) > self.parking_params.ERROR_STEERING):
                        print("Please steer to ", self.parking_params.MAX_STEERING)
                    else:
                        self.state = 2
                """ State 2: move the car to curve """
                if (self.state == 2 ):
                    if (self.heading_angle < self.parking_params.HEADING_ANGLE):
                        print("Please release brake pedal and keep the steering angle the same")
                    else:
                        self.state = 3
                """ State 3: stop the car, return the steering angle to 0 degree """
                if (self.state == 3):
                    if (abs(self.steering_angle - self.parking_params.DEFAULT_STEERING) > self.parking_params.ERROR_STEERING):
                        print("Please apply brake and turn steering angle to ", self.parking_params.DEFAULT_STEERING)
                    else:
                        self.state = 4
                        self.last_time = time.time()
                """ State 4: move the car straight to parking slot, in a constant time """
                if (self.state == 4):
                    if (abs(self.brake_pressure - self.parking_params.DEFAULT_BRAKING) > self.parking_params.ERROR_BRAKING):
                        self.last_time = time.time()
                    if (time.time() - self.last_time < self.parking_params.DELAY_TIME):
                        # self.last_time = time.time()
                        print(time.time() - self.last_time)
                        print("Please release brake")
                    else:
                        print("Please apply brake")
                        self.is_parking = 0
                        self.state = 0
            print("Waiting for parking...")
            pass

    
    """ Create node ros and subscribe the topics """
    def ros_init(self):
        
        rospy.init_node("controller", anonymous=True)

        rospy.Subscriber("start_parking", String, self.start_parking_handle)
    
        rospy.Subscriber("steering_angle", String, self.ros_handle_steering_angle)

        rospy.Subscriber("heading_angle", String, self.ros_handle_heading_angle)

        rospy.Subscriber("brake_pressure", String, self.ros_handle_brake_pressure)
        
        # self.pub = rospy.Publisher('chosen_position', String, queue_size=10)

    """ Start parking process, switch state to 1 """
    def start_parking_handle(self, message):
        self.state = 1
        self.is_parking = 1

    def ros_handle_steering_angle(self, message):
        try:
            self.set_steering_angle(int(message.data))
        except Exception as e:
            print("Error:", str(e))

    def ros_handle_heading_angle(self, message):
        try:
            self.set_heading_angle(int(message.data))
        except Exception as e:
            print("Error:", str(e))

    def ros_handle_brake_pressure(self, message):
        try:
            self.set_brake_pressure(float(message.data))
        except Exception as e:
            print("Error:", str(e))

    def set_steering_angle(self, data):
        self.steering_angle = data

    def set_heading_angle(self, data):
        self.heading_angle = data

    def set_brake_pressure(self, data):
        self.brake_pressure = data

    def destroy(self):
        pass
if __name__ == '__main__':
    car_parking = CarParking()
    try:
        try:
            car_parking.run()
        except KeyboardInterrupt:
            sys.exit()
    finally:
        if car_parking is not None:
            car_parking.destroy()