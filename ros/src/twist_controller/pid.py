
#import rospy
MIN_NUM = float('-inf')
MAX_NUM = float('inf')


class PID(object):
    def __init__(self, kp, ki, kd, mn=MIN_NUM, mx=MAX_NUM):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.min = mn
        self.max = mx

        self.int_val = self.last_int_val = self.last_error = 0.

    def reset(self):
        self.int_val = 0.0
        self.last_int_val = 0.0

    def step(self, error,  sample_time):
        self.last_int_val = self.int_val

        integral = self.int_val + error * sample_time
        derivative = (error - self.last_error) / sample_time
        self.last_error = error


        y = self.kp * error + self.ki * self.int_val + self.kd * derivative
        val = y #max(self.min, min(y, self.max))

        if val > self.max:
            val = self.max   
            #self.int_val = integral - (y - val)/ki          
        elif val < self.min:
            val = self.min
            #self.int_val = integral - (y - val)/ki          
        else:
            self.int_val = integral
        #rospy.loginfo("SPD_CTRL: last {0} current{1}  val{2}".format(self.last_error, error,val))
        return val
