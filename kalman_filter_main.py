import rospy;
import cv2;
import numpy as np;
from geometry_msgs.msg import Point32;
from geometry_msgs.msg import Polygon;



class Kalman_filter(object):
    def __init__(self):
        super(Kalman_filter, self).__init__();
        self.last_measurement = np.array((4, 1), np.float32);
        self.current_measurement = np.array((4, 1), np.float32);
        self.last_predicition = np.zeros((4, 1), np.float32);
        self.current_prediction = np.zeros((4, 1), np.float32);

        kalman = cv2.KalmanFilter(4, 4);
        kalman.measurementMatrix = np.array([[1, 0, 0, 0], [0, 1, 0, 0], [0, 0, 1, 0], [0, 0, 0, 1]], np.float32);
        kalman.transitionMatrix = np.array([[1, 0, 0, 0], [0, 1, 0, 0], [0, 0, 1, 0], [0, 0, 0, 1]], np.float32);
        kalman.processNoiseCov = np.array([[1, 0, 0, 0], [0, 1, 0, 0], [0, 0, 1, 0], [0, 0, 0, 1]], np.float32) * 0.3;


    def update(self, Z):
        self.last_measurement = self.current_measurement;
        self.last_prediction = self.current_prediction;
     
        self.current_measurement = Z;
        kalman.correct(current_measurement);
        self.current_prediction = kalman.predict();



class Subscriber(object):
    def __init__(self):
        super(Subscriber, self).__init__();
        rospy.init_node('kalman_filter_node', anonymous=True);
        self.kalman_filter = Kalman_filter();
        self.pub_kalman = rospy.Publisher('kalman_output', Polygon);

        rospy.Subscriber('cmt_output', Polygon, self.callback_cmt);
        rospy.spin();


            
    def callback_cmt(self, data):
        print "received data: ", data;
        width = max(abs(data[1].x - data[0].x), abs(data[2].x - data[0].x), abs(data[3].x - data[0].x));
        height = max(abs(data[1].y - data[0].y), abs(data[2].y - data[0].y), abs(data[3].y - data[0].y));
        center_x = min(data[0].x, data[1].x, data[2].x, data[3].x) + 0.5 * width;
        center_y = min(data[0].y, data[1].y, data[2].y, data[3].y) + 0.5 * height;
        
        Z = np.array([[np.float32(center_x)], [np.float32(center_y)], [np.float32(width)], [np.float32(height)]]);
        self.kalman.update(Z);

        new_data = Polygon();
        new_data[0].x = self.current_prediction[0];
        new_data[0].y = self.current_prediction[1];
        new_data[1].x = self.current_prediction[2];
        new_data[1].y = self.current_prediction[3];
        self.pub_kalman.publish(new_data);



if __name__ == '__main__':
    subscriber = Subscriber();



    
