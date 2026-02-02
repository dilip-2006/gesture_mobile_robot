#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import cv2
import mediapipe as mp

class GestureControlNode(Node):
    def __init__(self):
        super().__init__('gesture_control')
        self.publisher_ = self.create_publisher(Twist, 'cmd_vel', 10)
        self.timer = self.create_timer(0.1, self.timer_callback)
        
        self.mp_hands = mp.solutions.hands
        self.hands = self.mp_hands.Hands(
            max_num_hands=1,
            min_detection_confidence=0.7)
        self.mp_draw = mp.solutions.drawing_utils
        
        self.cap = cv2.VideoCapture(0)
        
        self.get_logger().info('Gesture Control Node Started. Show fingers to control: 1=Fwd, 2=Bwd, 3=Left, 4=Right, 5/0=Stop')

    def count_fingers(self, hand_landmarks):
        fingers = []
        
        
        tips = [8, 12, 16, 20]
        pips = [6, 10, 14, 18]
        
        for tip, pip in zip(tips, pips):
            if hand_landmarks.landmark[tip].y < hand_landmarks.landmark[pip].y:
                fingers.append(1)
            else:
                fingers.append(0)
        
        if hand_landmarks.landmark[4].x < hand_landmarks.landmark[3].x: 
             fingers.append(1)
        else:
             fingers.append(0)
             
        return sum(fingers)

    def timer_callback(self):
        ret, frame = self.cap.read()
        if not ret:
            return
            
        frame = cv2.flip(frame, 1)
        rgb_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        result = self.hands.process(rgb_frame)
        
        msg = Twist()
        gesture = "Stop"
        
        if result.multi_hand_landmarks:
            for hand_landmarks in result.multi_hand_landmarks:
                self.mp_draw.draw_landmarks(frame, hand_landmarks, self.mp_hands.HAND_CONNECTIONS)
                
                
                finger_count = 0
                
             
                lmList = []
                for id, lm in enumerate(hand_landmarks.landmark):
                    h, w, c = frame.shape
                    cx, cy = int(lm.x * w), int(lm.y * h)
                    lmList.append([id, cx, cy])
                    
                if len(lmList) != 0:
                    tips = [8, 12, 16, 20]
                    # Check 4 fingers
                    up_fingers = []
                    for tip in tips:
                        if lmList[tip][2] < lmList[tip-2][2]: 
                            up_fingers.append(1)
                        else:
                            up_fingers.append(0)
                   
                    count = sum(up_fingers)
                   
                    
                   
                    if lmList[4][0] > lmList[3][0]: 
                         pass
                         
                    finger_count = count
                    
                    if finger_count == 1:
                        msg.linear.x = 0.5
                        gesture = "Forward"
                    elif finger_count == 2:
                        msg.linear.x = -0.5
                        gesture = "Backward"
                    elif finger_count == 3:
                        msg.angular.z = 1.0
                        gesture = "Left"
                    elif finger_count == 4:
                        msg.angular.z = -1.0
                        gesture = "Right"
                    else:
                        msg.linear.x = 0.0
                        msg.angular.z = 0.0
                        gesture = "Stop"

                    if  msg.linear.x != 0 or msg.angular.z != 0:
                         self.get_logger().info(f'Publishing: {gesture} (Linear: {msg.linear.x}, Angular: {msg.angular.z})')


        self.publisher_.publish(msg)
        
        cv2.putText(frame, f'Gesture: {gesture}', (10, 50), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
        cv2.imshow("Hand Gesture Control", frame)
        cv2.waitKey(1)

def main(args=None):
    rclpy.init(args=args)
    node = GestureControlNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main()
