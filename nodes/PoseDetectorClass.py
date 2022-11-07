# https://google.github.io/mediapipe/solutions/face_mesh#python-solution-api
# https://github.com/nicknochnack/MediaPipePoseEstimation/blob/main/Media%20Pipe%20Pose%20Tutorial.ipynb


import cv2
import mediapipe as mp
from math import dist, sqrt # distace between two points


class PoseDetector():
 
    def __init__(self, frame_shape):

        # centerpoint
        self.frame_shape = frame_shape
        self.center_point = (int(frame_shape[0] / 2), int(frame_shape[1] / 2)) 
        
        # mediapipe setup
        #self.mp_drawing = mp.solutions.drawing_utils
        self.mp_pose = mp.solutions.pose
        self.mp_pose_process = self.mp_pose.Pose(min_detection_confidence=0.5, min_tracking_confidence=0.5)


    def process_view(self, frame):

        # analyse frame for body landmarks
        landmarks, results = self.get_body_landmarks(frame)
   
        # check for body landmarks 
        if landmarks is not None:
            nose, shoulder, hip, length = self.get_body_position_absolut(landmarks, self.frame_shape) 
            frame = self.draw_body_landmarks(frame, nose, shoulder, hip) 
            nose, shoulder, hip = self.get_body_position_relativ(nose, shoulder, hip)
            frame = self.write_relativ_body_position_on_frame(frame, nose, length)
            positions = (nose[0],nose[1],length)
        else: 
            positions = None

        return frame, positions   


    def get_body_landmarks(self, frame):

        # Recolor image to RGB
        frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        frame.flags.writeable = False
    
        # Make detection
        results = self.mp_pose_process.process(frame)
        # Recolor back to BGR
        frame.flags.writeable = True

        # Extract landmarks
        try:
            landmarks = results.pose_landmarks.landmark
            return landmarks, results
        except:
            return None, None

    
    def get_body_position_absolut(self, landmarks, shape):
        
        # get positions relativ to frame shape (0,0 --> upper left corner; 1,1 --> bottom right corner)
        nose = [landmarks[self.mp_pose.PoseLandmark.NOSE.value].x,landmarks[self.mp_pose.PoseLandmark.NOSE.value].y]
        shoulder = [landmarks[self.mp_pose.PoseLandmark.LEFT_SHOULDER.value].x,landmarks[self.mp_pose.PoseLandmark.LEFT_SHOULDER.value].y]
        hip = [landmarks[self.mp_pose.PoseLandmark.LEFT_HIP.value].x,landmarks[self.mp_pose.PoseLandmark.LEFT_HIP.value].y]
 
        # multiply relativ coords by frame shape to get position in pixel
        nose_x = int((nose[0]*shape[1]))
        nose_y = int((nose[1]*shape[0]))
        shoulder_x = int(shoulder[0]*shape[1])
        shoulder_y = int(shoulder[1]*shape[0])
        hip_x = int(hip[0]*shape[1])
        hip_y = int(hip[1]*shape[0])
        length = int(dist([shoulder_x, shoulder_y], [hip_x, hip_y]))

        return [nose_x, nose_y], [shoulder_x, shoulder_y], [hip_x, hip_y], length

    
    def get_body_position_relativ(self, nose, shoulder, hip):
        
        nose_x = nose[0] - self.center_point[1]
        nose_y = self.center_point[0] - nose[1]
        shoulder_x = shoulder[0] - self.center_point[1]
        shoulder_y = shoulder[1] - self.center_point[0]
        hip_x = hip[0] - self.center_point[1]
        hip_y = hip[1] - self.center_point[0]

        return [nose_x, nose_y], [shoulder_x, shoulder_y], [hip_x, hip_y]


    def draw_body_landmarks(self, frame, nose, shoulder, hip):

        cv2.circle(frame, nose, radius=5, color=(225, 200, 0), thickness=10) 
        cv2.line(frame, shoulder, hip, color=(0, 160, 255), thickness=8)
        cv2.circle(frame, shoulder, radius=5, color=(0, 0, 255), thickness=10) 
        cv2.circle(frame, hip, radius=5, color=(0, 255, 255), thickness=10) 
        return frame

    
    def write_relativ_body_position_on_frame(self, frame, nose, length):

        cv2.putText(frame, f'x: {nose[0]} pix', (20, 40), cv2.FONT_HERSHEY_PLAIN,1, (0, 255, 0), 1) # bgr
        cv2.putText(frame, f'y: {nose[1]} pix', (20, 60), cv2.FONT_HERSHEY_PLAIN,1, (255, 0, 0), 1) 
        cv2.putText(frame, f'l: {length} pix', (20, 80), cv2.FONT_HERSHEY_PLAIN,1, (0, 160, 255), 1)
        return frame
        
  

if __name__ == '__main__':
    
    cap = cv2.VideoCapture(0)
    ret, frame = cap.read()

    frame_shape = frame.shape
    #frame_width = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH) / 2)

    #goal = (0,0,40) # goal values (x,y,z) --> read values from yaml file
    roi = (4,145,374) # regions of interest (face_landmarks nose, left_eye, right_eye)

    detector = PoseDetector(frame_shape)
    
    while cap.isOpened():

        success, frame = cap.read()
        if not success:
            print("Ignoring empty camera frame.")
            continue

        # get analysed image from FaceDetector
        face_found, frame, positions  = detector.process_view(frame)
        
        cv2.imshow('FaceDetector', frame)
        
        if cv2.waitKey(1) & 0xFF == 27:
            cap.release()
            cv2.destroyAllWindows()
            break