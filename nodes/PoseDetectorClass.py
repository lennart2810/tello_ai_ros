# https://google.github.io/mediapipe/solutions/face_mesh#python-solution-api
# https://github.com/nicknochnack/MediaPipePoseEstimation/blob/main/Media%20Pipe%20Pose%20Tutorial.ipynb


import cv2
import mediapipe as mp
from math import dist, sqrt # distace between two points


class PoseDetector():
 
    def __init__(self, frame_shape):
        
        #self.goal = goal # goal values (x,y,z)
        #self.roi = roi # regions of interest (face_landmarks nose, left_eye, right_eye)

        # centerpoint
        self.frame_shape = frame_shape
        self.center_point = (int(frame_shape[1] / 2), int(frame_shape[0] / 2)) # (height, width) or vica versa?!
        
        # mediapipe setup
        self.face_mesh = mp.solutions.face_mesh.FaceMesh(max_num_faces=1,
                                                         refine_landmarks=True,
                                                         min_detection_confidence=0.5,
                                                         min_tracking_confidence=0.5)
        self.mp_drawing = mp.solutions.drawing_utils
        self.mp_pose = mp.solutions.pose
        self.mp_pose_process = self.mp_pose.Pose(min_detection_confidence=0.5, min_tracking_confidence=0.5)
                                                    
    def process_view(self, frame):

        # analyse frame for body landmarks
        landmarks, results = self.get_body_landmarks(frame)
   
        # check for facial landsmarks 
        if landmarks is not None:

            object_detected = True

            #positions, shoulder, hip = self.get_body_position(landmarks, frame.shape)
            #[nose_x, nose_y], [shoulder_x, shoulder_y], [hip_x, hip_y], length
            nose, shoulder, hip, length = self.get_body_position(landmarks, frame.shape)
            positions = (nose[0],nose[1],length)
            
            self.draw_body_landmarks(frame, results, nose, shoulder, hip, length)

        else: 
            object_detected = False
            positions = None
        

        # positions: x and y of nose relativ to frame center point [pixel] and lenght of shoulder to hip [pixel]

        return frame, positions   

    def get_body_landmarks(self, frame):

        # Recolor image to RGB
        frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        frame.flags.writeable = False
    
        # Make detection
        results = self.mp_pose_process.process(frame)
        # Recolor back to BGR
        frame.flags.writeable = True
        #frame = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)

        # Extract landmarks
        try:
            landmarks = results.pose_landmarks.landmark
            return landmarks, results
        except:
            return None, None

    def draw_body_landmarks(self, frame, results, nose, shoulder, hip, length):

        # Render detections
        # self.mp_drawing.draw_landmarks(frame, results.pose_landmarks, self.mp_pose.POSE_CONNECTIONS,
        #                         self.mp_drawing.DrawingSpec(color=(245,117,66), thickness=2, circle_radius=2), 
        #                         self.mp_drawing.DrawingSpec(color=(245,66,230), thickness=2, circle_radius=2))  

        nose_x = int(nose[0] + (frame.shape[1]/2)) # why calc necessary ? error in get body position?
        nose_y = int((frame.shape[0]/2)-nose[1])
        nose = [nose_x, nose_y]

        cv2.circle(frame, nose, radius=5, color=(225, 200, 0), thickness=10) 
        cv2.line(frame, shoulder, hip, color=(0, 160, 255), thickness=8)
        cv2.circle(frame, shoulder, radius=5, color=(0, 0, 255), thickness=10) 
        cv2.circle(frame, hip, radius=5, color=(0, 255, 255), thickness=10) 

        cv2.putText(frame, f'x: {nose_x} pix', (20, 40), cv2.FONT_HERSHEY_PLAIN,1, (0, 255, 0), 1) # bgr
        cv2.putText(frame, f'y: {nose_y} pix', (20, 60), cv2.FONT_HERSHEY_PLAIN,1, (255, 0, 0), 1) 
        cv2.putText(frame, f'l: {length} pix', (20, 80), cv2.FONT_HERSHEY_PLAIN,1, (0, 160, 255), 1)
        
        return frame
   
    def get_body_position(self, landmarks, shape):

        nose = [landmarks[self.mp_pose.PoseLandmark.NOSE.value].x,landmarks[self.mp_pose.PoseLandmark.NOSE.value].y]
        shoulder = [landmarks[self.mp_pose.PoseLandmark.LEFT_SHOULDER.value].x,landmarks[self.mp_pose.PoseLandmark.LEFT_SHOULDER.value].y]
        hip = [landmarks[self.mp_pose.PoseLandmark.LEFT_HIP.value].x,landmarks[self.mp_pose.PoseLandmark.LEFT_HIP.value].y]
        
        nose_x = int((nose[0]*shape[1])-(shape[1]/2))
        nose_y = int((shape[0]/2)-(nose[1]*shape[0]))
        shoulder_x = int(shoulder[0]*shape[1])
        shoulder_y = int(shoulder[1]*shape[0])
        hip_x = int(hip[0]*shape[1])
        hip_y = int(hip[1]*shape[0])

        length = int(dist([shoulder_x, shoulder_y], [hip_x, hip_y]))
        #length = sqrt((shoulder_x-hip_x)**2 + (shoulder_y-hip_y)**2)
        positions = (nose_x, nose_y, length)

        return [nose_x, nose_y], [shoulder_x, shoulder_y], [hip_x, hip_y], length
        

def main():

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
        
if __name__ == '__main__':
    main()