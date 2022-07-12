import cv2
import mediapipe as mp
from math import dist # distace between two points

# mp_drawing = mp.solutions.drawing_utils
# mp_drawing_styles = mp.solutions.drawing_styles
# mp_face_mesh = mp.solutions.face_mesh

class FaceDetector():
 
    def __init__(self, frame_shape, roi):
        
        #self.goal = goal # goal values (x,y,z)
        self.roi = roi # regions of interest (face_landmarks nose, left_eye, right_eye)

        # centerpoint
        self.center_point = (int(frame_shape[1] / 2), int(frame_shape[0] / 2)) # (height, width) or vica versa?!
        
        # mediapipe setup
        self.face_mesh = mp.solutions.face_mesh.FaceMesh(max_num_faces=1,
                                                         refine_landmarks=True,
                                                         min_detection_confidence=0.5,
                                                         min_tracking_confidence=0.5)
    
    def get_face_position(self, landmarks):
        
        # calc pixels between nose and centerpoint
        nose = landmarks[0]
        x = int(dist((nose[0],self.center_point[1]), self.center_point)) # assume nose_y = center_point_y
        y = int(dist((self.center_point[0],nose[1]), self.center_point)) # assume nose_x = center_point_x
        
        if nose[0] >= self.center_point[0]:
            x = -x
        if nose[1] >= self.center_point[1]:
            y = -y
        
        # calc pixels between both eyes 
        eye_l = landmarks[1]
        eye_r = landmarks[2]
        z = int(dist(eye_l, eye_r))
        
        return (x, y, z)
        
    def get_face_landmarks(self, frame):

        shape = frame.shape
        image = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        results = self.face_mesh.process(image)
        points = []
        
        # check for face with landmarks
        if results.multi_face_landmarks: 
            
            # iterate over all faces in results
            for face in results.multi_face_landmarks: 
                
                # iterate over all interesting face_landmarks
                for landmark in self.roi: 
                    
                    # get landmark points in frame
                    # lm.x and lm.y are relative to frame.shape --> multiply by shape to get absolute points
                    lm = face.landmark[landmark]
                    x = int(lm.x * shape[1])
                    y = int(lm.y * shape[0])
                    points.append((x,y))

            return points
        
        else:
            return None

    def draw_infos_on_frame(self, img, landmarks, positions):

        # draw interesting landmarks (nose and eyes)
        for lm in landmarks:
            cv2.circle(img, lm, radius=5, color=(225, 0, 100), thickness=2) 

        # draw goal_arrow (startpoint = nose, endpoint = centerpoint)
        cv2.arrowedLine(img, landmarks[0], self.center_point, color=(0, 255, 0), thickness=5)
        # draw monobraue (eye to eye)
        cv2.line(img, landmarks[1], landmarks[2], color=(0, 0, 0), thickness=8)

        # draw crosshair (Fadenkreuz)
        cv2.line(img, (self.center_point[0]-30, self.center_point[1]), (self.center_point[0]+30, self.center_point[1]), color=(0, 0, 0), thickness=2)
        cv2.line(img, (self.center_point[0],self.center_point[1]-30), (self.center_point[0],self.center_point[1]+30), color=(0, 0, 0), thickness=2)
        cv2.circle(img, self.center_point, radius=25, color=(0, 0, 0), thickness=2)
        cv2.circle(img, self.center_point, radius=18, color=(0, 0, 0), thickness=2)
        cv2.circle(img, self.center_point, radius=1, color=(0, 0, 255), thickness=5)

        # flip image
        #img = cv2.flip(img, 1)

        # display positions
        cv2.putText(img, f'x: {positions[0]} pix', (20, 40), cv2.FONT_HERSHEY_PLAIN,1, (255, 255, 255), 1)
        cv2.putText(img, f'y: {positions[1]} pix', (20, 60), cv2.FONT_HERSHEY_PLAIN,1, (255, 255, 255), 1)
        cv2.putText(img, f'z: {positions[2]} pix', (20, 80), cv2.FONT_HERSHEY_PLAIN,1, (255, 255, 255), 1)
        
        # display errors
        #cv2.putText(img, f'x: {errors[0]} pix', (20, 100), cv2.FONT_HERSHEY_PLAIN,1, (0, 0, 255), 1)
        #cv2.putText(img, f'y: {errors[1]} pix', (20, 120), cv2.FONT_HERSHEY_PLAIN,1, (0, 0, 255), 1)
        #cv2.putText(img, f'z: {errors[2]} pix', (20, 140), cv2.FONT_HERSHEY_PLAIN,1, (0, 0, 255), 1)

        # calculate and display FPS
        # cTime = time.time()
        # fps = 1.0 / (cTime - pTime)
        # pTime = cTime
        # cv2.putText(image, f'FPS: {int(fps)}', (20, 20), cv2.FONT_HERSHEY_PLAIN,1, (255, 255, 255), 1)

        return img


    def process_view(self, frame):
 
        # analyse frame for facial landmarks
        landmarks = self.get_face_landmarks(frame)
   
        # check for facial landsmarks 
        if landmarks is not None:

            face_found = True

            # calculate face position (x,y for nose position and z for distance between eyes)
            positions = self.get_face_position(landmarks)

            # draw some infos on the frame
            frame = self.draw_infos_on_frame(frame, landmarks, positions)

        else: 
            face_found = False
            positions = (0,0,0)


        return face_found, frame, positions

        

def main():

    cap = cv2.VideoCapture(0)
    ret, frame = cap.read()

    frame_shape = frame.shape
    #frame_width = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH) / 2)

    #goal = (0,0,40) # goal values (x,y,z) --> read values from yaml file
    roi = (4,145,374) # regions of interest (face_landmarks nose, left_eye, right_eye)

    detector = FaceDetector(frame_shape, roi)
    
    while cap.isOpened():

        success, frame = cap.read()
        if not success:
            print("Ignoring empty camera frame.")
            continue

        # get analysed image from FaceDetector
        face_frame = detector.draw_infos_on_img(frame)
        
        cv2.imshow('FaceDetector', face_frame)
        
        if cv2.waitKey(5) & 0xFF == 27:
            cap.release()
            cv2.destroyAllWindows()
            break
        
if __name__ == '__main__':
    main()