import glob
import cv2
import os
import numpy as np

number_sq_x = 10
number_sq_y = 10
nX = number_sq_x - 1
nY = number_sq_y - 1

square_size = 0.025 #meters
criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)

obj_points_3d = np.zeros((nX * nY, 3), np.float32)
obj_points_3d[:,:2] = np.mgrid[0:nY, 0:nX].T.reshape(-1, 2)
obj_points_3d = obj_points_3d * square_size

obj_points_list = []
img_points_list = []

def getPics(calibrationDir):
    
    cnt = 0
    cap = cv2.VideoCapture(1)

    while True:

        frame = cap.read()
        if cnt >= 10:
            break

        cv2.imshow('camera', frame)
        img_name = os.path_join(calibrationDir, f'calibrationImg_{cnt}.jpg')
        
        if cv2.waitKey(1) & 0xFF == ord('1'):
            cv2.imwrite(img_name, frame)
            cv2.imshow('Captured img', frame)
            cnt += 1

    cap.release()
    cv2.destroyAllWindows()


def calibrate(showPics=True, takePic=False):

    root = os.getcwd()
    calibrationDir = os.path.join(root, 'calibrationImg')

    if takePic:
        getPics(calibrationDir)

    imgPathList = glob.glob(os.path.join(calibrationDir, 'calibrationImg_*.jpg'))

    for curImgPath in imgPathList:

        img_bgr = cv2.imread(curImgPath)
        img_gray = cv2.cvtColor(img_bgr, cv2.COLOR_BGR2GRAY)

        cornersFound, cornersOrg = cv2.findChessboardCorners(img_gray, (nY, nX), None)

        if cornersFound:
            obj_points_list.append(obj_points_3d)

            cornersRefined = cv2.cornerSubPix(img_gray, cornersOrg, (11, 11), (-1, -1), criteria)
            img_points_list.append(cornersRefined)

            if showPics:
                cv2.drawChessboardCorners(img_bgr, (nY, nX), cornersRefined, cornersFound)
                cv2.imshow('Chessboard', img_bgr)
                cv2.waitKey(500)


    cv2.destroyAllWindows()

    repError, camMatrix, distCoeff, rvecs, tvecs = cv2.calibrateCamera(obj_points_list, img_points_list, 
                                                                       img_gray.shape[::-1], None, None)
    print('Camera matrix: \n', camMatrix)
    print('Reproj error (px): {:.4f}'.format(repError))

    curFolder = os.path.dirname(os.path.abspath(__file__))
    paramPath = os.path.join(curFolder, 'calibration.npz')
    np.savez(paramPath, camMatrix=camMatrix, distCoeff=distCoeff)

if __name__ == '__main__':
    calibrate(takePic=True)