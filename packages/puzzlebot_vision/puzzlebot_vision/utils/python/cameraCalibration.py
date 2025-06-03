import glob
import time 
import cv2 as cv2
import os
import numpy as np
# Función para obtener 10 fotos
def getPics(calibrationDir):
    cap = cv2.VideoCapture(0)  # Abre la cámara predeterminada
    cnt = 0
    if not cap.isOpened():
        print("No se pudo abrir la cámara.")
        return

    os.makedirs(calibrationDir, exist_ok=True)

    while True:
        ret, frame = cap.read()
        if not ret:
            print("No se pudo leer el frame de la cámara.")
            break

        cv2.imshow('camera', frame)
        img_name = os.path.join(calibrationDir, f'calibrationImg_{cnt}.jpg')

        # Si se presiona '1', guarda la imagen
        if cv2.waitKey(1) & 0xFF == ord('1'):
            cv2.imwrite(img_name, frame)
            print(f'Imagen {cnt+1}/10 guardada: {img_name}')
            cv2.imshow('Captured img', frame)
            cnt += 1

        if cnt >= 10:
            print("Se capturaron 10 imágenes.")
            break

    cap.release()
    cv2.destroyAllWindows()

#Función para calibrar cámara
def calibrate(showPics=True, takePic=False):
    #Se obtiene carpeta donde se guardan las imágenes usadas para calibrar cámara
    root = os.getcwd()
    calibrationDir = os.path.join(root,'calibrationImg')
    
    # Si se desea tomar fotografias, se realiza esta función
    if takePic:
        getPics(calibrationDir)

    #Se obtiene la lista de imágenes a usar para la calibración
    imgPathList = glob.glob(os.path.join(calibrationDir, '*.jpg'))

    # Variables del objeto y del mundo real.

    #Numero de filas y columnas del chessboard
    row = 10 
    col = 7

    #Filas y columnas interiores del chessboard
    nRows = row - 1
    nCols = col - 1

    #Tamaño de un cuadrado
    square_size = 2 #cm

    #Criterios para encontrar puntos
    termCriteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)

    #Puntos del mundo escalados a cm
    worldPtsCur = np.zeros((nRows*nCols, 3), np.float32)
    worldPtsCur[:,:2] = np.mgrid[0:nRows, 0:nCols].T.reshape(-1,2)
    worldPtsCur = worldPtsCur*square_size

    worldPtsList = []
    imgPtsList = []

    #Para cada imagen, se encuentran puntos en el chessboard
    for curImgPath in imgPathList:
        #Preprocesamiento de imagen
        imgBGR = cv2.imread(curImgPath)
        imgGray = cv2.cvtColor(imgBGR, cv2.COLOR_BGR2GRAY)

        #Encontrar puntos en el Chessboard
        cornersFound, cornersOrg = cv2.findChessboardCorners(imgGray, (nRows,nCols), None)

        #Si se encuentran puntos
        if cornersFound:

            #Se guardan los puntos en el mundo real
            worldPtsList.append(worldPtsCur)
            #Se obtienen puntos más exactos con la siguiente función, y se guardan
            cornersRefined = cv2.cornerSubPix(imgGray, cornersOrg, (11,11), (-1,-1), termCriteria)
            imgPtsList.append(cornersRefined)
            if showPics:
                #Se muestran los puntos de Chessboard encontrados 
                cv2.drawChessboardCorners(imgBGR, (nRows, nCols), cornersRefined, cornersFound)
                cv2.imshow('Chessboard', imgBGR)
                cv2.waitKey(500)
    
    cv2.destroyAllWindows()

    #Se obtiene la matriz de la cámara, los coeficientes de distorción, la matriz de rotación y el vector de translación
    # a partir de los puntos en el mundo real y los puntos en la imagen
    repError, camMatrix, distCoeff, rvecs, tvecs = cv2.calibrateCamera(worldPtsList, imgPtsList, imgGray.shape[::-1], None, None)
    print('Camera matrix: \n', camMatrix)
    print('reproj error (px): {:.4f}'.format(repError))
    print(f'dist coeffs: {distCoeff}')

    # Se guardan los datos de calibración en un archivo .npz
    curFolder = os.path.dirname(os.path.abspath(__file__))
    paramPath = os.path.join(curFolder, 'calibration.npz')
    np.savez(paramPath,
             repError=repError,
             camMatrix=camMatrix,
             distCoeff=distCoeff,
             rvecs=rvecs,
             tvecs=tvecs)

if __name__ == '__main__':
    #Si takePic es verdadero, se capturan 10 imagenes para calibrar la cámara y se guarda en la carpeta calibrationImg
    #Sino, se obtienen las imagenes de la carpeta calibrationImg para calibrar la cámara
    calibrate(takePic=False)
