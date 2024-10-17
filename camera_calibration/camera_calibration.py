import numpy as np
import cv2
import glob

def calibrate_camera():
    CHECKERBOARD = (9,6)
    criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)

    objp = np.zeros((CHECKERBOARD[0] * CHECKERBOARD[1], 3), np.float32)
    objp[:,:2] = np.mgrid[0:CHECKERBOARD[0], 0:CHECKERBOARD[1]].T.reshape(-1,2)

    objpoints = []
    imgpoints = []

    images = glob.glob('images/calibration/*.jpg')
    
    if len(images) == 0:
        print("Aucune image trouvée dans le dossier 'images/calibration'. Assurez-vous d'avoir des images .jpg dans ce dossier.")
        return
    
    print(f"Nombre total d'images trouvées : {len(images)}")
    successful_calibrations = 0

    for fname in images:
        img = cv2.imread(fname)
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

        ret, corners = cv2.findChessboardCorners(gray, CHECKERBOARD, None)

        if ret == True:
            objpoints.append(objp)
            corners2 = cv2.cornerSubPix(gray, corners, (11,11), (-1,-1), criteria)
            imgpoints.append(corners2)

            img = cv2.drawChessboardCorners(img, CHECKERBOARD, corners2, ret)
            
            cv2.imshow('Calibration Image', img)
            cv2.waitKey(500)

            print(f"Échiquier détecté avec succès dans {fname}. {len(corners2)} coins trouvés.")
            successful_calibrations += 1
        else:
            print(f"Échec de la détection de l'échiquier dans {fname}.")

    cv2.destroyAllWindows()

    print(f"\nRésumé : {successful_calibrations}/{len(images)} images utilisées avec succès pour la calibration.")

    if len(objpoints) == 0:
        print("Aucun échiquier détecté dans les images. Assurez-vous que les images contiennent un échiquier visible.")
        return

    print("\nDébut de la calibration...")
    ret, mtx, dist, rvecs, tvecs = cv2.calibrateCamera(objpoints, imgpoints, gray.shape[::-1], None, None)

    if not ret:
        print("La calibration a échoué. Vérifiez vos images et les paramètres de l'échiquier.")
        return

    np.save('camera_calibration/camera_matrix.npy', mtx)
    np.save('camera_calibration/dist_coeffs.npy', dist)

    print("\nCalibration terminée avec succès.")
    print("Matrice de la caméra sauvegardée dans 'camera_calibration/camera_matrix.npy'")
    print("Coefficients de distorsion sauvegardés dans 'camera_calibration/dist_coeffs.npy'")

    print("\nMatrice de la caméra:")
    print(mtx)
    print("\nCoefficients de distorsion:")
    print(dist)

    mean_error = 0
    for i in range(len(objpoints)):
        imgpoints2, _ = cv2.projectPoints(objpoints[i], rvecs[i], tvecs[i], mtx, dist)
        error = cv2.norm(imgpoints[i], imgpoints2, cv2.NORM_L2)/len(imgpoints2)
        mean_error += error

    print(f"\nErreur de reprojection totale : {mean_error/len(objpoints)}")

if __name__ == "__main__":
    calibrate_camera()