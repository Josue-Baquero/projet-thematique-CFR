import numpy as np
import cv2
import glob

# Définir la taille de l'échiquier (nombre de coins internes)
CHECKERBOARD = (7,4)
criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)

# Préparer les points objets, comme (0,0,0), (1,0,0), (2,0,0) ....,(8,5,0)
objp = np.zeros((CHECKERBOARD[0] * CHECKERBOARD[1], 3), np.float32)
objp[:,:2] = np.mgrid[0:CHECKERBOARD[0], 0:CHECKERBOARD[1]].T.reshape(-1,2)

# Arrays pour stocker les points objets et images de toutes les images
objpoints = [] # points 3d dans l'espace réel
imgpoints = [] # points 2d dans le plan image

# Charger les images de l'échiquier
images = glob.glob('chessphotos/*.jpg')

if len(images) == 0:
    print("Aucune image trouvée dans le dossier 'chessphotos'. Assurez-vous d'avoir des images .jpg dans ce dossier.")
    exit()

print(f"Nombre total d'images trouvées : {len(images)}")

successful_calibrations = 0
for i, fname in enumerate(images):
    print(f"\nTraitement de l'image {i+1}/{len(images)} : {fname}")
    img = cv2.imread(fname)
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    
    # Trouver les coins de l'échiquier
    ret, corners = cv2.findChessboardCorners(gray, CHECKERBOARD, None)
    
    if ret == True:
        objpoints.append(objp)
        corners2 = cv2.cornerSubPix(gray, corners, (11,11), (-1,-1), criteria)
        imgpoints.append(corners2)
        
        # Dessiner et afficher les coins
        img = cv2.drawChessboardCorners(img, CHECKERBOARD, corners2, ret)
        cv2.imshow('img', img)
        cv2.waitKey(500)
        
        print(f"  Échiquier détecté avec succès. {len(corners2)} coins trouvés.")
        successful_calibrations += 1
    else:
        print(f"  Échec de la détection de l'échiquier dans cette image.")

cv2.destroyAllWindows()

print(f"\nRésumé : {successful_calibrations}/{len(images)} images utilisées avec succès pour la calibration.")

if len(objpoints) == 0:
    print("Aucun échiquier détecté dans les images. Assurez-vous que les images contiennent un échiquier visible.")
    exit()

print("\nDébut de la calibration...")
# Calibration
ret, mtx, dist, rvecs, tvecs = cv2.calibrateCamera(objpoints, imgpoints, gray.shape[::-1], None, None)

if not ret:
    print("La calibration a échoué. Vérifiez vos images et les paramètres de l'échiquier.")
    exit()

# Sauvegarder les résultats
np.save('camera_matrix.npy', mtx)
np.save('dist_coeffs.npy', dist)

print("\nCalibration terminée avec succès.")
print("Matrice de la caméra sauvegardée dans 'camera_matrix.npy'")
print("Coefficients de distorsion sauvegardés dans 'dist_coeffs.npy'")

# Afficher les résultats
print("\nMatrice de la caméra:")
print(mtx)
print("\nCoefficients de distorsion:")
print(dist)

# Calculer l'erreur de reprojection
mean_error = 0
for i in range(len(objpoints)):
    imgpoints2, _ = cv2.projectPoints(objpoints[i], rvecs[i], tvecs[i], mtx, dist)
    error = cv2.norm(imgpoints[i], imgpoints2, cv2.NORM_L2)/len(imgpoints2)
    mean_error += error

print(f"\nErreur de reprojection totale : {mean_error/len(objpoints)}")