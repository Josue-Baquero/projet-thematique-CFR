# Système de Localisation par ArUco

Ce projet implémente un système de localisation robotique utilisant des marqueurs ArUco. Il permet de détecter et calculer la position et l'orientation d'un robot mobile équipé d'un marqueur ArUco, relativement à un plateau de référence défini par quatre marqueurs fixes.

## Structure du Projet

```
.
├── .git/                      # Dossier de gestion de version Git
├── .venv/                     # Environnement virtuel Python
│
├── camera/                    # Gestion de la caméra IP
│   ├── __init__.py           # Expose get_ip_webcam_frame
│   └── camera_utils.py       # Utilitaires pour la caméra IP (capture d'images)
│
├── camera_calibration/        # Fichiers de calibration de la caméra
│   ├── camera_matrix.npy     # Matrice de calibration
│   └── dist_coeffs.npy       # Coefficients de distorsion
│
├── generators/                # Scripts de génération de marqueurs
│   ├── generate_board_markers.py    # Génère marqueurs ArUco du plateau (IDs 20-23)
│   ├── generate_chessboard.py       # Génère échiquier pour calibration
│   └── generate_robot_marker.py     # Génère marqueur ArUco du robot (ID 36)
│
├── images/                    # Stockage des images
│   ├── calibration/          # Images pour la calibration de la caméra (.jpg)
│   └── markers/              # Marqueurs ArUco générés
│       ├── board_markers.png # Marqueurs du plateau
│       ├── chessboard.png    # Pattern d'échiquier
│       └── robot_marker.png  # Marqueur du robot
│
├── localization/             # Système de localisation principal
│   ├── __init__.py          # Expose les classes principales
│   ├── aruco_localization.py # Classe principale de localisation ArUco
│   ├── heading.py           # Calcul de l'orientation (heading)
│   └── robot_pose.py        # Classe pour stocker position/orientation
│
├── processing/               # Traitement des images selon les modes
│   ├── __init__.py          # Import des différents modes
│   ├── all_mode.py          # Détection robot + plateau
│   ├── board_mode.py        # Détection plateau uniquement
│   └── robot_mode.py        # Détection robot uniquement
│
├── utils/                    # Utilitaires généraux
│   ├── __init__.py          # Expose la classe Config
│   ├── config.py            # Gestion de la configuration
│   └── markers.py           # Configuration des marqueurs ArUco
│
├── visualization/            # Affichage et visualisation
│   ├── __init__.py          # Expose les fonctions d'affichage
│   └── display.py           # Fonctions d'affichage et GUI
│
├── config.yaml              # Configuration du système
├── main.py                  # Point d'entrée principal
├── README.md                # Ce fichier
└── requirements.txt         # Dépendances Python requises
```

## Installation et Configuration

### 1. Cloner le Projet

```bash
git clone https://github.com/Josue-Baquero/projet-thematique-CFR
cd projet-thematique-CFR
```

### 2. Configuration de l'Environnement

1. Installer VSCode si ce n'est pas déjà fait
2. Ouvrir le projet dans VSCode
3. Créer un environnement virtuel Python :
```bash
python -m venv .venv
```
4. Activer l'environnement virtuel :
   - Windows : `.venv\Scripts\activate`
   - Linux/MacOS : `source .venv/bin/activate`
5. Installer les dépendances :
```bash
pip install -r requirements.txt
```

### 3. Calibration de la Caméra

#### 3.1 Génération de l'Échiquier
1. Modifier les paramètres dans `generators/generate_chessboard.py` :
```python
square_size = 100  # Taille des carrés en pixels
width = 10         # Nombre de carrés en largeur
height = 7         # Nombre de carrés en hauteur
```
2. Exécuter le script :
```bash
python generators/generate_chessboard.py
```
3. Imprimer l'échiquier généré dans `images/markers/chessboard.png`

#### 3.2 Prise de Photos pour la Calibration

1. Créer un dossier `images/calibration/`
2. Prendre au moins 20 photos de l'échiquier sous différents angles
   - Varier les orientations (portrait/paysage)
   - Varier les distances
   - Éviter les reflets
   - Assurer un bon éclairage uniforme
   - S'assurer que l'échiquier est entièrement visible
   - Placer l'échiquier dans différentes zones de l'image

#### 3.3 Processus de Calibration

1. Ajuster les paramètres de l'échiquier dans `camera_calibration.py` :
```python
CHECKERBOARD = (9,6)  # Nombre de coins internes (largeur-1, hauteur-1)
```

2. Lancer la calibration :
```bash
python camera_calibration.py
```

3. Pour réduire l'erreur de reprojection :
   - Supprimer les images qui donnent une erreur élevée
   - Refaire des photos avec plus de précision
   - Vérifier la planéité de l'échiquier imprimé

### 4. Configuration du Système

#### 4.1 Configuration de la Caméra IP

1. Installer "IP Webcam" sur votre smartphone Android :
   - Disponible sur [Google Play](https://play.google.com/store/apps/details?id=com.pas.webcam)
   - Ou utiliser une application équivalente pour iOS

2. Configurer l'application :
   - Connecter le téléphone au même réseau WiFi que l'ordinateur
   - Lancer l'application
   - Noter l'URL affichée (ex: http://192.168.1.100:8080)

3. Modifier `config.yaml` :
```yaml
camera_url: "http://192.168.1.100:8080/shot.jpg"  # Remplacer par votre URL
```

#### 4.2 Configuration du Plateau (si nécessaire)

Si vous n'utilisez pas le plateau de la Coupe de France, modifiez les dimensions dans `utils/markers.py` :
```python
def setup_board(self, use_real_board):
    if use_real_board:
        objPoints = np.array([
            # Modifier ces coordonnées selon votre plateau
            [[-950, 450, 0], [-850, 450, 0], [-850, 350, 0], [-950, 350, 0]],  # Marker 20
            [[850, 450, 0], [950, 450, 0], [950, 350, 0], [850, 350, 0]],      # Marker 21
            [[-950, -350, 0], [-850, -350, 0], [-850, -450, 0], [-950, -450, 0]], # Marker 22
            [[850, -350, 0], [950, -350, 0], [950, -450, 0], [850, -450, 0]]     # Marker 23
        ], dtype=np.float32)
```

### 5. Exécution du Programme

Le programme peut être exécuté dans trois modes différents :

```bash
# Détection complète (robot + plateau)
python main.py --mode all

# Détection du robot uniquement
python main.py --mode robot

# Détection du plateau uniquement
python main.py --mode board
```

## Ressources pour Comprendre

### Calcul des Poses Relatives
OpenCV utilise l'algorithme solvePnP pour calculer la position et l'orientation relatives entre la caméra et les marqueurs. Pour comprendre en détail :
- [Documentation solvePnP](https://docs.opencv.org/4.x/d5/d1f/calib3d_solvePnP.html)

### Calibration de Caméra
La calibration de la caméra est essentielle pour des mesures précises. Pour approfondir :
- [Tutorial Calibration OpenCV](https://docs.opencv.org/4.x/dc/dbb/tutorial_py_calibration.html)
