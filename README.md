# Robot Localization with ArUco Markers

Ce projet utilise des marqueurs ArUco pour localiser un robot dans un environnement contrôlé. Il utilise une caméra pour détecter les marqueurs et estimer la pose du robot par rapport à un plateau de référence.

## Structure du projet

```
ROBOT/
│
├── main.py                 # Point d'entrée principal du programme
├── config.yaml             # Fichier de configuration
├── requirements.txt        # Liste des dépendances du projet
│
├── localization/           # Module de localisation
├── camera/                 # Utilitaires pour la caméra
├── processing/             # Modes de traitement (robot, plateau, tous)
├── utils/                  # Utilitaires généraux
├── visualization/          # Fonctions d'affichage
│
├── generators/             # Scripts pour générer les marqueurs et l'échiquier
├── images/
│   ├── markers/            # Images des marqueurs ArUco générés
│   │   ├── board_markers.png
│   │   └── robot_marker.png
│   ├── calibration/        # Images de l'échiquier pour la calibration
│   │   ├── chessboard.png  # Échiquier généré
│   │   └── *.jpg           # Photos de l'échiquier pour la calibration
│
├── camera_calibration/     # Calibration de la caméra
│   ├── camera_matrix.npy
│   └── dist_coeffs.npy
│
└── README.md               # Ce fichier
```

## Prérequis

- Python 3.7+
- Voir `requirements.txt` pour la liste complète des dépendances

## Installation

1. Clonez ce dépôt :
   ```
   git clone https://gitlab.com/projet-thematique/Robot.git
   cd Robot
   ```

2. Installez les dépendances :
   ```
   pip install -r requirements.txt
   ```

## Configuration

Modifiez le fichier `config.yaml` pour ajuster les paramètres selon vos besoins.

## Utilisation

1. Générez les marqueurs ArUco et l'échiquier de calibration :
   ```
   python generators/generate_board_markers.py
   python generators/generate_robot_marker.py
   python generators/generate_chessboard.py
   ```
   Les images générées seront sauvegardées dans le dossier `images/markers/`.

2. Imprimez les marqueurs générés (`images/markers/board_markers.png` et `images/markers/robot_marker.png`) et l'échiquier de calibration (`images/calibration/chessboard.png`).

3. Calibrez la caméra :
   - Utilisez l'échiquier imprimé pour prendre plusieurs photos sous différents angles et distances.
   - Placez ces photos (format .jpg) dans le dossier `images/calibration/`.
   - Exécutez le script de calibration :
     ```
     python camera_calibration/camera_calibration.py
     ```
   - Assurez-vous que l'échiquier est entièrement visible dans chaque image pour une calibration précise.
   - Il est recommandé d'avoir 15-20 images pour une bonne calibration.

4. Lancez le programme principal :
   ```
   python main.py --mode [robot|board|all]
   ```

## Modes de fonctionnement

- `robot` : Détecte uniquement le marqueur du robot par rapport à la caméra
- `board` : Détecte uniquement les marqueurs du plateau par rapport à la caméra
- `all` : Détecte tous les marqueurs et estime la pose relative du robot par rapport au plateau

## Contribution

Les contributions sont les bienvenues ! N'hésitez pas à ouvrir une issue ou à soumettre une pull request.

## Licence

Ce projet est sous licence MIT. Voir le fichier `LICENSE` pour plus de détails.
