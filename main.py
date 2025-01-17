import argparse
from localization.aruco_localization import ArucoLocalization
from utils.config import Config

def main():
    # Configuration du parser d'arguments
    parser = argparse.ArgumentParser(description="ArUco marker detection")
    # Ajout de l'argument --mode avec trois options possibles:
    # - robot : détecte uniquement le marqueur du robot (ID 36)
    # - board : détecte uniquement les marqueurs du plateau (ID 20,21,22,23)
    # - all : détecte tous les marqueurs (par défaut)
    parser.add_argument("--mode", choices=["robot", "board", "all"], default="all",
                        help="Detection mode: robot (only robot marker), board (only board markers), or all (default)")
    
    # Récupération des arguments passés en ligne de commande
    args = parser.parse_args()
    
    # Chargement de la configuration depuis config.yaml
    config = Config('config.yaml')
    
    # Initialisation du système de localisation avec la configuration
    localization = ArucoLocalization(config)
    
    # Lancement de la détection dans le mode spécifié
    localization.run(args.mode)

if __name__ == "__main__":
    main()