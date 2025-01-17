import yaml

class Config:
    """
   Classe de gestion de la configuration du système de localisation.
   
   Cette classe a pour objectif de :
   - Charger et valider le fichier de configuration YAML

    """

    def __init__(self, config_file: str):
        with open(config_file, 'r') as f:
            config = yaml.safe_load(f)
        self.camera_matrix_file = config['camera_matrix_file']
        self.dist_coeffs_file = config['dist_coeffs_file']
        self.marker_length = config['marker_length']
        self.fixed_marker_ids = config['fixed_marker_ids']
        self.robot_marker_id = config['robot_marker_id']
        self.camera_url = config['camera_url']
        self.axis_length = config['axis_length']
        self.use_real_board = config.get('use_real_board', True)