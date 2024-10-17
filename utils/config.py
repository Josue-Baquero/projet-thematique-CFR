import yaml

class Config:
    def __init__(self, config_file: str):
        with open(config_file, 'r') as f:
            config = yaml.safe_load(f)
        self.camera_matrix_file = config['camera_matrix_file']
        self.dist_coeffs_file = config['dist_coeffs_file']
        self.marker_length = config['marker_length']
        self.fixed_marker_ids = config['fixed_marker_ids']
        self.robot_marker_id = config['robot_marker_id']
        self.camera_url = config['camera_url']
        self.use_serial = config.get('use_serial', False)
        if self.use_serial:
            self.serial_port = config['serial_port']
            self.serial_baudrate = config['serial_baudrate']