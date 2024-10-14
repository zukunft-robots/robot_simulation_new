import sys
from PyQt5.QtWidgets import QApplication, QWidget, QVBoxLayout, QPushButton, QLabel, QMessageBox, QStackedWidget,QInputDialog, QFileDialog
from PyQt5.QtGui import QIcon
from PyQt5.QtCore import Qt, QProcess
import yaml
import re
import os
class GUI(QWidget):
    def __init__(self):
        super().__init__()
        self.initUI()
        self.processes = []

    def initUI(self):
        self.stacked_widget = QStackedWidget()
        main_screen = QWidget()
        main_layout = QVBoxLayout()

        self.btn_remote = QPushButton('Remote Mode', self)
        self.btn_remote.clicked.connect(self.show_remote_mode)
        main_layout.addWidget(self.btn_remote)

        self.btn_autonomous = QPushButton('Autonomous Mode', self)
        self.btn_autonomous.clicked.connect(self.show_autonomous_mode)
        main_layout.addWidget(self.btn_autonomous)

        main_screen.setLayout(main_layout)
        remote_screen = QWidget()
        remote_layout = QVBoxLayout()


        self.status_label = QLabel('Status: Ready', self)
        self.status_label.setAlignment(Qt.AlignCenter)
        remote_layout.addWidget(self.status_label)

        self.btn_spwan_remote = QPushButton('Spawn Robot', self)
        self.btn_spwan_remote.clicked.connect(self.show_robot_mode)
        remote_layout.addWidget(self.btn_spwan_remote)


        self.btn_rviz_remote = QPushButton(' Open RVIZ2', self)
        self.btn_rviz_remote.setIcon(QIcon('icons/rviz_icon.png'))
        self.btn_rviz_remote.clicked.connect(self.open_rviz2)
        remote_layout.addWidget(self.btn_rviz_remote)
        

        self.btn_teleop_remote = QPushButton(' Start Teleop', self)
        self.btn_teleop_remote.setIcon(QIcon('icons/teleop_icon.png'))
        self.btn_teleop_remote.clicked.connect(self.start_teleop)
        remote_layout.addWidget(self.btn_teleop_remote)

        self.btn_back_remote = QPushButton('Back', self)
        self.btn_back_remote.clicked.connect(self.go_back_to_main)
        remote_layout.addWidget(self.btn_back_remote)

        remote_screen.setLayout(remote_layout)

        autonomous_screen = QWidget()
        autonomous_layout = QVBoxLayout()

        self.status_label = QLabel('Status: Ready', self)
        self.status_label.setAlignment(Qt.AlignCenter)
        autonomous_layout.addWidget(self.status_label)

        # self.btn_spwan_autonomous = QPushButton('Spawn Robot', self)
        # self.btn_spwan_autonomous.clicked.connect(self.show_robot_mode)
        # remote_layout.addWidget(self.btn_spwan_autonomous)

        # self.btn_spawn_autonomous = QPushButton('Spawn Robot', self)
        # self.btn_spawn_autonomous.setIcon(QIcon('icons/robot_icon.png'))
        # self.btn_spawn_autonomous.clicked.connect(self.spawn_robot)
        # robot_layout.addWidget(self.btn_spawn_autonomous)

        self.btn_spwan_remote = QPushButton('Spawn Robot', self)
        self.btn_spwan_remote.clicked.connect(self.show_robot_autonomous_mode)
        autonomous_layout.addWidget(self.btn_spwan_remote)
        

        self.btn_rviz_autonomous = QPushButton('Open RVIZ2', self)
        self.btn_rviz_autonomous.setIcon(QIcon('icons/rviz_icon.png'))
        self.btn_rviz_autonomous.clicked.connect(self.open_rviz)
        autonomous_layout.addWidget(self.btn_rviz_autonomous)

        self.btn_teleop_autonomous = QPushButton('Start Teleop', self)
        self.btn_teleop_autonomous.setIcon(QIcon('icons/teleop_icon.png'))
        self.btn_teleop_autonomous.clicked.connect(self.start_teleop)
        autonomous_layout.addWidget(self.btn_teleop_autonomous)

        self.btn_mapsave = QPushButton('Mapping ', self)
        self.btn_mapsave.clicked.connect(self.show_mapping_mode)
        autonomous_layout.addWidget(self.btn_mapsave)

        self.btn_nav = QPushButton('Autonomous Navigation ', self)
        self.btn_nav.clicked.connect(self.show_navigation_mode)
        autonomous_layout.addWidget(self.btn_nav)

        self.btn_back_autonomous = QPushButton('Back', self)
        self.btn_back_autonomous.clicked.connect(self.go_back_to_main)
        autonomous_layout.addWidget(self.btn_back_autonomous)

        autonomous_screen.setLayout(autonomous_layout)

        mapping_screen = QWidget()
        mapping_layout = QVBoxLayout()



        self.btn_mapping = QPushButton('Start Mapping', self)
        self.btn_mapping.setIcon(QIcon('icons/map_icon.png'))
        self.btn_mapping.clicked.connect(self.start_mapping)
        mapping_layout.addWidget(self.btn_mapping)

        self.btn_mapsaving = QPushButton('Save Map', self)
        self.btn_mapsaving.setIcon(QIcon('icons/mapsave_icon.png'))
        self.btn_mapsaving.clicked.connect(self.save_map)
        mapping_layout.addWidget(self.btn_mapsaving)

        self.btn_back_autonomous = QPushButton('Back', self)
        self.btn_back_autonomous.clicked.connect(self.go_back_to_autonomous_mode)
        mapping_layout.addWidget(self.btn_back_autonomous)
        
        mapping_screen.setLayout(mapping_layout)

        navigation_screen = QWidget()
        navigation_layout = QVBoxLayout()

        self.btn_select_map = QPushButton('Select Map', self)
        self.btn_select_map.setIcon(QIcon('icons/map_icon.png'))
        self.btn_select_map.clicked.connect(self.select_map)
        navigation_layout.addWidget(self.btn_select_map)

        self.btn_navigation = QPushButton('Start Navigation', self)
        self.btn_navigation.setIcon(QIcon('icons/nav_icon.png'))
        self.btn_navigation.clicked.connect(self.start_navigation)
        navigation_layout.addWidget(self.btn_navigation)

        self.btn_back_autonomous = QPushButton('Back', self)
        self.btn_back_autonomous.clicked.connect(self.go_back_to_autonomous_mode)
        navigation_layout.addWidget(self.btn_back_autonomous)

        # self.select_xacro_button = QPushButton('Select Xacro File')
        # self.select_xacro_button.clicked.connect(self.select_xacro_file)
        # remote_layout.addWidget(self.select_xacro_button)

        navigation_screen.setLayout(navigation_layout)

        robot_screen = QWidget()
        robot_layout = QVBoxLayout()

        self.btn_select_robot = QPushButton(' Select Robot', self)
        self.btn_select_robot.setIcon(QIcon('icons/robot_icon.png'))
        self.btn_select_robot.clicked.connect(self.show_robot)
        robot_layout.addWidget(self.btn_select_robot)

        self.btn_gazebo_remote = QPushButton(' Spawn Robot', self)
        self.btn_gazebo_remote.setIcon(QIcon('icons/robot_icon.png'))
        self.btn_gazebo_remote.clicked.connect(self.spawn_robot)
        robot_layout.addWidget(self.btn_gazebo_remote)

        self.btn_back_spawn_remote = QPushButton('Back', self)
        self.btn_back_spawn_remote.clicked.connect(self.go_back_to_remote_mode)
        robot_layout.addWidget(self.btn_back_spawn_remote)

        robot_screen.setLayout(robot_layout)

        robot_screen_autonomous = QWidget()
        robot_layout_autonomous = QVBoxLayout()

        self.btn_select_robot = QPushButton(' Select Robot', self)
        self.btn_select_robot.setIcon(QIcon('icons/robot_icon.png'))
        self.btn_select_robot.clicked.connect(self.show_robot)
        robot_layout_autonomous.addWidget(self.btn_select_robot)

        self.btn_gazebo_remote = QPushButton(' Spawn Robot', self)
        self.btn_gazebo_remote.setIcon(QIcon('icons/robot_icon.png'))
        self.btn_gazebo_remote.clicked.connect(self.spawn_robot)
        robot_layout_autonomous.addWidget(self.btn_gazebo_remote)

        self.btn_back_spawn_autonomous = QPushButton('Back', self)
        self.btn_back_spawn_autonomous.clicked.connect(self.go_back_to_autonomous_mode)
        robot_layout_autonomous.addWidget(self.btn_back_spawn_autonomous)

        robot_screen_autonomous.setLayout(robot_layout_autonomous)


        self.stacked_widget.addWidget(main_screen)
        self.stacked_widget.addWidget(remote_screen)
        self.stacked_widget.addWidget(autonomous_screen)
        self.stacked_widget.addWidget(mapping_screen)
        self.stacked_widget.addWidget(navigation_screen)
        self.stacked_widget.addWidget(robot_screen)
        self.stacked_widget.addWidget(robot_screen_autonomous)

        main_layout = QVBoxLayout()
        main_layout.addWidget(self.stacked_widget)
        self.setLayout(main_layout)

        self.setWindowTitle('Robot Simulation Control GUI')
        self.setWindowIcon(QIcon('icons/main_icon.png'))
        self.show()

    def spawn_robot(self):

        # self.run_command(['ros2', 'launch', 'sim_robot', 'gazebo.launch.py','world:=./src/robot_simulation_new/sim_robot/world/maze.world'], 'Spawning Robot...')
        world_folder = './src/robot_simulation_new/sim_robot/world'
        world_file_path, _ = QFileDialog.getOpenFileName(self, "Select World File", world_folder, "World Files (*.world);;All Files (*)")
        if world_file_path:
            self.run_command(['ros2', 'launch', 'sim_robot', 'gazebo.launch.py', f'world:={world_file_path}'], 'Spawning Robot...')
        else:
            QMessageBox.warning(self, 'Error', 'No world file selected.')

    def open_rviz(self):
        self.run_command(['ros2', 'run', 'rviz2', 'rviz2','-d','./src/robot_simulation_new/sim_robot/config/map.rviz'], 'Opening RVIZ2...')

    def start_teleop(self):
        self.run_command(['gnome-terminal', '--', 'ros2', 'run', 'teleop_twist_keyboard', 'teleop_twist_keyboard'], 'Starting Teleop...')

    def open_rviz2(self):
        self.run_command(['ros2', 'run', 'rviz2', 'rviz2','-d','./src/robot_simulation_new/sim_robot/config/remote.rviz'], 'Opening RVIZ2...')


    def start_mapping(self):
        self.run_command(['ros2', 'launch', 'sim_robot', 'mapping_launch.py'], 'Starting Mapping...')

    def start_navigation(self):
        self.run_command(['ros2','launch','sim_robot','online_async_launch.py'], 'Starting Localization')
        self.run_command(['ros2', 'launch', 'sim_robot', 'navigation_launch.py'], 'Starting Navigation...')

    # def save_map(self):
    #     self.run_command(['ros2', 'run', 'nav2_map_server', 'map_saver_cli','-f','src/robot_simulation_new/maps/my_map'], 'Saving Map...')
    #     self.run_command(['ros2', 'service', 'call', '/slam_toolbox/serialize_map', 'slam_toolbox/srv/SerializePoseGraph', "{filename: 'src/robot_simulation_new/maps/my_map'}"],'Saving..')
    
    def save_map(self):
        filename, ok = QInputDialog.getText(self, 'Save Map', 'Enter the map filename:')
        if ok and filename:
            map_path = f'src/robot_simulation_new/maps/{filename}'

            self.run_command(['ros2', 'run', 'nav2_map_server', 'map_saver_cli', '-f', map_path], 'Saving Map...')
            self.run_command(['ros2', 'service', 'call', '/slam_toolbox/serialize_map', 'slam_toolbox/srv/SerializePoseGraph', f"{{filename: '{map_path}'}}"], 'Saving...')
        else:
            self.status_label.setText('Status: Map saving cancelled')

    def select_map(self):
        options = QFileDialog.Options()
        map_folder = './src/robot_simulation_new/maps'
        file_name, _ = QFileDialog.getOpenFileName(self, "Select Map File", map_folder, "YAML Files (*.yaml);;All Files (*)", options=options)
        if file_name:
            self.map_file_path = file_name
            self.update_map_file_name_in_yaml(self.map_file_path)
            self.status_label.setText(f'Selected Map: {file_name}')
    def update_map_file_name_in_yaml(self, map_file_path):
        #yaml_file = '/home/keerthi/sim_test/src/robot_simulation_new/sim_robot/config/mapper_params_online_async.yaml'
        yaml_file = './install/sim_robot/share/sim_robot/config/mapper_params_online_async.yaml'
        
        with open(yaml_file, 'r') as file:
            yaml_data = yaml.safe_load(file)

        map_file_path = re.sub(r'\.yaml$', '', map_file_path)

        map_file_path = os.path.relpath(map_file_path, os.getcwd())
        yaml_data['slam_toolbox']['ros__parameters']['map_file_name'] = map_file_path

        with open(yaml_file, 'w') as file:
            yaml.dump(yaml_data, file)

        print(f'Updated map_file_name in YAML to: {map_file_path}')


    def show_robot(self):
        self.xacro_file_path = None
        #self.launch_file_path = './install/sim_robot/share/sim_robot/launch/rsp.launch.py'
        options = QFileDialog.Options()
        xacro_folder = './src/robot_simulation_new/sim_robot/description'  
        file_name, _ = QFileDialog.getOpenFileName(self, "Select Xacro File", xacro_folder, "Xacro Files (*.urdf.xacro);;All Files (*)", options=options)
        if file_name:
            self.xacro_file_path = file_name
            self.status_label.setText(f'Selected Xacro: {file_name}')
            self.update_launch_file()  
        else:
            self.status_label.setText('No xacro file selected')

    def update_launch_file(self):
        if self.xacro_file_path is None:
            QMessageBox.warning(self, 'Error', 'No xacro file selected.')
            return

       
        new_xacro_file_name = os.path.basename(self.xacro_file_path)

       
        launch_file_path = './install/sim_robot/share/sim_robot/launch/rsp.launch.py'  

        try:
         
            with open(launch_file_path, 'r') as file:
                launch_content = file.readlines()

        
            updated_content = []
            for line in launch_content:
                if 'default_value=os.path.join(' in line and '.xacro' in line:
                
                    line_parts = line.split(',')
                    for i, part in enumerate(line_parts):
                        if '.xacro' in part:
                            line_parts[i] = f" '{new_xacro_file_name}')"
                    updated_line = ','.join(line_parts)
                    updated_content.append(updated_line + '\n')
                    self.status_label.setText(f'Updated xacro file to: {new_xacro_file_name}')
                else:
                    updated_content.append(line)

          
            with open(launch_file_path, 'w') as file:
                file.writelines(updated_content)

            QMessageBox.information(self, 'Success', 'Launch file updated successfully.')

        except Exception as e:
            QMessageBox.critical(self, 'Error', f'Failed to update launch file: {str(e)}')

    def run_command(self, command, status_message):
        process = QProcess(self)
        process.setProgram(command[0])
        process.setArguments(command[1:])
        process.readyReadStandardOutput.connect(self.handle_stdout)
        process.readyReadStandardError.connect(self.handle_stderr)
        process.finished.connect(self.process_finished)
        process.start()

        self.processes.append(process)
        self.status_label.setText(f'Status: {status_message}')

    def show_remote_mode(self):
        self.stacked_widget.setCurrentIndex(1)  

    def show_autonomous_mode(self):
        self.stacked_widget.setCurrentIndex(2)  

    def show_mapping_mode(self):
        self.stacked_widget.setCurrentIndex(3)

    def show_navigation_mode(self):
        self.stacked_widget.setCurrentIndex(4)

    def show_robot_mode(self):
        self.stacked_widget.setCurrentIndex(5)

    def show_robot_autonomous_mode(self):
        self.stacked_widget.setCurrentIndex(6)

    def go_back_to_main(self):
        self.stacked_widget.setCurrentIndex(0)
    
    def go_back_to_autonomous_mode(self):
        self.stacked_widget.setCurrentIndex(2)

    def go_back_to_remote_mode(self):
        self.stacked_widget.setCurrentIndex(1)


    def handle_stdout(self):
        process = self.sender()
        output = process.readAllStandardOutput().data().decode()
        print(f"STDOUT: {output}")

    def handle_stderr(self):
        process = self.sender()
        error_output = process.readAllStandardError().data().decode()
        print(f"STDERR: {error_output}")

    def process_finished(self):
        process = self.sender()
        self.status_label.setText(f'Status: Process finished with exit code {process.exitCode()}')
        print(f"Process finished with exit code: {process.exitCode()}")


    def closeEvent(self, event):
        for process in self.processes:
            if process.state() != QProcess.NotRunning:
                print(f"Terminating process: {process.program()}")
                process.terminate()
                if not process.waitForFinished(5000):
                    print(f"Killing process: {process.program()}")
                    process.kill()
        event.accept()

def main():
    app = QApplication(sys.argv)
    with open('./src/robot_simulation_new/sim_robot/launch/style.qss', 'r') as file:
        app.setStyleSheet(file.read())
    gui = GUI()
    sys.exit(app.exec_())

if __name__ == '__main__':
    main()
