#!/usr/bin/env python3

import os
from pydoc_data.topics import topics
import rospy, rospkg
import roslaunch

from qt_gui.plugin import Plugin
from python_qt_binding import loadUi
from python_qt_binding.QtWidgets import QWidget
from rospy.core import loginfo

from std_msgs.msg import String , Bool
from sensor_msgs.msg import JointState


class AGVSimilationPlugin(Plugin):

    def __init__(self, context):
        super(AGVSimilationPlugin, self).__init__(context)
        self.setObjectName('SimilationPlugin')

        # Process standalone plugin command-line arguments
        from argparse import ArgumentParser
        parser = ArgumentParser()
        # Add argument(s) to the parser.
        parser.add_argument("-q", "--quiet", action="store_true",
                      dest="quiet",
                      help="Put plugin in silent mode")
        args, unknowns = parser.parse_known_args(context.argv())
        # if not args.quiet:
        #     print('arguments: ', args)
        #     print('unknowns: ', unknowns)

        # Create QWidget
        self._widget = QWidget()
        # Get path to UI file which should be in the "resource" folder of this package
        ui_file = os.path.join(rospkg.RosPack().get_path('agv_similation_ui'), 'resource', 'ITUAGVPlugin.ui')
        # Extend the widget with all attributes and children from UI file
        loadUi(ui_file, self._widget)
        # Give QObjects reasonable names
        self._widget.setObjectName('SimilationPluginUi')
       
        if context.serial_number() > 1:
            self._widget.setWindowTitle(self._widget.windowTitle() + (' (%d)' % context.serial_number()))
        # Add widget to the user interface
        context.add_widget(self._widget)

        rospy.logwarn("UI is initializing.")



        
        
        ####INIT PARAMETERS
        self.oppened_world = None
        self.agv_spanned = False 
        self.slam_started = False
        self.uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
        self.worlds = ['5x5','20x20','50x50']
        self.start_path = rospkg.RosPack().get_path('itu_agv_gazebo')+'/launch/start_factory.launch'
        self.spawn_path = rospkg.RosPack().get_path('itu_agv_gazebo')+'/launch/spawn_itu_agv_moduler.launch'
        self.rviz_path = rospkg.RosPack().get_path('itu_agv_gazebo')+'/launch/rviz_itu_agv.launch'
        self.slam_path = rospkg.RosPack().get_path('itu_agv_slam')+'/launch/itu_agv_slam.launch'
        self.bag_record_path = rospkg.RosPack().get_path('itu_agv_gazebo')+ '/launch/record_data.launch'
        self.agv_spawn_config= rospy.get_param('agv_config')
        self.agv_1_sensor = [0,0,0,0]
        self.agv_2_sensor = [0,0,0,0]
        self.agv_3_sensor = [0,0,0,0]
        self.bag_topics = []



        ##UI Connections
        for name in self.agv_spawn_config['sensor_location']:
            self._widget.comboBox_sensor_loc.addItem(name)
            self._widget.comboBox_sensor_loc_2.addItem(name)
            self._widget.comboBox_sensor_loc_4.addItem(name)

        #SLAM BOX
        self._widget.comboBox_slam.addItem("gmapping")
        self._widget.pushButton_start_slam.clicked.connect(self.slam_button_cb)
        self._widget.pushButton_save_map.clicked.connect(self.save_map_button_cb)

            

        self._widget.W5X5pushButton.clicked.connect(self.w5x5button_cb)
        self._widget.W20X20pushButton.clicked.connect(self.w20x20button_cb)
        self._widget.W50X50pushButton.clicked.connect(self.w50x50button_cb)

        self._widget.checkBox_Camera.stateChanged.connect(self.check_box_camera_cb)
        self._widget.checkBox_IMU.stateChanged.connect(self.check_box_IMU_cb)
        self._widget.checkBox_Lidars.stateChanged.connect(self.check_box_Lidars_cb)
        self._widget.checkBox_Veledoyne.stateChanged.connect(self.check_box_Velodyne_cb)
        self._widget.pushButton_spawn_agv.clicked.connect(self.spawn_agv_button_cb)

        self._widget.checkBox_Camera_2.stateChanged.connect(self.check_box_camera_2_cb)
        self._widget.checkBox_IMU_2.stateChanged.connect(self.check_box_IMU_2_cb)
        self._widget.checkBox_Lidars_2.stateChanged.connect(self.check_box_Lidars_2_cb)
        self._widget.checkBox_Veledoyne_2.stateChanged.connect(self.check_box_Velodyne_2_cb)
        self._widget.pushButton_spawn_agv_2.clicked.connect(self.spawn_agv_button_2_cb)

        self._widget.checkBox_Camera_4.stateChanged.connect(self.check_box_camera_3_cb)
        self._widget.checkBox_IMU_4.stateChanged.connect(self.check_box_IMU_3_cb)
        self._widget.checkBox_Lidars_4.stateChanged.connect(self.check_box_Lidars_3_cb)
        self._widget.checkBox_Veledoyne_4.stateChanged.connect(self.check_box_Velodyne_3_cb)
        self._widget.pushButton_spawn_agv_4.clicked.connect(self.spawn_agv_button_3_cb)

        self._widget.pushButton_rviz.clicked.connect(self.rviz_button_cb)

        self._widget.pushButton_get_topics.clicked.connect(self.get_topic_cb)
        self._widget.pushButton_add_to_bag.clicked.connect(self.add_to_bag_cb)
        self._widget.pushButton_reset_bag_list.clicked.connect(self.reset_bag_list_cb)
        self._widget.pushButton_record_bag.clicked.connect(self.record_bag_cb)
    def launch(self,cli_args):     
        parent = roslaunch.parent.ROSLaunchParent(self.uuid, cli_args)
        parent.start()


    def start_world(self,world_name):
        
        roslaunch_arg = 'world_name:='+world_name
        cli_arg = [(self.start_path,roslaunch_arg)]    
        self.launch(cli_arg)

    




    # UI Functions
    def w5x5button_cb(self):
        rospy,loginfo('Starting 5X5 Factory World')
        self.oppened_world = self.worlds[0]
        self.start_world(self.worlds[0])

    def w20x20button_cb(self):
        rospy,loginfo('Starting 20X20 Factory World')
        self.oppened_world = self.worlds[1]
        self.start_world(self.worlds[1])

    def w50x50button_cb(self):
        rospy,loginfo('Starting 20X20 Factory World')
        self.oppened_world = self.worlds[2]
        self.start_world(self.worlds[2])

    def rviz_button_cb(self):
        self.launch([self.rviz_path])

    ##AGV1 SPAWN
    def check_box_camera_cb(self,val):
        if val :
            self.agv_1_sensor[0] = 1
        else :
            self.agv_1_sensor[0] = 0

    def check_box_IMU_cb(self,val):
        if val :
            self.agv_1_sensor[1] = 1
        else :
            self.agv_1_sensor[1] = 0

    def check_box_Lidars_cb(self,val):
        if val :
            self.agv_1_sensor[2] = 1
        else : 
            self.agv_1_sensor[2] = 0

    def check_box_Velodyne_cb(self,val):
        if val :
            self.agv_1_sensor[3] = 1
        else :
            self.agv_1_sensor[3] = 0


    def spawn_agv_button_cb(self):

        if self.oppened_world == None :
            self._widget.info_text.setText('To Spawn AGV Please Start Simulation First')
            return

        if self.agv_1_sensor[0] or self.agv_1_sensor[1] or self.agv_1_sensor[2] or self.agv_1_sensor[3]:
            
            sensor_location = self._widget.comboBox_sensor_loc.currentText()
            robot_tf_prefix = list(self.agv_spawn_config['tf_prefix'].keys())[0]
            sensors= ''
            for i  in range(len(self.agv_1_sensor)):
                if self.agv_1_sensor[i]:
                    if sensors == '' :sensors=sensors+self.agv_spawn_config['sensors'][i]
                    else : sensors=sensors+','+self.agv_spawn_config['sensors'][i]

            params = self.agv_spawn_config['tf_prefix'][robot_tf_prefix]



            roslaunch_arg = ['tf_prefix:='+robot_tf_prefix, 
                             'robot:='+params['robot'],
                             'location:='+sensor_location,
                             'on:='+sensors,
                             'sensors:='+params['base_link'],
                             'robot_name:='+params['robot_name'],
                             'x:='+params[self.oppened_world][0],
                             'y:='+params[self.oppened_world][1],
                             'z:='+params[self.oppened_world][2],
                             'roll:='+params[self.oppened_world][3],
                             'pitch:='+params[self.oppened_world][4],
                             'yaw:='+params[self.oppened_world][5]]

            roslauch_file = [(self.spawn_path,roslaunch_arg)]

            self.agv_spanned = True
            self.launch(roslauch_file)
        else:
            self._widget.info_text.setText('Please pick at least one sensor')







##AGV2 SPAWN
    def check_box_camera_2_cb(self,val):
        if val :
            self.agv_2_sensor[0] = 1
        else:
            self.agv_2_sensor[0] = 0
            
    def check_box_IMU_2_cb(self,val):
        if val :
            self.agv_2_sensor[1] = 1
        else:
            self.agv_2_sensor[1] = 1

    def check_box_Lidars_2_cb(self,val):
        if val :
            self.agv_2_sensor[2] = 1
        else:
            self.agv_2_sensor[2] = 0

    def check_box_Velodyne_2_cb(self,val):
        if val :
            self.agv_2_sensor[3] = 1
        else:
            self.agv_2_sensor[3] = 0

    def spawn_agv_button_2_cb(self):
        
        if self.oppened_world == None :
            self._widget.info_text.setText('To Spawn AGV Please Start Simulation First')
            return

        if self.oppened_world == '5x5': 
            self._widget.info_text.setText('AGV2 cannot open at 5x5 world')
            return


        if self.agv_2_sensor[0] or self.agv_2_sensor[1] or self.agv_2_sensor[2] or self.agv_2_sensor[3]:
            
            sensor_location = self._widget.comboBox_sensor_loc_2.currentText()
            robot_tf_prefix = list(self.agv_spawn_config['tf_prefix'].keys())[1]
            sensors= ''
            for i  in range(len(self.agv_2_sensor)):
                if self.agv_2_sensor[i]:
                    if sensors == '' :sensors=sensors+self.agv_spawn_config['sensors'][i]
                    else : sensors=sensors+','+self.agv_spawn_config['sensors'][i]

            params = self.agv_spawn_config['tf_prefix'][robot_tf_prefix]



            roslaunch_arg = ['tf_prefix:='+robot_tf_prefix, 
                             'robot:='+params['robot'],
                             'location:='+sensor_location,
                             'on:='+sensors,
                             'sensors:='+params['base_link'],
                             'robot_name:='+params['robot_name'],
                             'x:='+params[self.oppened_world][0],
                             'y:='+params[self.oppened_world][1],
                             'z:='+params[self.oppened_world][2],
                             'roll:='+params[self.oppened_world][3],
                             'pitch:='+params[self.oppened_world][4],
                             'yaw:='+params[self.oppened_world][5]]

            roslauch_file = [(self.spawn_path,roslaunch_arg)]
            self.agv_spanned = True
            self.launch(roslauch_file)
        else:
            self._widget.info_text.setText('Please pick at least one sensor')


##AGV3 SPAWN
    def check_box_camera_3_cb(self,val):
        if val :
            self.agv_3_sensor[0] = 1

    def check_box_IMU_3_cb(self,val):
        if val :
            self.agv_3_sensor[1] = 1

    def check_box_Lidars_3_cb(self,val):
        if val :
            self.agv_3_sensor[2] = 1

    def check_box_Velodyne_3_cb(self,val):
        if val :
            self.agv_3_sensor[3] = 1

    def spawn_agv_button_3_cb(self):


        if self.oppened_world == None :
            self._widget.info_text.setText('To Spawn AGV Please Start Simulation First')
            return
            
        if not self.oppened_world == '50x50':
            self._widget.info_text.setText('AGV3 cannot open at {} world'.format(self.oppened_world))
            return

        if self.agv_3_sensor[0] or self.agv_3_sensor[1] or self.agv_3_sensor[2] or self.agv_3_sensor[3]:
            
            sensor_location = self._widget.comboBox_sensor_loc_4.currentText()
            robot_tf_prefix = list(self.agv_spawn_config['tf_prefix'].keys())[2]
            sensors= ''
            for i  in range(len(self.agv_3_sensor)):
                if self.agv_3_sensor[i]:
                    if sensors == '' :sensors=sensors+self.agv_spawn_config['sensors'][i]
                    else : sensors=sensors+','+self.agv_spawn_config['sensors'][i]

            params = self.agv_spawn_config['tf_prefix'][robot_tf_prefix]



            roslaunch_arg = ['tf_prefix:='+robot_tf_prefix, 
                             'robot:='+params['robot'],
                             'location:='+sensor_location,
                             'on:='+sensors,
                             'sensors:='+params['base_link'],
                             'robot_name:='+params['robot_name'],
                             'x:='+params[self.oppened_world][0],
                             'y:='+params[self.oppened_world][1],
                             'z:='+params[self.oppened_world][2],
                             'roll:='+params[self.oppened_world][3],
                             'pitch:='+params[self.oppened_world][4],
                             'yaw:='+params[self.oppened_world][5]]

            roslauch_file = [(self.spawn_path,roslaunch_arg)]

            self.launch(roslauch_file)
            self.agv_spanned = True
        else:
            self._widget.info_text.setText('Please pick at least one sensor')


    ## SLAM FUNCT

    def slam_button_cb(self):

        pass_arg =self._widget.comboBox_slam.currentText()

        roslaunch_arg = ['slam_methods:='+pass_arg]

        roslaunch_file = [(self.slam_path,roslaunch_arg)]

        if self.agv_spanned :
            self.launch(roslaunch_file)
            self.slam_started= True
            self._widget.info_text_slam.setStyleSheet('background-color: green;')
            self._widget.info_text_slam.setText("Slam sucssefully started with"+ pass_arg + " method")
        else :
            self._widget.info_text_slam.setStyleSheet('background-color: red;')
            self._widget.info_text_slam.setText("Before Initilaize Slam , Spawn a ITU_AGV")


    def get_topic_cb(self):
        topics = rospy.get_published_topics()
        self._widget.comboBox_topics.clear()

        for tpcs in topics:

            self._widget.comboBox_topics.addItem(tpcs[0])
       

    def add_to_bag_cb(self):

       _topic = self._widget.comboBox_topics.currentText()
       self.bag_topics.append(_topic) 
       _bag_text = ""
       for tpcs in self.bag_topics:
           _bag_text = _bag_text +  "\n" + tpcs

       self._widget.textEdit_bag_list.setText(_bag_text)

    def reset_bag_list_cb(self):
        self.bag_topics = []
        self._widget.textEdit_bag_list.clear()


    def record_bag_cb(self):
        _bag_name = self._widget.lineEdit_bag_name.text()
        _bag_duration = self._widget.lineEdit_bag_duration.text()

        _topics = ""

        for tpcs in self.bag_topics:
            _topics = _topics + "  " + tpcs

        roslaunch_args = ['topics_name:='+_topics,
                          'bag_name:='+ _bag_name,
                          'duration:='+ _bag_duration]

        roslaunch_file = [(self.bag_record_path,roslaunch_args)]

        self._widget.info_text.setText(_topics + ' topics are recording to '+ _bag_name + ' file for ' + _bag_duration + " seconds.")

        self.launch(roslaunch_file)
    

    def save_map_button_cb(self):

        # os.system('rosrun map_server map_saver -f deneme_mapppp ')
        # node =roslaunch.core.Node('map_server' , 'map_saver','map_saver','',None,'-f new_map map:=/home/murat')
        # launch = roslaunch.scriptapi.ROSLaunch()
        # launch.start()

        # process = launch.launch(node)
        # print(process.is_alive())    
        pass