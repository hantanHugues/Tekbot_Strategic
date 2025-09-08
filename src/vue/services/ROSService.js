import ROSLIB from 'roslib';

class ROSService {
  constructor() {
    this.ros = null;
    this.connected = false;
    this.topics = {};
    this.services = {};
    this.listeners = [];
  }

  connect(url = 'ws://localhost:9090') {
    return new Promise((resolve, reject) => {
      this.ros = new ROSLIB.Ros({
        url: url
      });

      this.ros.on('connection', () => {
        console.log('Connected to ROS WebSocket server');
        this.connected = true;
        this.initializeTopicsAndServices();
        resolve(true);
      });

      this.ros.on('error', (error) => {
        console.error('Error connecting to ROS WebSocket server:', error);
        this.connected = false;
        reject(error);
      });

      this.ros.on('close', () => {
        console.log('Connection to ROS WebSocket server closed');
        this.connected = false;
      });
    });
  }

  initializeTopicsAndServices() {
    // Topics pour la réception de données
    this.topics = {
      // Robot
      robotOdom: new ROSLIB.Topic({
        ros: this.ros,
        name: '/robot/odom',
        messageType: 'nav_msgs/Odometry'
      }),
      
      robotBattery: new ROSLIB.Topic({
        ros: this.ros,
        name: '/robot/battery_state',
        messageType: 'sensor_msgs/BatteryState'
      }),

      qrScanResult: new ROSLIB.Topic({
        ros: this.ros,
        name: '/robot/qr_scan_result',
        messageType: 'custom_msgs/QuartierInfo'
      }),

      // Game
      gameScore: new ROSLIB.Topic({
        ros: this.ros,
        name: '/game/score',
        messageType: 'std_msgs/Int32'
      }),

      gameTimer: new ROSLIB.Topic({
        ros: this.ros,
        name: '/game/timer',
        messageType: 'std_msgs/Float32'
      }),

      gameLog: new ROSLIB.Topic({
        ros: this.ros,
        name: '/game/log',
        messageType: 'std_msgs/String'
      }),

      // Convoyeur
      conveyorSensorColor: new ROSLIB.Topic({
        ros: this.ros,
        name: '/conveyor/sensor/color',
        messageType: 'std_msgs/ColorRGBA'
      }),

      conveyorMotorStatus: new ROSLIB.Topic({
        ros: this.ros,
        name: '/conveyor/motor/status',
        messageType: 'std_msgs/Bool'
      }),

      ldrFront: new ROSLIB.Topic({
        ros: this.ros,
        name: '/conveyor/sensor/ldr_front/raw_value',
        messageType: 'std_msgs/Int32'
      }),

      ldrBack: new ROSLIB.Topic({
        ros: this.ros,
        name: '/conveyor/sensor/ldr_back/raw_value',
        messageType: 'std_msgs/Int32'
      }),

      // Bras robotique
      armStatus: new ROSLIB.Topic({
        ros: this.ros,
        name: '/arm/status',
        messageType: 'std_msgs/String'
      }),

      // Tri
      cubeDetected: new ROSLIB.Topic({
        ros: this.ros,
        name: '/tri/color_detected',
        messageType: 'std_msgs/String'
      }),

      cubeSorted: new ROSLIB.Topic({
        ros: this.ros,
        name: '/tri/cube_sorted',
        messageType: 'custom_msgs/SortedWaste'
      })
    };

    // Services pour les commandes
    this.services = {
      // Calibration
      calibrateLDR: new ROSLIB.Service({
        ros: this.ros,
        name: '/conveyor/calibrate_ldr',
        serviceType: 'custom_srv/CalibrateLDR'
      }),

      learnColor: new ROSLIB.Service({
        ros: this.ros,
        name: '/conveyor/learn_color',
        serviceType: 'custom_srv/LearnColor'
      }),

      getColorProfiles: new ROSLIB.Service({
        ros: this.ros,
        name: '/conveyor/get_color_profiles',
        serviceType: 'custom_srv/GetColorProfiles'
      }),

      // Robot
      goToTarget: new ROSLIB.Service({
        ros: this.ros,
        name: '/robot/go_to_target',
        serviceType: 'custom_srv/SetNavigationGoal'
      }),

      startQRScan: new ROSLIB.Service({
        ros: this.ros,
        name: '/robot/start_qr_scan',
        serviceType: 'std_srvs/Trigger'
      }),

      // Game
      requestManualReset: new ROSLIB.Service({
        ros: this.ros,
        name: '/game/request_manual_reset',
        serviceType: 'std_srvs/Trigger'
      }),

      // Bras
      pickAndPlace: new ROSLIB.Service({
        ros: this.ros,
        name: '/arm/pick_and_place',
        serviceType: 'custom_srv/SortWaste'
      })
    };
  }

  // Méthodes pour s'abonner aux topics
  subscribe(topicName, callback) {
    if (this.topics[topicName]) {
      this.topics[topicName].subscribe(callback);
      this.listeners.push({ topic: topicName, callback });
    }
  }

  unsubscribe(topicName, callback) {
    if (this.topics[topicName]) {
      this.topics[topicName].unsubscribe(callback);
    }
  }

  // Méthodes pour appeler les services
  async callService(serviceName, request = {}) {
    return new Promise((resolve, reject) => {
      if (this.services[serviceName]) {
        const serviceRequest = new ROSLIB.ServiceRequest(request);
        
        this.services[serviceName].callService(serviceRequest, (result) => {
          resolve(result);
        }, (error) => {
          reject(error);
        });
      } else {
        reject(new Error(`Service ${serviceName} not found`));
      }
    });
  }

  // Méthodes utilitaires
  isConnected() {
    return this.connected;
  }

  disconnect() {
    if (this.ros) {
      this.ros.close();
      this.connected = false;
    }
  }

  // Nettoyage des listeners
  cleanup() {
    this.listeners.forEach(({ topic, callback }) => {
      this.unsubscribe(topic, callback);
    });
    this.listeners = [];
  }
}

export default new ROSService();