# Pilot02 Connectivity Bridge ROS2 MQTT

This repository contains a docker based version of the https://github.com/ika-rwth-aachen/mqtt_client package that acts as a bridge between ROS2 topics and MQTT channels.

## Description

The repository includes the following components:
- Configuration files (how to address the bridge from ROS->MQTT and MQTT->ROS)
- Launch files 
- Docker setup for easy containerization
- A Docker Compose-based test environment to verify end-to-end functionality: ROS publisher → MQTT bridge → MQTT broker → Python MQTT client; Python MQTT client → MQTT broker → MQTT bridge → ROS subscriber.

## Guidelines for build and test the component 

### 1. **Build the Main Docker Image:**

In this step, we build the Docker image using the provided `Dockerfile`. The image is named `pilot02-connectivity-bridge-ros2-mqtt`.

```bash
docker build -t pilot02-connectivity-bridge-ros2-mqtt .
```
Make sure the path to your configuration and launch files is correctly mapped to the Docker container.

### 2. **Run the ROS 2 Container:**

After building the Docker image, you can run the container using the following command:

```bash
docker run -e LAUNCH_FILE=your_custom_launch_file.py -e CONFIG_FILE=your_custom_config_file pilot02-connectivity-bridge-ros2-mqtt
```

This will start the container and launch the MQTT bridge with the configuration given.

### 3. **Build and Run the test automation:**

Test automation is integrated by docker-compose file:

Run: 
```bash
docker-compose up --build
```

In case it works, you should see:
```python
mqtt_broker       | 1742547211: New connection from 172.28.0.4:57843 on port 1883.
mqtt_broker       | 1742547211: New client connected from 172.28.0.4:57843 as auto-DF9E9C87-73E5-5BBE-B760-303552DAEE97 (p2, c1, k60).
ros_subscriber    | [INFO] [1742547211.606496306] [ros_subscriber]: RosSubscriber started!
ros_publisher     | [INFO] [1742547211.625629761] [ros_publisher]: RosPublisher started!
ros_mqtt_bridge   | [mqtt_client-1] [INFO] [1742547212.467938864] [mqtt_client]: Subscribed ROS topic '/ros_to_mqtt' of type 'std_msgs/msg/String'
ros_publisher     | [INFO] [1742547212.635628896] [ros_publisher]: RosPublisher Published: "Hello from ROS Publisher! Message 1"
mqtt_test_client  | 🕐 Waiting for ROS → MQTT messages...
mqtt_test_client  | ✅ MQTT Test received on to_mqtt: Hello from ROS Publisher! Message 1
ros_publisher     | [INFO] [1742547213.635568929] [ros_publisher]: RosPublisher Published: "Hello from ROS Publisher! Message 2"
mqtt_test_client  | ✅ MQTT Test received on to_mqtt: Hello from ROS Publisher! Message 2
ros_publisher     | [INFO] [1742547214.634762917] [ros_publisher]: RosPublisher Published: "Hello from ROS Publisher! Message 3"
mqtt_test_client  | ✅ MQTT Test received on to_mqtt: Hello from ROS Publisher! Message 3
ros_publisher     | [INFO] [1742547215.634856061] [ros_publisher]: RosPublisher Published: "Hello from ROS Publisher! Message 4"
mqtt_test_client  | ✅ MQTT Test received on to_mqtt: Hello from ROS Publisher! Message 4
mqtt_test_client  | 📤 Sending MQTT → ROS test messages every second for 30 seconds...
mqtt_test_client  | 📨 Published: Hello from MQTT client, message 1
mqtt_test_client  | ✅ MQTT Test received on mqtt_to_ros: Hello from MQTT client, message 1
ros_mqtt_bridge   | [mqtt_client-1] [INFO] [1742547216.313187661] [mqtt_client]: ROS publisher message type on topic '/from_mqtt' set to 'std_msgs/msg/String'
ros_subscriber    | [INFO] [1742547216.316337837] [ros_subscriber]: RosSubscriber received via MQTT → ROS: "Hello from MQTT client, message 1"
ros_publisher     | [INFO] [1742547216.635094823] [ros_publisher]: RosPublisher Published: "Hello from ROS Publisher! Message 5"
mqtt_test_client  | ✅ MQTT Test received on to_mqtt: Hello from ROS Publisher! Message 5
mqtt_test_client  | 📨 Published: Hello from MQTT client, message 2
mqtt_test_client  | ✅ MQTT Test received on mqtt_to_ros: Hello from MQTT client, message 2
ros_subscriber    | [INFO] [1742547217.315315265] [ros_subscriber]: RosSubscriber received via MQTT → ROS: "Hello from MQTT client, message 2"
mqtt_test_client  | ✅ MQTT Test received on to_mqtt: Hello from ROS Publisher! Message 6
ros_publisher     | [INFO] [1742547217.635658172] [ros_publisher]: RosPublisher Published: "Hello from ROS Publisher! Message 6"
mqtt_test_client  | 📨 Published: Hello from MQTT client, message 3
mqtt_test_client  | ✅ MQTT Test received on mqtt_to_ros: Hello from MQTT client, message 3
ros_subscriber    | [INFO] [1742547218.314429357] [ros_subscriber]: RosSubscriber received via MQTT → ROS: "Hello from MQTT client, message 3"
ros_publisher     | [INFO] [1742547218.635468667] [ros_publisher]: RosPublisher Published: "Hello from ROS Publisher! Message 7"
mqtt_test_client  | ✅ MQTT Test received on to_mqtt: Hello from ROS Publisher! Message 7
mqtt_test_client  | 📨 Published: Hello from MQTT client, message 4
```

## Bridge configuration

Include your bridge configuration from ROS to MQTT and from MQTT to ROS, as well as the MQTT broker:

```yaml
/**/*:
  ros__parameters:
    broker:
      host: mqtt_broker
      port: 1883
    bridge:
      ros2mqtt:
        ros_topics: 
          - /ros_to_mqtt
        /ros_to_mqtt:
          mqtt_topic: to_mqtt
          primitive: true
      mqtt2ros:
        mqtt_topics: 
          - mqtt_to_ros
        mqtt_to_ros:
          ros_topic: /from_mqtt
          primitive: true
```

## Contributing

Feel free to open issues or submit pull requests. Contributions are welcome!

## License

This project is licensed under the MIT - see the [LICENSE](LICENSE) file for details.
