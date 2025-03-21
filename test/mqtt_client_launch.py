import time
import paho.mqtt.client as mqtt

def on_message(client, userdata, msg):
    print(f'✅ MQTT Test received on {msg.topic}: {msg.payload.decode()}', flush=True)

client = mqtt.Client()
client.connect("mqtt_broker", 1883, 60)

client.subscribe("#")  # Listen to all topics
client.on_message = on_message
client.loop_start()

print("🕐 Waiting for ROS → MQTT messages...")
time.sleep(5)  # Give time for ROS → MQTT bridge to start publishing

print("📤 Sending MQTT → ROS test messages every second for 30 seconds...", flush=True)
for i in range(30):
    message = f"Hello from MQTT client, message {i + 1}"
    client.publish("mqtt_to_ros", message)
    print(f"📨 Published: {message}", flush=True)
    time.sleep(1)

client.loop_stop()
print("✅ MQTT test completed.", flush=True)

