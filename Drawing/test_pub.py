import paho.mqtt.client as mqtt
import time

BROKER = "192.168.0.238"
PORT = 1883
TOPIC = "test"

def on_connect(client, userdata, flags, rc):
    if rc == 0:
        print("Connected to MQTT broker")
    else:
        print(f"Failed to connect, return code {rc}")

def main():
    client = mqtt.Client()
    client.on_connect = on_connect

    client.connect(BROKER, PORT, keepalive=60)
    client.loop_start()

    try:
        while True:
            message = input("Enter message to publish (Ctrl+C to exit): ")
            client.publish(TOPIC, message)
            print(f"Published to {TOPIC}: {message}")
            time.sleep(0.1)
    except KeyboardInterrupt:
        print("\nExiting...")
    finally:
        client.loop_stop()
        client.disconnect()

if __name__ == "__main__":
    main()