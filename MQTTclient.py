from dataclasses import dataclass
from distutils.log import info
from paho.mqtt import client as mqttclient

class MQTTclient:
    def __init__(self, broker, port, client_id):
        self.broker = broker
        self.port = port
        self.client_id = client_id
        self.waypointlist = []
        self.waypointcame = False

    def connect(self):
        self.client = mqttclient.Client(self.client_id)
        self.client.connect(self.broker, self.port)

    def publish(self, message, topic):
        self.client.publish(topic, message)

    def subscribe(self, topic):
        self.client.subscribe(topic, 1)

    def onMessage(self, client, userdata, message):
        global info
        info = str(message.payload.decode("utf-8"))
        if (message.topic == "control/auto"):
            if(info == "StartWaypoint"):
                self.waypointlist.clear()
            elif(info == "EndWaypoint"):
                #do something to inform that all of them arrived
                self.waypointcame = True
            else:
                self.waypointlist.append(info)

    def loop_start(self):
        self.client.loop_start()

    def init(self, subscribeTopic):
        self.connect()
        self.subscribe(subscribeTopic)
        self.client.on_message = self.onMessage
        self.client.loop_start()

    def writeMessageArray(self):
        print(self.message)