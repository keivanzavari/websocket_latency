#! /usr/bin/env python3
from typing import Any, Dict
import rospy
import dynamic_reconfigure.server
from websocket_latency.cfg import DynamicParametersConfig
import time
import asyncio
import websockets
import json
import sv_msgs.msg
from rospy_message_converter import message_converter


class CommunicationParams:

    def __init__(self, topic: str = "") -> None:
        self.uri = "ws://127.0.0.1:49152"
        self.freq = 1.0
        self.topic = topic
        self.msg_type = sv_msgs.msg.PTUState_v2._type


def dynamic_reconfigure_callback(config: Dict[str, Any], level: Any, param: CommunicationParams) -> Dict[str, Any]:
    param.freq = config.freq
    return config


async def publish(param: CommunicationParams) -> None:
    async with websockets.connect(param.uri) as websocket:
        await websocket.send(json.dumps({"op": "advertise", "topic": param.topic, "type": param.msg_type}))
        await asyncio.sleep(1.0 / param.freq)
        p0 = sv_msgs.msg.PTUState_v2()
        p1 = sv_msgs.msg.PTUState_v2()

        while True:
            msg_to_send = {
                "op": "publish",
                "topic": param.topic,
                "msg": message_converter.convert_ros_message_to_dictionary(p0)
            }
            await websocket.send(json.dumps(msg_to_send))

            p1.header.stamp = rospy.Time.now()
            msg_to_send = {
                "op": "publish",
                "topic": param.topic,
                "msg": message_converter.convert_ros_message_to_dictionary(p1)
            }

            await websocket.send(json.dumps(msg_to_send))
            await asyncio.sleep(1.0 / param.freq)


def publish_on_thread(param):
    loop = asyncio.new_event_loop()
    asyncio.set_event_loop(loop)

    loop.run_until_complete(publish(param))
    loop.close()


import threading
if __name__ == "__main__":

    param = CommunicationParams(topic="/api/external/timestamp")

    try:
        rospy.init_node("publisher", log_level=rospy.WARN)
        dynamic_reconfigure_srv = dynamic_reconfigure.server.Server(
            DynamicParametersConfig, lambda config, level: dynamic_reconfigure_callback(config, level, param))

        _thread = threading.Thread(target=publish_on_thread, args=(param, ))
        _thread.start()
        rospy.spin()
    except KeyboardInterrupt:
        rospy.loginfo("Shutting down.")
