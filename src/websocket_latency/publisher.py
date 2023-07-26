#! /usr/bin/env python3
from typing import Any, Dict, Type
import asyncio
import websockets
import json
import threading

import rospy
import dynamic_reconfigure.server
from rospy_message_converter import message_converter

import websocket_latency.definitions as definitions

from websocket_latency.cfg import DynamicParametersConfig


def dynamic_reconfigure_callback(config: Dict[str, Any], level: Any,
                                 param: definitions.CommunicationParams) -> Dict[str, Any]:
    param.freq = config.freq
    return config


async def publish(param: definitions.CommunicationParams) -> None:
    async with websockets.connect(param.uri) as websocket:
        await websocket.send(json.dumps({"op": "advertise", "topic": param.topic, "type": definitions.MSG_TYPE}))
        await asyncio.sleep(1.0 / param.freq)
        msg_zero = definitions.MSG_CLASS()
        msg_one = definitions.MSG_CLASS()

        while True:
            msg_to_send = {
                "op": "publish",
                "topic": param.topic,
                "msg": message_converter.convert_ros_message_to_dictionary(msg_zero)
            }
            await websocket.send(json.dumps(msg_to_send))

            msg_one.stamp = rospy.Time.now()
            msg_to_send = {
                "op": "publish",
                "topic": param.topic,
                "msg": message_converter.convert_ros_message_to_dictionary(msg_one)
            }

            await websocket.send(json.dumps(msg_to_send))
            await asyncio.sleep(1.0 / param.freq)


def publish_on_thread(param):
    loop = asyncio.new_event_loop()
    asyncio.set_event_loop(loop)

    loop.run_until_complete(publish(param))
    loop.close()


if __name__ == "__main__":

    param = definitions.CommunicationParams(topic="/external/timestamp")

    try:
        rospy.init_node("publisher", log_level=rospy.WARN)
        dynamic_reconfigure_srv = dynamic_reconfigure.server.Server(
            DynamicParametersConfig, lambda config, level: dynamic_reconfigure_callback(config, level, param))

        _thread = threading.Thread(target=publish_on_thread, args=(param, ))
        _thread.start()
        rospy.spin()
    except KeyboardInterrupt:
        rospy.loginfo("Shutting down.")
