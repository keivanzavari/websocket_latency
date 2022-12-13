#! /usr/bin/env python3
from typing import Any, Dict
import rospy
import dynamic_reconfigure.server
from websocket_latency.cfg import ExampleDynamicParametersConfig
import time
import asyncio
import websockets
import json


class CommunicationParams:

    def __init__(self, topic: str = "") -> None:
        self.uri = "ws://127.0.0.1:49152"
        self.freq = 1.0
        self.topic = topic


def dynamic_reconfigure_callback(config: Dict[str, Any], level: Any, param: CommunicationParams) -> Dict[str, Any]:
    param.freq = config.freq
    return config


async def subscribe(param: CommunicationParams) -> float:
    async with websockets.connect(param.uri) as websocket:
        await websocket.send(json.dumps({"op": "subscribe", "topic": param.topic}))
        message = json.loads(await websocket.recv())
        msg_timestamp = message["msg"]["data"]
        recv_timestamp = time.time()
        elapsed = recv_timestamp - msg_timestamp
        # return elapsed
        print(f"elapsed {elapsed}")


import sv_msgs.msg


async def publish(param: CommunicationParams) -> None:
    async with websockets.connect(param.uri) as websocket:
        await websocket.send(json.dumps({"op": "advertise", "topic": param.topic, "type": sv_msgs.msg.cvPerson._type}))
        # await websocket.send(json.dumps({"op": "subscribe", "topic": param.topic}))
        await asyncio.sleep(1.0 / param.freq)
        while True:
            msg_to_send = {"op": "publish", "topic": param.topic, "msg": {"data": time.time()}}
            await websocket.send(json.dumps(msg_to_send))

            # await websocket.send(json.dumps({"op": "subscribe", "topic": param.topic}))
            # message = json.loads(await websocket.recv())
            # msg_timestamp = message["msg"]["data"]
            # recv_timestamp = time.time()
            # elapsed = recv_timestamp - msg_timestamp
            # print(f"elapsed {elapsed}")

            await asyncio.sleep(1.0 / param.freq)


def publish_on_thread(param):
    loop = asyncio.new_event_loop()
    asyncio.set_event_loop(loop)

    loop.run_until_complete(publish(param))
    loop.close()


import threading
if __name__ == "__main__":

    param = CommunicationParams(topic="/external/timestamp")

    try:
        rospy.init_node("publisher", log_level=rospy.WARN)
        dynamic_reconfigure_srv = dynamic_reconfigure.server.Server(
            ExampleDynamicParametersConfig, lambda config, level: dynamic_reconfigure_callback(config, level, param))

        _thread = threading.Thread(target=publish_on_thread, args=(param, ))
        _thread.start()
        rospy.spin()
    except KeyboardInterrupt:
        rospy.loginfo("Shutting down.")
