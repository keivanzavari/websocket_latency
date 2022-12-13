#! /usr/bin/env python3
import rospy
import time
import asyncio
import websockets
import json
from rospy_message_converter import message_converter

import sv_msgs.msg


class CommunicationParams:

    def __init__(self, topic: str = "") -> None:
        self.uri = "ws://10.20.6.12:49152"
        self.topic = topic
        self.msg_type = sv_msgs.msg.PTUState_v2._type


async def subscribe(param: CommunicationParams) -> float:
    async with websockets.connect(param.uri) as websocket:
        await websocket.send(json.dumps({"op": "subscribe", "topic": param.topic}))
        while True:
            not_started = True
            start = 0.0
            while not_started:
                message = json.loads(await websocket.recv())
                ptu_state: sv_msgs.msg.PTUState_v2 = message_converter.convert_dictionary_to_ros_message(
                    param.msg_type, message["msg"])
                to_sec = ptu_state.header.stamp.to_sec()
                if to_sec == 0.0:
                    start = time.time()
                    print("started")
                    not_started = False

            message = json.loads(await websocket.recv())
            ptu_state: sv_msgs.msg.PTUState_v2 = message_converter.convert_dictionary_to_ros_message(
                param.msg_type, message["msg"])
            recv_timestamp = time.time()
            elapsed = recv_timestamp - start
            print(f"elapsed {elapsed}")
            not_started = True


if __name__ == "__main__":

    param = CommunicationParams(topic="/api/external/timestamp")

    try:
        rospy.init_node("subcriber", log_level=rospy.WARN)

        task = subscribe(param)
        loop = asyncio.get_event_loop()
        loop.run_until_complete(task)
        rospy.spin()
    except KeyboardInterrupt:
        rospy.loginfo("Shutting down.")
    finally:
        loop.close()
