#! /usr/bin/env python3
import time
import asyncio
import websockets
import json

from rospy_message_converter import message_converter
import rospy

import websocket_latency.definitions as definitions


async def subscribe(param: definitions.CommunicationParams) -> float:
    async with websockets.connect(param.uri) as websocket:
        await websocket.send(json.dumps({"op": "subscribe", "topic": param.topic}))
        while True:
            started = False
            start_time = 0.0
            while not started:
                message = json.loads(await websocket.recv())
                msg_zero: definitions.MSG_CLASS = message_converter.convert_dictionary_to_ros_message(
                    definitions.MSG_TYPE, message["msg"])
                to_sec = msg_zero.stamp.to_sec()
                if to_sec == 0.0:
                    start_time = time.time()
                    rospy.loginfo("started")
                    started = True

            message = json.loads(await websocket.recv())
            _: definitions.MSG_CLASS = message_converter.convert_dictionary_to_ros_message(
                definitions.MSG_TYPE, message["msg"])
            elapsed = time.time() - start_time
            rospy.logwarn(f"elapsed {elapsed}")
            started = False


if __name__ == "__main__":

    param = definitions.CommunicationParams(topic="/external/timestamp")
    # If subscriber is running on a different machine, change the uri like this:
    # param.uri = "ws://{publisher_ip}:49152"

    try:
        rospy.init_node("subcriber", log_level=rospy.INFO)

        task = subscribe(param)
        loop = asyncio.get_event_loop()
        loop.run_until_complete(task)
        rospy.spin()
    except KeyboardInterrupt:
        rospy.loginfo("Shutting down.")
    finally:
        loop.close()
