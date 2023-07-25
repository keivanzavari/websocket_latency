from typing import Type

import std_msgs.msg

MSG_TYPE: str = std_msgs.msg.Header._type
MSG_CLASS: Type = std_msgs.msg.Header


class CommunicationParams:

    def __init__(self, topic: str = "") -> None:
        self.uri = "ws://127.0.0.1:49152"
        self.freq = 1.0
        self.topic = topic
