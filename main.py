from enes100 import enes100
import config
from pid import PID
from voltageDetector import VoltageDetector
import motion
import math_utils
import time
import testing


# Intialize ENES100 and ESP32
def setup():
    enes100.begin(
        team_name=config.TEAM_NAME,
        team_type=config.TEAM_TYPE,
        aruco_id=config.ARUCO_ID,
        room_num=config.ROOM_NUM
    )


def main():
    setup()
    testing.aruco_reading()


git config --global user.email "akshayjana646@gmail.com"
git config --global user.name "AkshayJana646"