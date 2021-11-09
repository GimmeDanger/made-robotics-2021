#!/usr/bin/env python

import asyncio
import json

import websockets
from pid import PID


# TODO: Initialize the pid variable.
MAX_SPEED = 25
steering_pid = PID(0.100, 0.00000, 10.000)
throttle_pid = PID(0.200, 0.00010, 5.000)

# Checks if the SocketIO event has JSON data.
# If there is data the JSON object in string format will be returned,
# else the empty string "" will be returned.
def getData(message):
    try:
        start = message.find("[")
        end = message.rfind("]")
        return message[start : end + 1]
    except:
        return ""


async def handleTelemetry(websocket, msgJson):
    cte = msgJson[1]["cte"]
    speed = msgJson[1]["speed"]
    angle = msgJson[1]["steering_angle"]

    print("CTE: ", cte, ", speed: ", speed, ", angle: ", angle)

    # TODO: Calculate steering value here, remember the steering value is
    # [-1, 1].
    # NOTE: Feel free to play around with the throttle and speed.
    # Maybe use another PID controller to control the speed!
    steering_pid.UpdateError(float(cte))
    steer_value = -steering_pid.TotalError()
    steer_value = max(min(steer_value, 1), -1)
    #target_speed = MAX_SPEED if abs(steer_value) < 0.1 else MAX_SPEED / 2
    steer_coef = 1 if abs(steer_value) < 0.1 else 0
    target_speed = MAX_SPEED * 0.8 + MAX_SPEED * steer_coef * 0.2
    throttle_pid.UpdateError(float(speed) - target_speed)
    throttle_value = -throttle_pid.TotalError()
    #throttle_value = max(min(throttle_value, 5.0), -5.0)
    print("steer_value: ", steer_value, ", target_speed: ", target_speed, ", throttle_value: ", throttle_value)

    response = {}
    response["steering_angle"] = steer_value
    response["throttle"] = throttle_value

    msg = '42["steer",' + json.dumps(response) + "]"

    await websocket.send(msg)


async def echo(websocket, path):
    async for message in websocket:
        if len(message) < 3 or message[0] != "4" or message[1] != "2":
            return

        s = getData(message)
        msgJson = json.loads(s)

        event = msgJson[0]
        if event == "telemetry":
            await handleTelemetry(websocket, msgJson)
        else:
            msg = '42["manual",{}]'
            await websocket.send(msg)


def main():
    start_server = websockets.serve(echo, "localhost", 4567)

    asyncio.get_event_loop().run_until_complete(start_server)
    asyncio.get_event_loop().run_forever()


if __name__ == "__main__":
    main()
