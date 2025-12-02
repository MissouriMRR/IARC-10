from asyncio.queues import Queue
import json
from typed_dicts_classes import MessageData
from json_config_reader import json_config_reader

import asyncio
import server
import client
import argparse

# DOCS: How to merge this with path finding:
"""
1: Change this to a async start_networking() function that runs as a task from real main function
2. Pass in clientInData, clientOutData, and serverOutData 
3. May need to put this code on a separate thread or something and find a way to pass queues in across threads (gonna be a necessary pain :( )
"""


async def main():
    # Create jsonConfigData instance to get data from config file
    jsonConfigData: json_config_reader = json_config_reader()

    # Create flag parser
    parser = argparse.ArgumentParser()

    # ID flag -i <Drone ID>
    parser.add_argument("-i", "--id", help="Self ID", type=int)
    # Startup override flag -i (1=override, anything else does not override)
    parser.add_argument("-s", "--skip", help="Startup override (1=true)", type=int)

    # Stores flag arguments passed on startup
    args = parser.parse_args()

    # Get our drones id from the flag if provided
    droneId: int
    if args.id is not None:
        droneId = args.id
        jsonConfigData.set_self_id(droneId)
    else:
        droneId = int(jsonConfigData.get_self_id())

    # TODO temporary startup skip flag. Need to rework this for a better system flag system
    # Check for system to arg to skip json config startup sequence

    startUpOverride: bool
    try:
        value = args.skip
        if value == 1:
            startUpOverride = True
        else:
            startUpOverride = False
    except Exception:
        startUpOverride = False

    # If not drone 1 and we're not overriding startup, start a temporary server and wait for update json file.
    if droneId != 1 and not startUpOverride:
        # TODO FIX THIS TO GET CURRENT IP FROM SYSTEM AND USE THAT FOR TEMP IP
        # Only start temp server and then move on to standard execution

        print("creating startupServer")
        startupServerOutData: Queue[str] = asyncio.Queue()
        startupServerInstance = server.Server(
            jsonConfigData=jsonConfigData, serverOutData=startupServerOutData
        )
        serverTask = asyncio.create_task(startupServerInstance.start_server_async())
        while True:
            # check for updated json message
            if not startupServerOutData.empty():
                # Check to see if JSON has been overwritten
                if await startupServerOutData.get() == "JSON Overwritten!":
                    break
            await asyncio.sleep(1)
        # Re-init jsonConfigData
        await asyncio.sleep(10)
        print(f"JSON overwritten on drone {droneId} and reinitializing the reader")
        jsonConfigData = json_config_reader()
        # may need code in here to get ip from pi and then it manually in config

    # Create Server and Client Data queues to pass data in and out of tasks
    serverOutData: Queue[str] = asyncio.Queue()
    clientInData: Queue[MessageData] = asyncio.Queue()
    # TODO POST LVP: TALK TO TEAM AND TRANSITION OVER TO USING MESSAGE DATA FOR CLIENT OUTPUT. MAJOR CHANGE
    clientOutData: Queue[str] = asyncio.Queue()

    # Instantiate Server and Client
    serverInstance = server.Server(
        jsonConfigData=jsonConfigData, serverOutData=serverOutData
    )
    clientInstance = client.Client(
        jsonConfigData=jsonConfigData,
        clientInData=clientInData,
        clientOutData=clientOutData,
    )

    # Run both server and client concurrently
    serverTask = asyncio.create_task(serverInstance.start_server_async())
    clientTask = asyncio.create_task(clientInstance.start_client_async())

    # Get our drones id (the sys arg here allows you pass in a self id from command line for efficient testing)

    # Create heartbeat message
    heartBeatMessage: MessageData = {
        "messageId": 504,
        "dronesToSendData": [],
        "data": {
            "timestamp": 0.0,
            "senderId": droneId,
            "payload": "Hello server!",
        },
    }
    taskMessage: MessageData = {
        "messageId": 604,
        "dronesToSendData": [1],
        "data": {
            "timestamp": 0.0,
            "senderId": droneId,
            "payload": "Hello server!",
        },
    }
    jsonOverwriteStartupMessage: MessageData = {
        "messageId": 501,
        "dronesToSendData": [],
        "data": {
            "successfulOverwrite": False,
            "payload": "",
        },
    }

    try:
        print("Server and Client started")

        # Startup loop for drone 1 to send updated config to all other drones. Does not run if start up is overridden
        if droneId == 1 and not startUpOverride:
            # Instantiate otherDrones lists
            otherDronesIds: list[int] = []
            # Loop through drones all drones to get IPs, Ports, and IDs of drones to connect to
            for i in range(1, jsonConfigData.get_number_of_drones() + 1):
                # If drone is self (drone running this script) don't add them otherDrones list
                if i != droneId:
                    # Add other drones IP and Ports to their respective lists
                    otherDronesIds.append(i)
            print(otherDronesIds)
            # move this up to above while loop and then have time out else logic inside if not empty
            for id in otherDronesIds:
                messageToSend = jsonOverwriteStartupMessage.copy()
                messageToSend["dronesToSendData"] = [id]
                messageToSend["data"] = messageToSend["data"].copy()
                messageToSend["data"]["payload"] = (
                    jsonConfigData.get_json_text_data_for_startup(id)
                )
                print(messageToSend["dronesToSendData"])
                await clientInData.put(messageToSend)
            while otherDronesIds:
                if not clientOutData.empty():
                    clientMsg: str = await clientOutData.get()
                    try:
                        clientMsgJson: MessageData = json.loads(clientMsg)
                        if clientMsgJson.get("messageId") == 501:
                            if clientMsgJson["data"]["successfulOverwrite"]:
                                idToRemove = clientMsgJson["dronesToSendData"][0]
                                if idToRemove in otherDronesIds:
                                    otherDronesIds.remove(idToRemove)
                                    print(
                                        f"Successfully sent new json data to drone {idToRemove} and updated it's file. Now removing it from otherDronesIds list"
                                    )
                            else:
                                # Put time out logic here to add in new message
                                print("Timeout received, resending...")
                                idToResend = clientMsgJson["dronesToSendData"][0]
                                messageToSend = jsonOverwriteStartupMessage.copy()
                                messageToSend["dronesToSendData"] = [idToResend]
                                messageToSend["data"] = messageToSend["data"].copy()
                                messageToSend["data"]["payload"] = (
                                    jsonConfigData.get_json_text_data_for_startup(
                                        idToResend
                                    )
                                )
                                await clientInData.put(messageToSend)

                    except Exception:
                        print("Trouble reading clientMsgJson in ")
                await asyncio.sleep(1)
            await clientInData.put(item=heartBeatMessage)

        # Continuous loop to send and receive data from server and client
        while True:
            # Check for serverOutData from the server task
            if not serverOutData.empty():
                # data= serverOutData.get()
                print(f"Server Data: {await serverOutData.get()}")

                pass

            # Check for clientOutData from the client task
            if not clientOutData.empty():
                clientMsg = await clientOutData.get()
                # Process speed test results if applicable
                try:
                    # result = json.loads(clientMsg)
                    print(f"Client Data: {clientMsg}")
                except Exception:
                    print(f"Client Data: {clientMsg}")

            # If previous heartbeat message has been sent, add new one to queue to be sent
            if clientInData.empty():
                await clientInData.put(item=heartBeatMessage)

            await asyncio.sleep(1)  # Adjust sleep time as needed

    except KeyboardInterrupt:
        print("Shutting down...")
        _ = serverTask.cancel()
        _ = clientTask.cancel()


if __name__ == "__main__":
    asyncio.run(main())
