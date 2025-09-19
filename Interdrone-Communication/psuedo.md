## Goal of Server

1. Receive data from clients
2. Respond with confirmation that data was received

## Goal of Client

1. Send data to servers it's connected to
2. Verify it gets confirmation that data was received

## Basic Server Functionality

1. Open socket server on ip and port listed in current drones config file (One time)
2. Listen for other drone clients to connect, and if they are listed in config, allow them to (function)
3. Listen for messages from clients (function)
4. Respond to clients messages with confirmation

## Basic Client Functionality

1. Attempt to establish connection to other drone servers (based on config).
2. Validate connection every 1s and attempt to reconnect if connection is lost
3. Send collected data deltas (what new mines have been connected)

## Questions and Complications

1. What port numbers do we want to use?
