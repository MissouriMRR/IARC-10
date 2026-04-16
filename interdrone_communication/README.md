# Team 5 Interdrone Communication

Missouri S&T Multirotor Drone Design Team's code for inter-drone communication for the 2026 IARC competition.

## Enviorment Setup:

The steps below document how to set up your environment to work with this repo and perform local testing

### Required Software:
- **uv** 0.9.5
- Git

### How to set up enviorment with git and uv:
1. Clone the repo <code>git clone https://github.com/MissouriMRR/IARC-10.git</code>
2. cd into, <code>cd Interdrone-Communication</code>
3. Run <code>uv python install 3.12</code>
4. Run <code>uv sync</code>

## How to peform tests with the code:
1. Edit **config.json** to match your desired settings. If you wish to peform the test on your local network use 127.0.0.* network and use the provided ports. If you wish to peform it over an external network, update the IP in config to match your computers ip.
2. After the config file is set up, simply run <code>uv run main.py #</code> (with number corresponding to which drone you are). This will start the server and client.

Tip:
Use the split terminal button in VScode to run multiple servers and clients at the same time.

