Table of Contents
=======================

* [What is CarFI?](#what-is-carfi)
* [Running in command-line mode](#running-in-command-line-mode)
* [Running via Web](#running-via-web)
* [Directory Structure](#directory-structure)

---

What is CarFI?
------

CarFI is a fault injection tool for validating the safety of autonomous vehicles. This tool is integrated with Carla and Openpilot. Carla is an open-source             simulator for autonomous driving research, and Openpilot is an open source driver assistance system. Using CarFI, you can simpulate various driving scenaios and analyze the impacts of different kinds of faults on the safety of Openpilot.


Running in command-line mode
------

To run CarFI in the command-line mode, you need to follow these steps:

1. Build Carla by running `make launch` in the home directory of CarFI
2. When the UnrealEngine UI shows up, press the play button on top of the UI so that Carla starts listening for the incoming connections on port 2000.
3. Spawn a shell within the openpilot virtual environment by runnig `pipenv shell` inside the home directory of openpilot (i.e., CarFI/openpilot).
4. Navigate to the home directory of the fault injector by running `cd tools/sim/carfi_tool` and then run carfi.py passing the appropriate command line arguments. For example, running the command `python3 carfi.py -g` in terminal means that you would like to run the fault injector in the golden-run mode (i.e., without any fault injection). For the complete list of command line arguments available for CarFI, please run `python3 carfi.py -h`.

Running via Web
------

To run CarFI via Web, you need to follow these steps:

1. Build Carla by running `make launch` in the home directory of CarFI
2. When the UnrealEngine UI shows up, press the play button on top of the UI so that Carla starts listening for the incoming connections on port 2000.
3. Spawn a shell within the openpilot virtual environment by runnig `pipenv shell` inside the home directory of openpilot (i.e., CarFI/openpilot).
4. Navigate to the home directory of the fault injector by running `cd tools/sim/carfi_tool` and then run `python3 carfi_web.py`. The Web interface will then start listening on port 5000.  
5. Open a web browser and type `http://127.0.0.1:5000` in the address bar.

Directory Structure
------
    .
    ├── agents                  # carla agents that might be needed in some FI scenarios 
    ├── analysis.py             # measures that could be used to analyze the data collected in FI experiemnts
    ├── can.py                  # CAN implementation in Openpilot
    ├── carfi.config            # FI campaign configuration
    ├── carfi.py                # core of CarFI responsible for runnig and sending commands to openpilot, parsing command-line arguments, etc.
    ├── carfi_web.py            # a Web interface for CarFI implemented on top of Flask  
    ├── fa_lib.py               # fault/attack library
    ├── launch_openpilot.sh     # script for launching Openpilot
    ├── log                     # event logs
    ├── out                     # data collected in experiemts and analysis results
    ├── sim.py                  # initializes and runs an experiment by simulating a traffic scenario based on the campaign configuration
    ├── static                  # css and js files needed by the Web interface 
    ├── templates               # Jinja templates (html files) implemented in the Web interface 
    └── util.py                 # utility functions 
    
    

