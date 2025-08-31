# The Travel-and-Offload Problem for Energy-Efficient UAV-Assisted LoRaWAN Data Collection

Official code for the paper:  
**The Travel-and-Offload Problem for Energy-Efficient UAV-Assisted LoRaWAN Data Collection**  
Authors: Perinello Lorenzo, Lugigi De Giovanni, Claudio E. Palazzi
Submitted at: *IEEE Consumer Communications & Networking Conference 2026*

## Abstract
> With the increasing adoption of Unmanned Aerial Vehicles (UAVs) and their technological advancements, particularly in terms of autonomy and computational capabilities, their role as active component in wireless networks could be groundbreaking. Such potential is especially relevant in constrained environments that lack proper telecommunication infrastructure or adequate network coverage. These contexts are where the Internet of Things (IoT) has emerged as a vital technology, capable of facilitating sensing operations and data retrieval by leveraging communication protocols like LoRa for long-range, low-energy transmissions. 
In this paper, we introduce the Travel-and-Offload Problem (TOP), a new combinatorial optimization problem, formulated both as a mixed-integer nonlinear programming (MINLP) model and as a mixed-integer linear programming (MILP) model. Its primary objective is to minimize the energy consumption associated with UAV operations and data offloading tasks. Our formulation, which incorporates realistic models for energy consumption and communication, aims to optimally determine the UAV’s trajectory, hovering points and durations, as well as critical transmission parameters of LoRa-enabled sensors, while ensuring compliance with the transmission policies defined by the LoRaWAN network protocol.


## Requirements
Please see `CMakeLists.txt` file for the required  C++ standard (17), compiler used and libraries (e.g., Gurobi). For installing Gurobi and getting a license, please refer to its official [documentation](https://support.gurobi.com/hc/en-us).


## Repository structure
```
├── CMakeLists.txt        # CMake build configuration file
├── main.cpp              # Main file to run the MILP model
├── include/              # External libraries header files
├── instances/            # Instances used for the paper experiments
└── results/              # Results of the experiments
```