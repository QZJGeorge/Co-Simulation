# Mcity Digital Maps Repository

Welcome to the Mcity digital maps repository! This repository hosts the digital maps for Mcity.

## Table of Contents
- [Mcity Digital Maps Repository](#mcity-digital-maps-repository)
  - [Table of Contents](#table-of-contents)
  - [SUMO Mcity Map](#sumo-mcity-map)
    - [Prerequisites](#prerequisites)
    - [File Descriptions](#file-descriptions)
    - [Running the Simulation](#running-the-simulation)

## SUMO Mcity Map

This section provides details about the SUMO map of Mcity.

### Prerequisites

1. Install [SUMO](https://sumo.dlr.de/docs/Downloads.php) on your computer.
2. Familiarize yourself with SUMO through this [quickstart guide](https://sumo.dlr.de/docs/Tutorials/quick_start.html).

### File Descriptions

The SUMO Mcity map is housed in the `sumo_map` folder and consists of three main files:

- `mcity.net.xml`: The network file for Mcity, detailing roads, intersections, and traffic lights.
- `mcity.route.xml`: The route file specifying vehicle routes in Mcity and the corresponding traffic demand.
- `mcity.sumocfg`: The configuration file outlining simulation parameters for SUMO.

### Running the Simulation

Execute the Mcity SUMO simulation with the following terminal command:

```
sumo-gui -c mcity.sumocfg
```

---

Thank you for using the Mcity digital maps repository. If you encounter any issues, feel free to raise them on the issue tracker.