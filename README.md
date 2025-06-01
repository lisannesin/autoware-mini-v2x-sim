# V2X Integration for Autoware Mini

## Overview
This folder contains four Python scripts that simulate and filter V2X (Vehicle-to-Everything) messages alongside an Autoware Mini setup:
- **v2xVehicle.py** : publishes simulated vehicle data  
- **v2xPedestrian.py** : publishes simulated pedestrian data  
- **v2xTrafficLights.py** : publishes simulated traffic-light status  
- **v2xProcessing.py** : subscribes to all three and outputs a filtered/combined V2X stream  

## Compatibility with Autoware Mini


This V2X package is designed to integrate directly with an [Autoware Mini](https://github.com/UT-ADL/autoware_mini). Once Autoware Mini is running (per the instructions in its README), you can launch the four V2X scripts (`v2xVehicle.py`, `v2xPedestrian.py`, `v2xTrafficLights.py`, `v2xProcessing.py`) in parallel so that Autoware Mini receives a consolidated “filtered” V2X stream.

## Example: Testing in Bad Weather with CARLA & Autoware Mini

This example shows how to run a “Hidden Pedestrian” scenario in CARLA, adjust weather settings, and then observe V2X behavior (pedestrian and traffic‐light messages) under poor visibility.

The entire `v2x` folder (containing all four scripts and this README) should live inside the `nodes/` directory of your Autoware Mini workspace.

1. **Edit the scenario weather**  
   - Open `PedestrianCrossingFront.xosc` (located in your CARLA scenarios folder).  
   - Under the `<Weather>` section, adjust values such as `visualRange` (in meters) and `precipitationIntensity` (0.0–1.0) to simulate fog, rain, etc.  
     ```xml
     <Weather>
       <!-- Example: heavy rain + low visibility -->
       <VisualRange>0</VisualRange>
       <PrecipitationIntensity>0.9</PrecipitationIntensity>
       <!-- ...other parameters... -->
     </Weather>
     ```

2. **Launch CARLA**  
   ```bash
   $CARLA_ROOT/CarlaUE4.sh
   
3. **Start Autoware Mini with Scenario Runner**  
   ```bash
   roslaunch autoware_mini start_carla.launch use_scenario_runner:=true
   
4. **Select the “PedestrianCrossingFront” Scenario**
5. **Run the V2X Publisher Nodes in two separate terminals**

   ```bash
   python3 v2xPedestrian.py
   python3 v2xTrafficLights.py
   
6. **Run the V2X Processing**
    ```bash
   python3 v2xProcessing.py
   
Result:
You’ll see that—even in heavy rain or fog—the V2X‐augmented triggers an alert/braking event.
