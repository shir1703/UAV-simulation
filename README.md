
# README - Autonomous UAV Simulation  

---

## **Introduction**  
This project implements a simulation for autonomous unmanned aerial vehicles (UAVs). Each UAV flies to specified target points and circles around them when reached. The simulation dynamically processes UAV movements, adhering to physical constraints like minimum turn radius and constant speed.

---

## **Project Structure**  
### **Key Files**  
1. **`main.cpp`**  
   - Entry point of the simulation.  
   - Defines UAV behavior, movement logic, and simulation flow.  

2. **`UAV.h`**  
   - Header file for the UAV class.  
   - Contains method declarations and member variables for UAV properties and behavior.  

3. **`UAV.cpp`**  
   - Implementation of the UAV class.  
   - Includes logic for linear and circular motion, azimuth calculations, and state tracking.

4. **Input Files**  
   - `SimParams.ini`: Initializes simulation parameters such as the number of UAVs, time step, initial conditions, and simulation duration.  
   - `SimCmds.txt`: Contains flight commands for UAVs, including the target coordinates and execution times.

5. **Output Files**  
   - `UAV<n>.txt`: For each UAV, logs its state (`time, x, y, azimuth`) at every simulation step.

---

## **How to Run the Simulation**  
### **Prerequisites**  
- Visual Studio (or any C++11-compatible compiler).  
- Basic understanding of simulation concepts and C++.

### **Steps**  
1. Clone or download the repository.  
2. Open the provided Visual Studio solution file (`<solution_name>.sln`).  
3. Verify the `SimParams.ini` and `SimCmds.txt` files are in the working directory and properly configured.  
4. Build the project in Visual Studio.  
5. Run the simulation executable.  
6. Check the output files (`UAV<n>.txt`) for results.

---

## **Simulation Details**  
1. **UAV Movement**  
   - UAVs travel linearly toward a target coordinate.  
   - Upon reaching the target, UAVs switch to circular motion, orbiting the target with a defined radius.  

2. **Input File Formats**  
   - **`SimParams.ini`**  
     ```ini
     Dt = <time_step>
     N_uav = <number_of_UAVs>
     R = <turning_radius>
     X0 = <initial_x>
     Y0 = <initial_y>
     Z0 = <initial_z>
     V0 = <speed>
     Az = <azimuth_angle>
     TimeLim = <simulation_duration>
     ```  
   - **`SimCmds.txt`**  
     ```plaintext
     <time> <UAV_number> <x_target> <y_target>
     ```  

3. **Output File Format**  
   - Each line represents the UAV's state at a simulation step:  
     ```plaintext
     <time> <x> <y> <azimuth>
     ```

4. **Key Assumptions**  
   - All UAVs fly at constant speed (`V0`).  
   - Each UAV has a minimum turn radius (`R`).  
   - Time progresses in discrete steps (`Dt`).

---

## **Code Highlights**  
- **Linear Movement**: Computes next position based on speed and azimuth angle.  
- **Circular Movement**: UAVs orbit around target points upon arrival.  
- **Dynamic Target Updates**: UAVs adjust paths based on commands from `SimCmds.txt`.  
- **Output**: Logs precise UAV states with two decimal places for accuracy.  

---

## **Visualization**  
# README - Autonomous UAV Simulation  

---

## **Introduction**  
This project implements a simulation for autonomous unmanned aerial vehicles (UAVs). Each UAV flies to specified target points and circles around them when reached. The simulation dynamically processes UAV movements, adhering to physical constraints like minimum turn radius and constant speed.

---

## **Project Structure**  
### **Key Files**  
1. **`main.cpp`**  
   - Entry point of the simulation.  
   - Defines UAV behavior, movement logic, and simulation flow.  

2. **`UAV.h`**  
   - Header file for the UAV class.  
   - Contains method declarations and member variables for UAV properties and behavior.  

3. **`UAV.cpp`**  
   - Implementation of the UAV class.  
   - Includes logic for linear and circular motion, azimuth calculations, and state tracking.

4. **Input Files**  
   - `SimParams.ini`: Initializes simulation parameters such as the number of UAVs, time step, initial conditions, and simulation duration.  
   - `SimCmds.txt`: Contains flight commands for UAVs, including the target coordinates and execution times.

5. **Output Files**  
   - `UAV<n>.txt`: For each UAV, logs its state (`time, x, y, azimuth`) at every simulation step.

---

## **How to Run the Simulation**  
### **Prerequisites**  
- Visual Studio (or any C++11-compatible compiler).  
- Basic understanding of simulation concepts and C++.

### **Steps**  
1. Clone or download the repository.  
2. Open the provided Visual Studio solution file (`<solution_name>.sln`).  
3. Verify the `SimParams.ini` and `SimCmds.txt` files are in the working directory and properly configured.  
4. Build the project in Visual Studio.  
5. Run the simulation executable.  
6. Check the output files (`UAV<n>.txt`) for results.

---

## **Simulation Details**  
1. **UAV Movement**  
   - UAVs travel linearly toward a target coordinate.  
   - Upon reaching the target, UAVs switch to circular motion, orbiting the target with a defined radius.  

2. **Input File Formats**  
   - **`SimParams.ini`**  
     ```ini
     Dt = <time_step>
     N_uav = <number_of_UAVs>
     R = <turning_radius>
     X0 = <initial_x>
     Y0 = <initial_y>
     Z0 = <initial_z>
     V0 = <speed>
     Az = <azimuth_angle>
     TimeLim = <simulation_duration>
     ```  
   - **`SimCmds.txt`**  
     ```plaintext
     <time> <UAV_number> <x_target> <y_target>
     ```  

3. **Output File Format**  
   - Each line represents the UAV's state at a simulation step:  
     ```plaintext
     <time> <x> <y> <azimuth>
     ```

4. **Key Assumptions**  
   - All UAVs fly at constant speed (`V0`).  
   - Each UAV has a minimum turn radius (`R`).  
   - Time progresses in discrete steps (`Dt`).

---

## **Code Highlights**  
- **Linear Movement**: Computes next position based on speed and azimuth angle.  
- **Circular Movement**: UAVs orbit around target points upon arrival.  
- **Dynamic Target Updates**: UAVs adjust paths based on commands from `SimCmds.txt`.  
- **Output**: Logs precise UAV states with two decimal places for accuracy.  

---

## **Visualization**  
To visualize flight paths:  
1. Use Python, MATLAB, or Excel to plot results from `UAV<n>.txt`.  
2. Example Python script for plotting:  
   ```python
   import matplotlib.pyplot as plt
   import numpy as np


   def read_path_data(filename):
       times = []
       x_coords = []
       y_coords = []
       azimuths = []

       with open(filename, 'r') as file:
           for line in file:
               # Skip empty lines
               if not line.strip():
                   continue

               # Check if the line contains commas or spaces to decide how to parse it
               if ',' in line:
                   # Parse the line with commas (old format)
                   parts = line.split(',')
                   times.append(float(parts[0].split(':')[1]))
                   x_coords.append(float(parts[1].split(':')[1]))
                   y_coords.append(float(parts[2].split(':')[1]))
                   azimuths.append(float(parts[3].split(':')[1]))
               else:
                   # Parse the line with spaces (new format)
                   parts = line.split()
                   times.append(float(parts[0]))
                   x_coords.append(float(parts[1]))
                   y_coords.append(float(parts[2]))
                   azimuths.append(float(parts[3]))

       return np.array(times), np.array(x_coords), np.array(y_coords), np.array(azimuths)


   def plot_points_only(filename):
       times, x_coords, y_coords, _ = read_path_data(filename)

       plt.figure(figsize=(10, 8))

       scatter = plt.scatter(x_coords, y_coords, c=times, cmap='viridis', s=100)

       # Highlight start and end points
       plt.scatter(x_coords[0], y_coords[0], color='green', s=200, label='Start')
       plt.scatter(x_coords[-1], y_coords[-1], color='red', s=200, label='End')

       # Graph settings
       plt.grid(True)
       plt.xlabel('X')
       plt.ylabel('Y')
       plt.title('Path Visualization for UAV1.txt')
       plt.colorbar(scatter, label='Time')
       plt.legend()

       plt.axis('equal')

       plt.show()


   # Use the function
   if __name__ == "__main__":
       plot_points_only("path_data.txt")  # replace with output file

