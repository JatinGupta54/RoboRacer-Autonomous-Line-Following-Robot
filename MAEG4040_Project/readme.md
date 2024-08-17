# RoboRacer: Autonomous Line-Following Robot

## MAE4040 Mechatronics Project

**Department:** Mechanical & Automation Engineering  
**Institution:** The Chinese University of Hong Kong  
**Course:** MAE4040 - Mechatronics  
**Instructor:** Martin Leung  
**Year:** 2023

## Project Overview

This project is designed to enhance students' practical understanding of mechatronics principles covered in lectures and MCU workshops. The objective is to program a two-wheel drive robot car to autonomously follow a black line (~18mm in width) on a racecourse, using a line scan camera (TSL1401-DB) for navigation.

## Project Requirements

1. **Team Formation:**
   - Maximum of 3 students per team.

2. **Provided Equipment:**
   - A two-wheel drive robot car with a battery.

3. **Modification Rules:**
   - No modifications to the robot car are allowed, except for adjusting the camera's pan-tilt.

4. **Algorithm Development:**
   - Implement an algorithm to track the black line using the TSL1401-DB line scan camera.
   - Sample code is provided to assist with programming.

5. **Troubleshooting:**
   - Apply troubleshooting techniques to optimize the robot's performance.

## Project Goal

The robot car must follow the black line from the start point (START) to the endpoint (End T) on the racecourse as quickly as possible. Each team will have two trials, with the best trial time used as the final score. The time limit for each trial is two minutes, with one minute allocated for setup.

## Game Rules & Scoring

1. **No Physical Interference:**
   - The car must operate autonomously with no external interference.

2. **Restart Rules:**
   - One restart is allowed per trial, starting from the restart line in the incomplete zone.
   - A 5-second penalty is added to the final score for each restart.

3. **Scoring Zones:**
   - The racecourse is divided into five zones. Scores are assigned based on the number of completed zones.

4. **Penalty for Incomplete Zones:**
   - A 3-second penalty is added if the car fails to stop at the "End T" in zone 5 after completing the first four zones.

5. **Strategy:**
   - Skipping one zone is allowed, but the algorithm should primarily focus on line tracking.

6. **Early Submission Bonus:**
   - Teams can submit a flowchart or mind map detailing the control algorithm within two weeks of the project start. A 10-second deduction is applied to the final score for early submission.

## Program Download Notes

- **Download Instructions:**
  - Press and hold the RST and PRG buttons simultaneously, release RST first, then PRG.
  - Ensure the green LED is off and the red LED is on. If the LED is not in this state, power cycle the robot or adjust the PCB as necessary.

## Program Notes

- **Motor Control:** 
  - Use `void motorMOVE(MOT_dir actX, uint8_t spdL , uint8_t spdR )` to control the robot's speed.

- **Camera Data Handling:**
  - The program `4040_car-yr2023_cam.ino` demonstrates how to process camera data and display it on a TFT display.
  - Use `XCam_plot(&camARR[0], 0)` to clear past display data and `XCam_plot(&camARR[0], 1)` to write new data.
  - Use `XCam_2serial(&camARR[0])` to display camera data on the serial monitor.

## Submission Requirements

- **Flowchart/Mind Map:**
  - Submit a flowchart or mind map detailing how to use camera data to control robot movement and how to read line scan camera data. Early submission within two weeks of the project start can earn a 10-second time deduction.

---

For more information, please refer to the course materials provided by the instructor.
