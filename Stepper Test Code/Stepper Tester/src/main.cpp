// ------------------------------
// File: main.cpp (or my_sketch_name.ino)
// ------------------------------

// You only need to include the header file
#include "KinematicSolver.h"

// Example Kinematic Parameters (DH parameters for a 6DOF arm - replace with your actual values)
// DH table parameters: a, d, alpha, theta
double a_params[DOF] =     {0,50,0,0,0,0}; // link lengths (a_i)
double d_params[DOF] =     {50, 0, 0, 50, 0, 0};   // link offsets (d_i)
double alpha_params[DOF] = {90, 0, 90, 90, 90, 0}; // twist angles (alpha_i) in degrees
double theta_params[DOF] = {0, 0, 90, -180, 90, 0};    // initial joint angles (theta_i) in degrees

 KinematicSolver robot(a_params, d_params, alpha_params, theta_params);

// Convert degrees to radians for initialization
void degToRad(double *arr, int size) {
    for (int i = 0; i < size; i++) {
        arr[i] *= DEG_TO_RAD;
    }
}

// Convert all DH angular parameters to radians before passing them to the constructor
void setup() {
    Serial.begin(115200);
    
    degToRad(alpha_params, DOF);
    degToRad(theta_params, DOF);
    
    // 1. Create an instance of your KinematicSolver class
   

    // 2. Define the target end-effector pose (X, Y, Z, Rx, Ry, Rz)
    // Example: move to (150, 0, 150) with 0, 0, 0 orientation
    double targetPose[DOF] = {50, 0, 50, 180, 0, 0}; 

    // 3. Array to hold the calculated joint angles (qDeg)
    double jointAngles[DOF] = {0}; 

    // 4. Call the IK function!
    // The linker will find the actual code in KinematicSolver.cpp
    robot.IK(targetPose, true, jointAngles); // true = elbow up

    Serial.println("Calculated Joint Angles (Degrees):");
    for (int i = 0; i < DOF; i++) {
        Serial.printf("Joint %d: %.2f degrees\n", i + 1, jointAngles[i]);
    }
}

void loop() {
     
    // // --- 1. Define Target Pose ---
    // double targetPose[DOF] = {150.0, 0.0, 150.0, 0.0, 0.0, 0.0}; 
    // double jointAngles[DOF] = {0}; 
    // bool elbowUp = true;
    
    // // --- 2. Start Timer ---
    // // Record the time just before the function call
    // unsigned long startTime = micros();

    // // --- 3. Execute the IK function ---
    // robot.IK(targetPose, elbowUp, jointAngles); 

    // // --- 4. Stop Timer ---
    // // Record the time immediately after the function returns
    // unsigned long endTime = micros();
    
    // // --- 5. Calculate and Print Runtime ---
    // unsigned long duration_us = endTime - startTime;

    // Serial.print("IK Execution Time: ");
    // Serial.print(duration_us);
    // Serial.println(" microseconds (µs)");
    
    // // Optional: Print the results to verify the calculation
    // // Serial.print("J1: "); Serial.println(jointAngles[0]);
    
    // // Wait before running again to avoid overwhelming the serial monitor
    delay(100);
}