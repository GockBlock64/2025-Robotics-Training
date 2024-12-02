package frc.robot.subsystems.elevator;

// The elevator subsystem is like an arm that can extend and retract
// This IO file defines a bunch of low-level functions that can do things to the elevator or get information from the elevator
// However, there is no actual content in these functions
// In ElevatorIOSim and ElevatorIOSparkMax, the actual content is written for both simulation and real-life hardware

import org.littletonrobotics.junction.AutoLog;

public interface ElevatorIO {
    // This is a class of "inputs", or information about the elevator, like its length, velocity, etc.
    // Basically the only reason it exists is for logging, because now our programs can just look here and constantly log the values of these inputs
    @AutoLog
    public static class ElevatorIOInputs {
        public double lengthMeters = 0.0; // Length of the elevator in meters
        public double velocityMetersPerSec = 0.0; // Velocity of the elevator in meters per second
        public double appliedVolts = 0.0; // Voltage applied to the elevator
        public double setpointMeters = 0.0; // This setpoint basically tells you how long the elevator *wants* to be
        public double[] currentAmps = new double[] {}; // Current flowing through each of the elevator motors
        public double[] tempCelsius = new double[] {}; // Temperature of each of the motors
    }

    // We are going to run this function periodically to update the inputs in real-time so they match the actual elevator's info
    public default void updateInputs(ElevatorIOInputs inputs) {}

    // Sets the velocity of the elevator motors
    // Velocity ranges from -1 (full speed, backwards) to 1 (full speed, forwards)
    public default void setVelocity(double velocity) {}

    // Returns the length of the elevator, in meters
    public default double getLength() { return 0; }

    // Returns the velocity of the elevator, in meters
    public default double getVelocity() { return 0; }

    // Calculates motor speed needed to go to a certain setpoint
    public default void goToSetpoint(double setpoint) {}

    // Returns true if the arm is at the setpoint, false if it is not
    public default boolean atSetpoint() { return false; }

    // PID constant setters
    public default void setP(double p) {}
    public default void setI(double i) {}
    public default void setD(double D) {}

    // PID constant getters
    public default double getP() { return 0; }
    public default double getI() { return 0; }
    public default double getD() { return 0; }
}
