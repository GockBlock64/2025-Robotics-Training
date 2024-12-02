package frc.robot.subsystems.elevator;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.SparkAbsoluteEncoder.Type;

import frc.robot.Constants;
import frc.robot.Constants.ElectricalLayout;

// In elevatorIOSparkMax, we actually implement the functionalities of the functions defined in ElevatorIO for real-life hardware
public class ElevatorIOSparkMax implements ElevatorIO {
    // We need a way of representing our motors and encoders in the code
    // CANSparkMax represents a motor
    // AbsoluteEncoder represents an encoder (a sensor that tells you the motor's position and velocity)
    private CANSparkMax elevatorMotor, elevatorFollower;
    private AbsoluteEncoder encoder;
    private SparkPIDController pidController;

    private double setpoint = 0; // The setpoint mentioned in ElevatorIO

    public ElevatorIOSparkMax() {
        // Set up elevator motors
        elevatorMotor = new CANSparkMax(ElectricalLayout.ELEVATOR_MOTOR_ID, MotorType.kBrushless); // Main motor
        elevatorFollower = new CANSparkMax(ElectricalLayout.ELEVATOR_FOLLOWER_ID, MotorType.kBrushless); // This motor will follow the main motor

        // Restore motor settings to factory default
        elevatorMotor.restoreFactoryDefaults();
        elevatorFollower.restoreFactoryDefaults();

        elevatorFollower.follow(elevatorMotor, true); // Now whatever you do to elevatorMotor will copy to elevatorFollower

        // We need the motors to resist any movement when they're idle
        elevatorMotor.setIdleMode(IdleMode.kBrake);
        elevatorFollower.setIdleMode(IdleMode.kBrake);

        // Our motors run at 12V
        elevatorMotor.enableVoltageCompensation(12.0);

        // Set a current limit so we don't burn the motors or do anything unspeakable to them
        elevatorMotor.setSmartCurrentLimit(Constants.NEO_CURRENT_LIMIT);
        elevatorFollower.setSmartCurrentLimit(Constants.NEO_CURRENT_LIMIT);

        // Actually writes the settings to the motors
        elevatorMotor.burnFlash();
        elevatorFollower.burnFlash();

        // Encoder setup
        encoder = elevatorMotor.getAbsoluteEncoder(Type.kDutyCycle);

        encoder.setPositionConversionFactor(ElevatorConstants.POSITION_CONVERSION_FACTOR); // Changes position units from motor rotations to meters
        encoder.setVelocityConversionFactor(ElevatorConstants.POSITION_CONVERSION_FACTOR / 60.0); // Changes velocity units from RPM to meters/sec

        // PID setup
        pidController = elevatorMotor.getPIDController();

        pidController.setP(ElevatorConstants.P_REAL);
        pidController.setI(ElevatorConstants.I_REAL);
        pidController.setD(ElevatorConstants.D_REAL);
    }

    // Update the inputs (mentioned in ElevatorIO) in real time
    @Override
    public void updateInputs(ElevatorIOInputs inputs) {
        inputs.lengthMeters = getLength(); // getLength() returns length in meters
        inputs.velocityMetersPerSec = getVelocity(); // getVelocity() returns velocity in meters/sec
        inputs.appliedVolts = elevatorMotor.getBusVoltage() * elevatorMotor.getAppliedOutput(); // getBusVoltage() returns the maximum voltage of the motors (12), getAppliedOutput() returns the percentage of that voltage we're using
        inputs.setpointMeters = setpoint; // setpoint tracks our setpoint
        inputs.currentAmps = new double[] { elevatorMotor.getOutputCurrent(), elevatorFollower.getOutputCurrent() }; // An array containing the currents of both motors
        inputs.tempCelsius = new double[] { elevatorMotor.getMotorTemperature(), elevatorFollower.getMotorTemperature() }; // An array containing the temperatures of both motors
    }

    @Override
    public void setVelocity(double velocity) {
        elevatorMotor.set(velocity); // Good code needs no comments because it explains itself
    }

    @Override
    public double getLength() {
        return encoder.getPosition() + ElevatorConstants.ELEVATOR_OFFSET; // Same here
    }

    @Override
    public double getVelocity() {
        return encoder.getVelocity(); // Same here
    }

    // This one is a bit trickier
    // If elevator is at setpoint, don't do anything
    // Else if elevator is shorter than setpoint, set velocity to something positive (extend the elevator)
    // Else, set velocity to something negative (retract the elevator)
    // This is called Bang-Bang Control
    // PID is better, we will explain how to code PID instead later
    @Override
    public void goToSetpoint(double setpoint) {
        pidController.setReference(setpoint, ControlType.kPosition);
    }

    // If the difference between the length of the arm and the setpoint is small enough, then you're at the setpoint
    @Override
    public boolean atSetpoint() {
        return Math.abs(setpoint - getLength()) < ElevatorConstants.SETPOINT_TOLERANCE;
    }

    // PID constant setters
    @Override
    public void setP(double p) {
        pidController.setP(p);
    }
    
    @Override
    public void setI(double i) {
        pidController.setI(i);
    }
    
    @Override
    public void setD(double d) {
        pidController.setD(d);
    }

    // PID constant getters
    @Override
    public double getP() {
        return pidController.getP();
    }
    
    @Override
    public double getI() {
        return pidController.getI();
    }
    
    @Override
    public double getD() {
        return pidController.getD();
    }
}
