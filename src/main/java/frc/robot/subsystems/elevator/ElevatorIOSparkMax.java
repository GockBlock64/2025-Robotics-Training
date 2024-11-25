package frc.robot.subsystems.elevator;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.SparkAbsoluteEncoder.Type;

import frc.robot.Constants;
import frc.robot.Constants.ElectricalLayout;

public class ElevatorIOSparkMax implements ElevatorIO {
    // Motors and encoders
    private CANSparkMax elevatorMotor, elevatorFollower;
    private AbsoluteEncoder encoder;

    private double setpoint = 0;

    public ElevatorIOSparkMax() {
        elevatorMotor = new CANSparkMax(ElectricalLayout.ELEVATOR_MOTOR_ID, MotorType.kBrushless);
        elevatorFollower = new CANSparkMax(ElectricalLayout.ELEVATOR_FOLLOWER_ID, MotorType.kBrushless);

        elevatorMotor.restoreFactoryDefaults();
        elevatorFollower.restoreFactoryDefaults();

        elevatorFollower.follow(elevatorMotor, true);

        elevatorMotor.setIdleMode(IdleMode.kBrake);
        elevatorFollower.setIdleMode(IdleMode.kBrake);

        elevatorMotor.enableVoltageCompensation(12.0);

        elevatorMotor.setSmartCurrentLimit(Constants.NEO_CURRENT_LIMIT);
        elevatorFollower.setSmartCurrentLimit(Constants.NEO_CURRENT_LIMIT);

        elevatorMotor.burnFlash();
        elevatorFollower.burnFlash();

        // Encoder setup
        encoder = elevatorMotor.getAbsoluteEncoder(Type.kDutyCycle);

        encoder.setPositionConversionFactor(ElevatorConstants.POSITION_CONVERSION_FACTOR);
        encoder.setVelocityConversionFactor(ElevatorConstants.POSITION_CONVERSION_FACTOR / 60.0);
    }
    
    @Override
    public void updateInputs(ElevatorIOInputs inputs) {
        inputs.lengthMeters = getLength();
        inputs.velocityMetersPerSec = getVelocity();
        inputs.appliedVolts = elevatorMotor.getBusVoltage() * elevatorMotor.getAppliedOutput();
        inputs.setpointMeters = setpoint;
        inputs.currentAmps = new double[] { elevatorMotor.getOutputCurrent(), elevatorFollower.getOutputCurrent() };
        inputs.tempCelsius = new double[] { elevatorMotor.getMotorTemperature(), elevatorFollower.getMotorTemperature() };
    }

    @Override
    public void setVelocity(double velocity) {
        elevatorMotor.set(velocity);
    }

    @Override
    public double getLength() {
        return encoder.getPosition() + ElevatorConstants.ELEVATOR_OFFSET;
    }

    @Override
    public double getVelocity() {
        return encoder.getVelocity();
    }

    @Override
    public void goToSetpoint(double setpoint) {
        this.setpoint = setpoint;
        if(atSetpoint()) {
        }
        else if(getLength() < setpoint) {
            setVelocity(ElevatorConstants.ELEVATOR_SPEED);
        }
        else {
            setVelocity(-ElevatorConstants.ELEVATOR_SPEED);
        }
    }

    @Override
    public boolean atSetpoint() {
        return Math.abs(setpoint - getLength()) < ElevatorConstants.SETPOINT_TOLERANCE;
    }
}
