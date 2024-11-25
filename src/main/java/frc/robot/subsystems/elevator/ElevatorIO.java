package frc.robot.subsystems.elevator;

import org.littletonrobotics.junction.AutoLog;

public interface ElevatorIO {
    @AutoLog
    public static class ElevatorIOInputs {
        public double lengthMeters = 0.0;
        public double velocityMetersPerSec = 0.0;
        public double appliedVolts = 0.0;
        public double setpointMeters = 0.0;
        public double[] currentAmps = new double[] {};
        public double[] tempCelsius = new double[] {};
    }

    public default void updateInputs(ElevatorIOInputs inputs) {}

    // Speed ranges from -1 to 1
    public default void setVelocity(double velocity) {}

    public default double getLength() { return 0; }

    public default double getVelocity() { return 0; }

    public default void goToSetpoint(double setpoint) {}

    public default boolean atSetpoint() { return false; }
}
