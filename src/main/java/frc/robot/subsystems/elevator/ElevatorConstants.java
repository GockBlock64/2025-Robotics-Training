package frc.robot.subsystems.elevator;

public class ElevatorConstants {
    public final static double POSITION_CONVERSION_FACTOR = 1.0; // CHANGE ASAP
    public final static double ELEVATOR_OFFSET = 0;
    public final static double SETPOINT_TOLERANCE = 0.005;

    public final static double ELEVATOR_SPEED = 0.7;

    public final static double ELEVATOR_GEARING = 1;
    public final static double ELEVATOR_DRUM_RADIUS = 0.1;
    public final static double ELEVATOR_MASS_KG = 1.257;
    public final static double ELEVATOR_MIN_LENGTH = 0.3;
    public final static double ELEVATOR_MAX_LENGTH = 1;

    public final static int ENCODER_A_CHANNEL = 1;
    public final static int ENCODER_B_CHANNEL = 2;

    // PID Constants
    public final static double P_REAL = 0;
    public final static double I_REAL = 0;
    public final static double D_REAL = 0;

    public final static double P_SIM = 0;
    public final static double I_SIM = 0;
    public final static double D_SIM = 0;
}
