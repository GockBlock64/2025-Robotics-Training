package frc.robot.subsystems.elevator;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.wpilibj.DigitalSource;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import edu.wpi.first.wpilibj.simulation.EncoderSim;
import frc.robot.subsystems.elevator.ElevatorIO.ElevatorIOInputs;

public class ElevatorIOSim implements ElevatorIO {
    private final DCMotor gearbox; // Simulates the IRL motors we use
    private final ElevatorSim sim; // Simulates our entire elevator
    private final ProfiledPIDController pidController;

    private double setpoint = 0;

    public ElevatorIOSim() {
        gearbox = DCMotor.getNEO(2); // We use 2 NEO motors so our gearbox needs to simulate 2 NEOs
        // Don't panic because of this massive constructor, just match up the parameters one by one by hovering over "ElevatorSim" in WPILib VSCode
        // In case you can't, the parameters are (DCMotor gearbox, double gearing, double carriageMassKg, double drumRadiusMeters, double minHeightMeters, double maxHeightMeters, boolean simulateGravity, double startingHeightMeters)
        sim = new ElevatorSim(
            gearbox, // Self-explanatory
            ElevatorConstants.ELEVATOR_GEARING, // How our gearing affects motor speed
            ElevatorConstants.ELEVATOR_MASS_KG, // Self-explanatory
            ElevatorConstants.ELEVATOR_DRUM_RADIUS, // Radius of our "drum" which rotates and connects to the actual elevator (this affects its speed)
            ElevatorConstants.ELEVATOR_MIN_LENGTH, // Self-explanatory
            ElevatorConstants.ELEVATOR_MAX_LENGTH, // Self-explanatory
            true, // Yes, we want to simulate gravity
            ElevatorConstants.ELEVATOR_MIN_LENGTH // Starting elevator position
        );

        pidController = new ProfiledPIDController(ElevatorConstants.P_SIM, ElevatorConstants.I_SIM, ElevatorConstants.D_SIM, new Constraints(4, 4));
    }

    // This is where we left off but we couldn't finish it
    @Override
    public void updateInputs(ElevatorIOInputs inputs) {
        inputs.lengthMeters = getLength();
        inputs.velocityMetersPerSec = getVelocity();
        inputs.appliedVolts = 0; // we don't care about voltage in sim
        inputs.setpointMeters = setpoint;
        inputs.currentAmps = new double[]{ sim.getCurrentDrawAmps() };
        inputs.tempCelsius = new double[] { 0, 0 }; // we don't care about this in sim
    }

    @Override
    public void setVelocity(double velocity) {
        sim.setInputVoltage(velocity * 12);
    }

    @Override
    public double getLength() {
        return sim.getPositionMeters();
    }

    @Override
    public double getVelocity() {
        return sim.getVelocityMetersPerSecond();
    }

    @Override
    public void goToSetpoint(double setpoint) {
        pidController.setGoal(new State(setpoint, 0));
        setVelocity(pidController.calculate(getLength()));
    }

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
