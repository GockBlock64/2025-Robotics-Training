package frc.robot.subsystems.elevator;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.DigitalSource;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import edu.wpi.first.wpilibj.simulation.EncoderSim;
import frc.robot.subsystems.elevator.ElevatorIO.ElevatorIOInputs;

public class ElevatorIOSim implements ElevatorIO {
    private final DCMotor gearbox; // Simulates the IRL motors we use
    private final ElevatorSim sim; // Simulates our entire elevator

    public ElevatorIOSim() {
        gearbox = DCMotor.getNEO(2); // We use 2 NEO motors so our gearbox needs to simulate 2 NEOs
        // Don't panic because of this massive constructor, just match up the parameters one by one by hovering over "ElevatorSim" in WPILib VSCode
        // In case you can't, the parameters are (DCMotor gearbox, double gearing, double carriageMassKg, double drumRadiusMeters, double minHeightMeters, double maxHeightMeters, boolean simulateGravity, double startingHeightMeters)
        sim = new ElevatorSim(
            gearbox, // Self-explanatory
            ElevatorConstants.ELEVATOR_GEARING, // How our gearing affects motor speed
            ElevatorConstants.ELEVATOR_MASS_KG, // Self-explanatory
            ElevatorConstants.ELEVATOR_DRUM_RADIUS, // Radius of our "drum" which rotates and connects to the actual elevator (this affects its speed)
            ElevatorConstants.ELEVATOR_MIN_HEIGHT, // Self-explanatory
            ElevatorConstants.ELEVATOR_MAX_HEIGHT, // Self-explanatory
            true, // Yes, we want to simulate gravity
            ElevatorConstants.ELEVATOR_MIN_HEIGHT // Starting elevator position
        );
    }

    // This is where we left off but we couldn't finish it
    @Override
    public void updateInputs(ElevatorIOInputs inputs) {
        inputs.lengthMeters = getLength();
        inputs.velocityMetersPerSec = getVelocity();
    }
}
