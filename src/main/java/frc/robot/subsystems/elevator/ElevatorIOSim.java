package frc.robot.subsystems.elevator;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.DigitalSource;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import edu.wpi.first.wpilibj.simulation.EncoderSim;
import frc.robot.subsystems.elevator.ElevatorIO.ElevatorIOInputs;

public class ElevatorIOSim implements ElevatorIO {
    private final DCMotor gearbox;
    private final ElevatorSim sim;
    private final Encoder encoder;
    private final EncoderSim encoderSim;

    public ElevatorIOSim() {
        gearbox = DCMotor.getNEO(2);
        sim = new ElevatorSim(
            gearbox,
            ElevatorConstants.POSITION_CONVERSION_FACTOR,
            ElevatorConstants.ELEVATOR_MASS_KG,
            ElevatorConstants.ELEVATOR_DRUM_RADIUS,
            ElevatorConstants.ELEVATOR_MIN_HEIGHT,
            ElevatorConstants.ELEVATOR_MAX_HEIGHT,
            true,
            ElevatorConstants.ELEVATOR_MIN_HEIGHT
        );

        encoder = new Encoder(ElevatorConstants.ENCODER_A_CHANNEL, ElevatorConstants.ENCODER_B_CHANNEL);
        encoderSim = new EncoderSim(encoder);
    }

    @Override
    public void updateInputs(ElevatorIOInputs inputs) {
        inputs.lengthMeters = getLength();
        inputs.velocityMetersPerSec = getVelocity();
        inputs.appliedVolts = getVelocity() * 12;
    }
}
