package frc.robot.subsystems.elevator;

import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Elevator extends SubsystemBase {
    private final ElevatorIOInputsAutoLogged inputs = new ElevatorIOInputsAutoLogged();

    private final ElevatorIO io;

    private MechanismLigament2d elevatorMechanism;

    public Elevator(ElevatorIO io) {
        this.io = io;
        elevatorMechanism = new MechanismLigament2d(getName(), ElevatorConstants.ELEVATOR_MIN_LENGTH, 0, 5, new Color8Bit(Color.kAqua));
    }

    public void setVelocity(double velocity) {
        if(io.getLength() > ElevatorConstants.ELEVATOR_MAX_LENGTH && velocity > 0) {
            velocity = 0;
        }
        else if(io.getLength() < ElevatorConstants.ELEVATOR_MIN_LENGTH && velocity < 0) {
            velocity = 0;
        }
        io.setVelocity(velocity);
    }

    public void setMechanism(MechanismLigament2d mechanism) {
        elevatorMechanism = mechanism;
    }

    public MechanismLigament2d getMechanism() {
        return elevatorMechanism;
    }

    public Command moveToSetpoint(double setpoint) {
        return new FunctionalCommand(
            () -> {},
            () -> io.goToSetpoint(setpoint),
            (interrupted) -> io.setVelocity(0),
            () -> io.atSetpoint(),
            this
        );
    }

    public Command moveToSetpointForever(double setpoint) {
        return new FunctionalCommand(
            () -> {},
            () -> io.goToSetpoint(setpoint),
            (interrupted) -> io.setVelocity(0),
            () -> false,
            this
        );
    }
}
