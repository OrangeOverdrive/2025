package frc.robot.subsystems;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ElevatorSubsystem extends SubsystemBase {
    private TalonFX leftMotor;
    private TalonFX rightMotor;

    public ElevatorSubsystem() {
        leftMotor = new TalonFX(21);
        rightMotor = new TalonFX(22);
    }

    public void moveUp(double speed) {
        leftMotor.set(speed);
        rightMotor.set(-speed);
    }

    public void moveDown(double speed) {
        leftMotor.set(-speed);
        rightMotor.set(speed);
    }

    public void stop() {
        leftMotor.stopMotor();
        rightMotor.stopMotor();
    }

    public Angle getPosition() {
        return leftMotor.getPosition().getValue();
    }
}
