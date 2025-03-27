package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class AlgaeSubsystem extends SubsystemBase {
    private final TalonFX kicker;
    private boolean locked = false;

    private double position() {
        return kicker.getRotorPosition().getValueAsDouble();
    }

    public AlgaeSubsystem() {
        kicker = new TalonFX(Constants.CanIDs.ALGAE_PIVOT);
        kicker.setPosition(0);
    }

    // Runs continuously!
    public void kickup(double speed) {
        if (position() <= 31) {
            kicker.set(speed);
        } else {
            kicker.set(0);
        }
    }

    // Runs continuously!
    public void kickdown(double speed) {
        if (position() >= 0) {
            kicker.set(-speed);
        } else {
            kicker.set(0);
        }
    }

    public void stopkick(double speed) {
        kicker.set(speed);
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Algae Kicker Encoder", kicker.getRotorPosition().getValueAsDouble());
    }
}
