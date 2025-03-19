package frc.robot.subsystems;


import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ClimberSubsystem extends SubsystemBase {
    private final TalonFX winch;
    private final DigitalInput limitSwitch;
    private boolean moving = false;

    public ClimberSubsystem() {
        winch = new TalonFX(Constants.CanIDs.WINCH);
        limitSwitch = new DigitalInput(0);
    }

    public void climb() {
        moving = true;
    }

    @Override
    public void periodic() {
        if (moving && !limitSwitch.get()) {
            winch.set(1);
        } else {
            winch.set(0);
        }
    }
}

