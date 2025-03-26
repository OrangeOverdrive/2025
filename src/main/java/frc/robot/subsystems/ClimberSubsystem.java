package frc.robot.subsystems;


import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ClimberSubsystem extends SubsystemBase {
    private final TalonFX winch;
    private final DigitalInput limitSwitch;
    private boolean moving = false;

    public ClimberSubsystem() {
        winch = new TalonFX(Constants.CanIDs.WINCH);
        winch.setPosition(0);
        limitSwitch = new DigitalInput(0);
    }

    public void climb(double speed) {
        System.out.println("ok we're climbing");
        winch.set(speed);
    }

    public void stop() {
        System.out.println("ok we're not climbing");
        winch.set(0);
    }

    @Override
    public void periodic() {
//        if (moving && !limitSwitch.get()) {
//            System.out.println("ok we're climbing");
//            winch.set(1);
//        } else {
//            System.out.println("ok we're not climbing");
//            winch.set(0);
//        }
    }
}

