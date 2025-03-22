package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.generated.CommandSwerveDrivetrain;

public class Autos {
    public static Command driveForwardAuto(CommandSwerveDrivetrain driveSubsystem) {
        return Commands.sequence();
    }

    public static Command doNothingAuto() {
        return null;
    }
}
