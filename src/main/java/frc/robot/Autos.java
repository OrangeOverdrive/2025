package frc.robot;

import com.ctre.phoenix6.swerve.SwerveModule;
import com.ctre.phoenix6.swerve.SwerveRequest;
import edu.wpi.first.wpilibj2.command.*;
import frc.robot.generated.CommandSwerveDrivetrain;
import frc.robot.generated.TunerConstants;

import static edu.wpi.first.units.Units.*;

public class Autos {
//    private double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond);
//    public SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric().withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
//            .withDriveRequestType(SwerveModule.DriveRequestType.OpenLoopVoltage);

    public static Command driveForwardAuto(CommandSwerveDrivetrain driveSubsystem) {
        double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond);
        double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond);
        SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric().withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
                .withDriveRequestType(SwerveModule.DriveRequestType.OpenLoopVoltage);

        return Commands.sequence(
                new WaitCommand(3),
                new ParallelDeadlineGroup(
                        new WaitCommand(1),
                        new RunCommand(() -> drive.withVelocityX(0.5))
                ),
                new RunCommand(() -> drive.withVelocityX(0))
        );
    }

    public static Command driveAndScoreCoralAuto(CommandSwerveDrivetrain driveSubsystem) {
        return Commands.sequence(
                driveForwardAuto(driveSubsystem)
                // move elevator to point
                // run coral wheels
        );
    }
}
