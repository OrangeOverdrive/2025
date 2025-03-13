// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;

import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.ElevatorSubsystem;

public class RobotContainer {
    private double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond);
    private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity

    /* Setting up bindings for necessary control of the swerve drive platform */
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric().withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();
    private final ElevatorSubsystem s_elevator = new ElevatorSubsystem();

    private final Telemetry logger = new Telemetry(MaxSpeed);

    private final CommandXboxController controller1 = new CommandXboxController(0);
    private final CommandXboxController controller2 = new CommandXboxController(1);

    private double inputScaler() {
        return Math.max(1.0 - controller1.getRightTriggerAxis(), 0.15);
    }


    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();

    public RobotContainer() {
        configureBindings();
    }

    private double scaleDown(double in) {
        return Math.pow(in, 2) * Math.signum(in);
    }

    private static double deadBand(double in) {
        return Math.abs(in) < 0.05 ? 0 : in;
    }

    private void configureBindings() {

        // Note that X is defined as forward according to WPILib convention,
        // and Y is defined as to the left according to WPILib convention.
        drivetrain.setDefaultCommand(
                // Drivetrain will execute this command periodically
                drivetrain.applyRequest(() -> drive.withVelocityX(scaleDown(MathUtil.applyDeadband(controller1.getLeftY() * inputScaler(), 0.05) * MaxSpeed)) // Drive forward with negative Y (forward)
                        .withVelocityY(scaleDown(MathUtil.applyDeadband(controller1.getLeftX() * inputScaler(), 0.05) * MaxSpeed)) // Drive left with negative X (left)
                        .withRotationalRate(MathUtil.applyDeadband(-controller1.getRightX(), 0.05) * MaxAngularRate) // Drive counterclockwise with negative X (left)
                ));

        controller1.a().whileTrue(drivetrain.applyRequest(() -> brake));
        controller1.b().whileTrue(drivetrain.applyRequest(() -> point.withModuleDirection(new Rotation2d(MathUtil.applyDeadband(-controller1.getLeftY(), 0.05), MathUtil.applyDeadband(-controller1.getLeftX(), 0.05)))));

        // Run SysId routines when holding back/start and X/Y.
        // Note that each routine should be run exactly once in a single log.
        controller1.back().and(controller1.y()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
        controller1.back().and(controller1.x()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
        controller1.start().and(controller1.y()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
        controller1.start().and(controller1.x()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));

        controller2.leftTrigger().whileTrue(Commands.run(() -> s_elevator.moveUp(controller2.getLeftTriggerAxis())));
        controller2.rightTrigger().whileTrue(Commands.run(() -> s_elevator.moveDown(controller2.getRightTriggerAxis())));

        controller2.rightTrigger().or(controller2.leftTrigger()).onFalse(Commands.runOnce(s_elevator::hold));

        // Pivot
        // Up
        controller2.povUp().whileTrue(Commands.run(() -> {
            s_elevator.moveArm(-0.5);
        }));

        // Down
        controller2.povDown().whileTrue(Commands.run(() -> {
            s_elevator.moveArm(0.25);
        }));

        controller2.povUp().or(controller2.povDown()).onFalse(Commands.runOnce(s_elevator::holdPivot));

        // Intake
        controller2.leftBumper().whileTrue(Commands.run(() -> {
            s_elevator.moveIntake(0.25);
        }));

        controller2.rightBumper().whileTrue(Commands.run(() -> {
            s_elevator.moveIntake(-0.1);
        }));

        controller2.leftBumper().or(controller2.rightBumper()).onFalse(Commands.run(() -> {
            s_elevator.moveIntake(0);
        }));


        // reset the field-centric heading on left bumper press
        controller1.leftBumper().onTrue(drivetrain.runOnce(drivetrain::seedFieldCentric));

        drivetrain.registerTelemetry(logger::telemeterize);
    }

    public Command getAutonomousCommand() {
        return Commands.print("No autonomous command configured");
    }
}
