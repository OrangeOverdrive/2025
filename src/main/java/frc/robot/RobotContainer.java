// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;

import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;

public class RobotContainer {
    private double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond);
    private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity

    /* Setting up bindings for necessary control of the swerve drive platform */
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric().withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();

    private final Telemetry logger = new Telemetry(MaxSpeed);

    private final CommandXboxController joystick = new CommandXboxController(0);

    private double inputScaler() {
        return Math.max(1.0 - joystick.getRightTriggerAxis(), 0.15);
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
                drivetrain.applyRequest(() -> drive.withVelocityX(scaleDown(MathUtil.applyDeadband(joystick.getLeftY() * inputScaler(), 0.05) * MaxSpeed)) // Drive forward with negative Y (forward)
                        .withVelocityY(scaleDown(MathUtil.applyDeadband(joystick.getLeftX() * inputScaler(), 0.05) * MaxSpeed)) // Drive left with negative X (left)
                        .withRotationalRate(MathUtil.applyDeadband(-joystick.getRightX(), 0.05) * MaxAngularRate) // Drive counterclockwise with negative X (left)
                ));

        joystick.a().whileTrue(drivetrain.applyRequest(() -> brake));
        joystick.b().whileTrue(drivetrain.applyRequest(() -> point.withModuleDirection(new Rotation2d(MathUtil.applyDeadband(-joystick.getLeftY(), 0.05), MathUtil.applyDeadband(-joystick.getLeftX(), 0.05)))));

        // Run SysId routines when holding back/start and X/Y.
        // Note that each routine should be run exactly once in a single log.
        joystick.back().and(joystick.y()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
        joystick.back().and(joystick.x()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
        joystick.start().and(joystick.y()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
        joystick.start().and(joystick.x()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));


        // reset the field-centric heading on left bumper press
        joystick.leftBumper().onTrue(drivetrain.runOnce(drivetrain::seedFieldCentric));

        SparkMax leftElevator = new SparkMax(21, SparkLowLevel.MotorType.kBrushless);
        SparkMax rightElevator = new SparkMax(22, SparkLowLevel.MotorType.kBrushless);

        joystick.x().whileTrue(Commands.runOnce(() -> {
            if (joystick.getLeftTriggerAxis() > 0) {
                leftElevator.set(-0.05);
                rightElevator.set(0.05);
            } else {
                leftElevator.set(0.05);
                rightElevator.set(-0.5);
            }
        }));

        joystick.x().whileFalse(Commands.runOnce(() -> {
            leftElevator.set(0);
            rightElevator.set(0);
        }));

        drivetrain.registerTelemetry(logger::telemeterize);
    }

    public Command getAutonomousCommand() {
        return Commands.print("No autonomous command configured");
    }
}
