package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.config.BaseConfig;
import com.revrobotics.spark.*;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.config.ClosedLoopConfig;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ElevatorSubsystem extends SubsystemBase {
    private SparkMax leftMotor;
    private SparkMax rightMotor;
    private double startingPosition;
    private final double DISTANCE_TO_FULL = 1;
    private final double VALUE_P = 0.001;

    private final double SET_POINT = 5;


    public ElevatorSubsystem() {
        leftMotor = new SparkMax(21, SparkLowLevel.MotorType.kBrushless);
        rightMotor = new SparkMax(22, SparkLowLevel.MotorType.kBrushless);

        SparkMaxConfig config = new SparkMaxConfig();
        config.idleMode(SparkBaseConfig.IdleMode.kBrake);
        config.encoder.positionConversionFactor(1000).velocityConversionFactor(1000);

        double kP = 0.001;
        double kI = 0;
        double kD = 0;

        SmartDashboard.putNumber("Elevator P", kP);
        SmartDashboard.putNumber("Elevator I", kI);
        SmartDashboard.putNumber("Elevator D", kD);

        config.closedLoop.feedbackSensor(ClosedLoopConfig.FeedbackSensor.kPrimaryEncoder).pid(kP, kI, kD);

        leftMotor.configure(config, SparkBase.ResetMode.kResetSafeParameters, SparkBase.PersistMode.kPersistParameters);
        rightMotor.configure(config, SparkBase.ResetMode.kResetSafeParameters, SparkBase.PersistMode.kPersistParameters);
    }

    public void setCurrentPosition() {

    }

    public void brake() {

    }

    public void moveUp() {
        leftMotor.set(0.25);
        rightMotor.set(-0.25);
    }

    public void moveDown() {
        leftMotor.set(-0.25);
        rightMotor.set(0.25);
    }

    public void stop() {
        leftMotor.set(0);
        rightMotor.set(0);
    }
}
