package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.config.BaseConfig;
import com.revrobotics.spark.*;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.config.ClosedLoopConfig;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ElevatorSubsystem extends SubsystemBase {
    private SparkMax leftMotor;
    private SparkMax rightMotor;
    private double startingPosition;
    private final double DISTANCE_TO_FULL = 1;
    private final double VALUE_P = 0.001;
    private final PIDController pid;

    private final double VALUE_KS = 0;
    private final double VALUE_KG = 0.31;
    private final double VALUE_KV = 3.07;
    private final double VALUE_KA = 0.03;

    private double STOP_POS;

    private final ElevatorFeedforward FEED_FORWARD = new ElevatorFeedforward(VALUE_KS, VALUE_KG, VALUE_KV, VALUE_KA);
    private double SET_POINT = 0;

    private double ZERO_POINT;

    private double leftMotorPrevPos = 0;
    private double rightMotorPrevPos = 0;


    public ElevatorSubsystem() {
        leftMotor = new SparkMax(21, SparkLowLevel.MotorType.kBrushless);
        rightMotor = new SparkMax(22, SparkLowLevel.MotorType.kBrushless);

        ZERO_POINT = leftMotor.getEncoder().getPosition();

        SparkMaxConfig config = new SparkMaxConfig();
        config.idleMode(SparkBaseConfig.IdleMode.kBrake);
//        config.encoder.positionConversionFactor(1000).velocityConversionFactor(1000);

        double kP = 0.001;
        double kI = 0;
        double kD = 0;

        pid = new PIDController(kP, kI, kD);

        SmartDashboard.putNumber("Elevator P", kP);
        SmartDashboard.putNumber("Elevator I", kI);
        SmartDashboard.putNumber("Elevator D", kD);

        SmartDashboard.putNumber("Left Encoder", leftMotor.getEncoder().getPosition());
        SmartDashboard.putNumber("Right Encoder", rightMotor.getEncoder().getPosition());

        SmartDashboard.putNumber("KS", VALUE_KS);
        SmartDashboard.putNumber("KG", VALUE_KG);
        SmartDashboard.putNumber("KV", VALUE_KV);
        SmartDashboard.putNumber("KA", VALUE_KA);


        config.closedLoop.feedbackSensor(ClosedLoopConfig.FeedbackSensor.kPrimaryEncoder).pidf(kP, kI, kD, 0);
//
//        leftMotor.configure(config, SparkBase.ResetMode.kResetSafeParameters, SparkBase.PersistMode.kPersistParameters);
//        rightMotor.configure(config, SparkBase.ResetMode.kResetSafeParameters, SparkBase.PersistMode.kPersistParameters);

//        leftMotor.setVoltage(FEED_FORWARD.calculate(0));
    }

    public void setCurrentPosition() {

    }

    public void brake() {

    }

    public void moveUp(double speed) {
        double scaledSpeed = speed * 0.5;
        move(scaledSpeed);
    }

    public void moveDown(double speed) {
        if (leftMotor.getEncoder().getPosition() > ZERO_POINT) {
            double scaledSpeed = speed * 0.1;
            move(-scaledSpeed);
        } else {
            hold();
        }
    }

    public void move(double speed) {
        leftMotor.set(speed);
        rightMotor.set(-speed);
        leftMotorPrevPos = leftMotor.getEncoder().getPosition();
        rightMotorPrevPos = rightMotor.getEncoder().getPosition();
        System.out.println("x should be held down rn");
    }

//    @Override
//    public void periodic() {
//        leftMotor.set(pid.calculate(leftMotor.getEncoder().getPosition(), SET_POINT));
//        rightMotor.set(-pid.calculate(rightMotor.getEncoder().getPosition(), SET_POINT));
//
//        System.out.println("Left Encoder: " + leftMotor.getEncoder().getPosition());
//    }

    public void hold() {
//        leftMotor.set(0);
//        rightMotor.set(0);
        leftMotor.getClosedLoopController().setReference(leftMotorPrevPos, ControlType.kPosition);
        rightMotor.getClosedLoopController().setReference(rightMotorPrevPos, ControlType.kPosition);
//        leftMotor.getClosedLoopController().setReference(leftMotor.getEncoder().getPosition(), ControlType.kMAXMotionPositionControl, ClosedLoopSlot.kSlot0, new ElevatorFeedforward(VALUE_KS, VALUE_KV, VALUE_KA, VALUE_KG).calculate(0), SparkClosedLoopController.ArbFFUnits.kVoltage);
//        rightMotor.getClosedLoopController().setReference(rightMotor.getEncoder().getPosition(), ControlType.kMAXMotionPositionControl, ClosedLoopSlot.kSlot0, new ElevatorFeedforward(VALUE_KS, VALUE_KV, VALUE_KA, VALUE_KG).calculate(0), SparkClosedLoopController.ArbFFUnits.kVoltage);
//        leftMotor.setVoltage(FEED_FORWARD.calculate(0));
//        rightMotor.setVoltage(FEED_FORWARD.calculate(0));
    }

    @Override
    public void periodic() {
        System.out.println("Holding at (" + leftMotorPrevPos + ", " + rightMotorPrevPos + ")");
        System.out.println("Current Position: (" + leftMotor.getEncoder().getPosition() + ", " + rightMotor.getEncoder().getPosition() + ")");
    }
}
