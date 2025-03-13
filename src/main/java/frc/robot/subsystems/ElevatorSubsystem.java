package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.spark.*;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.config.ClosedLoopConfig;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ElevatorSubsystem extends SubsystemBase {
    private SparkMax leftMotor;
    private SparkMax rightMotor;
    private double startingPosition;
    private final double DISTANCE_TO_FULL = 1;
    private final double VALUE_P = 0.001;

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

    // Arm
    private final SparkMax m_arm;
    private final TalonFX m_armIntake;
    private double armPrevPos = 0;

    public ElevatorSubsystem() {
        leftMotor = new SparkMax(Constants.CanIDs.ELEVATOR_LEFT, SparkLowLevel.MotorType.kBrushless);
        rightMotor = new SparkMax(Constants.CanIDs.ELEVATOR_RIGHT, SparkLowLevel.MotorType.kBrushless);

        ZERO_POINT = leftMotor.getEncoder().getPosition();

        SparkMaxConfig leftConfig = new SparkMaxConfig();
        SparkMaxConfig rightConfig = new SparkMaxConfig();
        leftConfig.idleMode(SparkBaseConfig.IdleMode.kBrake);
        rightConfig.idleMode(SparkBaseConfig.IdleMode.kBrake);
        rightConfig.inverted(true);
//      config.encoder.positionConversionFactor(1000).velocityConversionFactor(1000);

        double kP = 0.001;
        double kI = 0;
        double kD = 0;

        SmartDashboard.putNumber("Elevator P", kP);
        SmartDashboard.putNumber("Elevator I", kI);
        SmartDashboard.putNumber("Elevator D", kD);

        SmartDashboard.putNumber("Left Encoder", leftMotor.getEncoder().getPosition());
        SmartDashboard.putNumber("Right Encoder", rightMotor.getEncoder().getPosition());

        leftConfig.closedLoop.feedbackSensor(ClosedLoopConfig.FeedbackSensor.kPrimaryEncoder).pidf(kP, kI, kD, 0);
        rightConfig.closedLoop.feedbackSensor(ClosedLoopConfig.FeedbackSensor.kPrimaryEncoder).pidf(kP, kI, kD, 0);
        leftMotor.configure(leftConfig, SparkBase.ResetMode.kNoResetSafeParameters, SparkBase.PersistMode.kNoPersistParameters);
        rightMotor.configure(rightConfig, SparkBase.ResetMode.kNoResetSafeParameters, SparkBase.PersistMode.kNoPersistParameters);

        // Arm
        m_arm = new SparkMax(Constants.CanIDs.ARM_PIVOT, SparkLowLevel.MotorType.kBrushless);
        m_armIntake = new TalonFX(Constants.CanIDs.ARM_INTAKE);
    }

    public void setCurrentPosition() {

    }

    public void brake() {

    }

    public void moveUp(double speed) {
        double scaledSpeed = speed * 0.2;
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
        rightMotor.set(speed);
        leftMotorPrevPos = leftMotor.getEncoder().getPosition();
        rightMotorPrevPos = rightMotor.getEncoder().getPosition();
    }

//    @Override
//    public void periodic() {
//        leftMotor.set(pid.calculate(leftMotor.getEncoder().getPosition(), SET_POINT));
//        rightMotor.set(-pid.calculate(rightMotor.getEncoder().getPosition(), SET_POINT));
//
//        System.out.println("Left Encoder: " + leftMotor.getEncoder().getPosition());
//    }

    public void hold() {
        if (leftMotor.getEncoder().getPosition() < ZERO_POINT) {
            leftMotor.set(0);
            rightMotor.set(0);
        } else {
            leftMotor.getClosedLoopController().setReference(leftMotorPrevPos, ControlType.kPosition);
            rightMotor.getClosedLoopController().setReference(rightMotorPrevPos, ControlType.kPosition);
        }
    }

    public void moveArm(double speed) {
        m_arm.set(speed);

        armPrevPos = m_arm.getEncoder().getPosition();
    }

    public void moveIntake(double speed) {
        m_armIntake.set(speed);
    }

    public void holdPivot() {
        m_arm.getClosedLoopController().setReference(armPrevPos, ControlType.kPosition);
    }

    @Override
    public void periodic() {
        System.out.println("Arm holding at (" + armPrevPos + ")");
        System.out.println("Arm Current Position: " + m_arm.getEncoder().getPosition());
    }
}
