package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.spark.*;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.config.ClosedLoopConfig;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ElevatorSubsystem extends SubsystemBase {
    private final SparkMax leftMotor;
    private final SparkMax rightMotor;

    private final DigitalInput elevatorBottomLS = new DigitalInput(Constants.DigitalIO.ELEVATOR_BOTTOM);
    private final DigitalInput elevatorTopLS = new DigitalInput(Constants.DigitalIO.ELEVATOR_TOP);

    private final double ELEVATOR_ZERO;

    private double leftMotorPrevPos = 0;
    private double rightMotorPrevPos = 0;

    // Arm
    private final TalonFX armPivotMotor;
    private final TalonFX armIntakeMotor;
//    private double armPivotPrevPos = 0;


    public ElevatorSubsystem() {
        leftMotor = new SparkMax(Constants.CanIDs.ELEVATOR_LEFT, SparkLowLevel.MotorType.kBrushless);
        rightMotor = new SparkMax(Constants.CanIDs.ELEVATOR_RIGHT, SparkLowLevel.MotorType.kBrushless);


        ELEVATOR_ZERO = leftMotor.getEncoder().getPosition();

        SparkMaxConfig leftConfig = new SparkMaxConfig();
        SparkMaxConfig rightConfig = new SparkMaxConfig();
//        leftConfig.idleMode(SparkBaseConfig.IdleMode.kBrake);
//        rightConfig.idleMode(SparkBaseConfig.IdleMode.kBrake);
        rightConfig.inverted(true);

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
        armPivotMotor = new TalonFX(Constants.CanIDs.ARM_PIVOT);
        armIntakeMotor = new TalonFX(Constants.CanIDs.ARM_INTAKE);
    }

    public void moveUp(double speed) {
        double scaledSpeed = speed * 0.4;
        move(scaledSpeed);
    }

    public void moveDown(double speed) {
        if (leftMotor.getEncoder().getPosition() > ELEVATOR_ZERO) {
            double scaledSpeed = speed * 0.03;
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

    public void hold() {
        if (leftMotor.getEncoder().getPosition() < ELEVATOR_ZERO) {
            leftMotor.set(0);
            rightMotor.set(0);
        } else {
            leftMotor.getClosedLoopController().setReference(leftMotorPrevPos, ControlType.kPosition);
            rightMotor.getClosedLoopController().setReference(rightMotorPrevPos, ControlType.kPosition);
        }
    }

    // Arm
    public void movePivot(double speed) {
        double scaledSpeed = speed * 0.2;
        armPivotMotor.set(scaledSpeed);

//        armPivotPrevPos = armPivotMotor.getRotorPosition().getValueAsDouble();
    }

    public void intake(double speed) {
        armIntakeMotor.set(speed);
    }

    public void holdPivot() {
        armPivotMotor.set(0);
    }

    @Override
    public void periodic() {
//        System.out.println("Arm holding at (" + armPivotPrevPos + ")");
//        System.out.println("Arm Current Position: " + armPivotMotor.getRotorPosition().getValueAsDouble());

        System.out.println("Right elevator is feeding " + rightMotor.getEncoder().getVelocity());
        SmartDashboard.putNumber("Left Encoder", leftMotor.getEncoder().getPosition());
        SmartDashboard.putNumber("Right Encoder", rightMotor.getEncoder().getPosition());
        SmartDashboard.putNumber("Wrist Position", armPivotMotor.getRotorPosition().getValueAsDouble());
    }
}
