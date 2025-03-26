package frc.robot.subsystems;

import static edu.wpi.first.units.Units.*;

import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import frc.robot.Constants;

public class ElevatorSubsystem2 {
    public SparkMax leftMotor;
    public SparkMax rightMotor;
    public SparkClosedLoopController closedLoopController;
    public SparkMaxConfig leftConfig;
    public SparkMaxConfig rightConfig;
    private double setpoint = 0;

    // Create a new instance of the RotateIONEO subsystem
    // Creates a new spark max using the provided motor id and creates a new motor controller and config
    public ElevatorSubsystem2() {
        leftMotor = new SparkMax(Constants.CanIDs.ELEVATOR_LEFT, SparkMax.MotorType.kBrushless);
        rightMotor = new SparkMax(Constants.CanIDs.ELEVATOR_RIGHT, SparkMax.MotorType.kBrushless);
        closedLoopController = leftMotor.getClosedLoopController();
        leftConfig = new SparkMaxConfig();
        rightConfig = new SparkMaxConfig();
        configure();
        leftMotor.getEncoder().setPosition(0);
//        rightMotor.getEncoder().setPosition(0);
    }

    /**
     * Configures the NEO motor
     */
    public void configure() {
        double P = 0;
        double I = 0;
        double D = 0;
        rightConfig.follow(Constants.CanIDs.ELEVATOR_LEFT);
        leftConfig.inverted(false).idleMode(IdleMode.kBrake);
        leftConfig.encoder
                .positionConversionFactor(1) // TODO: Find correct conversion factor
                .velocityConversionFactor(1); // TODO: Find correct conversion factor
        leftConfig.closedLoop
                .feedbackSensor(FeedbackSensor.kPrimaryEncoder) // TODO: Find correct feedback sensor - defaulted at primary encoder for now :)
                .pid(P, I, D) // using default slot 0 for this NEO - we will probably not use this slot much or at all
                .outputRange(-1, 1) // same thing here, will probably not use this
                .pid(P, I, D, ClosedLoopSlot.kSlot1) // TODO: Find correct PID values
                .velocityFF(0, ClosedLoopSlot.kSlot1) // TODO: Find correct velocity feedforward value - defaulted at 0 for now  :)
                .outputRange(-1, 1, ClosedLoopSlot.kSlot1);

        leftConfig.smartCurrentLimit(40);

        rightConfig
                .softLimit
                .forwardSoftLimit(20000)
                .reverseSoftLimit(-20000); // TODO: Find correct soft limits - both set to zero for now :)

        rightConfig.inverted(true).idleMode(IdleMode.kBrake);
        rightConfig.encoder
                .positionConversionFactor(1) // TODO: Find correct conversion factor
                .velocityConversionFactor(1); // TODO: Find correct conversion factor
        rightConfig.closedLoop
                .feedbackSensor(FeedbackSensor.kPrimaryEncoder) // TODO: Find correct feedback sensor - defaulted at primary encoder for now :)
                .pid(P, I, D) // using default slot 0 for this NEO - we will probably not use this slot much or at all
                .outputRange(-1, 1) // same thing here, will probably not use this
                .pid(P, I, D, ClosedLoopSlot.kSlot1) // TODO: Find correct PID values
                .velocityFF(0, ClosedLoopSlot.kSlot1) // TODO: Find correct velocity feedforward value - defaulted at 0 for now  :)
                .outputRange(-1, 1, ClosedLoopSlot.kSlot1);

        rightConfig.smartCurrentLimit(40);

        rightConfig
                .softLimit
                .forwardSoftLimit(20000)
                .reverseSoftLimit(-20000); // TODO: Find correct soft limits - both set to zero for now :)

        rightMotor.configure(rightConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        leftMotor.configure(
                leftConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    // Gets the encoder position of the NEO motor
    public double getEncoder() {
        return leftMotor.getEncoder().getPosition();
    }

    // Sets the target angle of the climber

    public void setElevatorTarget(double target) {
        if (target == 1) {
            closedLoopController.setReference(Constants.ElevatorConstants.CORAL_L1, ControlType.kPosition, ClosedLoopSlot.kSlot1);
        } else if (target == 2) {
            closedLoopController.setReference(Constants.ElevatorConstants.CORAL_L2, ControlType.kPosition, ClosedLoopSlot.kSlot1);
        } else if (target == 3) {
            closedLoopController.setReference(Constants.ElevatorConstants.CORAL_L3, ControlType.kPosition, ClosedLoopSlot.kSlot1);
        } else if (target == 4) {
            closedLoopController.setReference(Constants.ElevatorConstants.CORAL_L4, ControlType.kPosition, ClosedLoopSlot.kSlot1);
        } else if (target == 0) {
            closedLoopController.setReference(0, ControlType.kPosition, ClosedLoopSlot.kSlot1);
        }
    }

    // Stops the motor of the climber

    public void stop() {
        leftMotor.stopMotor();
    }

    // Sets the voltage of the climber

    public void setVoltage(double voltage) {
        leftMotor.setVoltage(voltage);
        //m_lockingServo.set(sAngle);
    }
}