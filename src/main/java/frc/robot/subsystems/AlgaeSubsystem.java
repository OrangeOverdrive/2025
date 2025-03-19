package frc.robot.subsystems;


import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class AlgaeSubsystem extends SubsystemBase {
    private final TalonFX pivot;
    private final SparkMax suction;

    public AlgaeSubsystem() {
        pivot = new TalonFX(Constants.CanIDs.ALGAE_PIVOT);
        suction = new SparkMax(Constants.CanIDs.ALGAE_SUCTION, SparkLowLevel.MotorType.kBrushed);
    }

    public void suction(boolean activated) {
        suction.set(activated ? 1 : 0);
    }
}

