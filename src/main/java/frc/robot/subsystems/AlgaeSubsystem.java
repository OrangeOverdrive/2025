package frc.robot.subsystems;


import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.ElevatorConstants;

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

    public void kickup(double speed) {
        pivot.set(speed);
    }

    public void kickdown(double speed) {
        pivot.set(-speed);
    }

    public void stopkick(double speed){
        pivot.set(speed);
    }
    

}

