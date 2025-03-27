package frc.robot;

public class Constants {
    public static final class CanIDs {
        public static final int ELEVATOR_LEFT = 21;
        public static final int ELEVATOR_RIGHT = 22;

        public static final int ARM_PIVOT = 30;
        public static final int ARM_INTAKE = 31;

        public static final int WINCH = 32;

        public static final int ALGAE_PIVOT = 33;
        public static final int ALGAE_SUCTION = 34;
    }

    public static final class DigitalIO {
        public static final int WINCH = 0;
        public static final int ELEVATOR_BOTTOM = 1;
        public static final int ELEVATOR_TOP = 2;
    }

    public static final class ElevatorConstants {
        public static final double CORAL_L1 = 0;
        public static final double CORAL_L2 = 0;
        public static final double CORAL_L3 = 5.99999;
        public static final double CORAL_L4 = 20.78562355;

        public static final double ARM_HOME = 0; //Level is 7.64599609375
        public static final double ARM_INTAKE_STATION = 4.1025390625;
        public static final double ARM_DEPOSIT_ANGLED = 0;
        public static final double ARM_DEPOSIT_UPRIGHT = 0;
        public static final double kicker = 0.1;
    }
}
