package frc.robot;

public final class Ports {
    public static final class Shooter {
        public static final int motorPort = 0;
    }

    public static final class Intake {
        public static final int beamBreakEntrancePort = 0;
        public static final int rollerPort = 0;
        public static final int pivotPort = 0;
    }

    public static final class Elevator {
        public static final int elevatorPort = 0;
    }

    public static final class Drive {
        public static final int leftEncoderSourceA = 0;
        public static final int leftEncoderSourceB = 0;
        public static final int rightEncoderSourceA = 0;
        public static final int rightEncoderSourceB = 0;

        public static final int leftLeaderID = 0;
        public static final int rightLeaderID = 0;
        public static final int leftFollowerID = 0;
        public static final int rightFollowerID = 0;
    }

    public static class OperatorConstants {
        public static final int driverControllerPort = 0;
        public static final int operatorControllerPort = 0;
    }
}
