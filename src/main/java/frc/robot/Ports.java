package frc.robot;

public final class Ports {
    public static final class Shooter {
        public static final int motorPort = 0; // port constant for motor instantiation
    }

    public static final class Intake {
        public static final int beamBreakEntrancePort = 0;
        public static final int rollerPort = 0; // port constants for motor instantiation
        public static final int pivotPort = 0;
    }

    public static final class Elevator {
        public static final int elevatorPort = 0;
    }

    public static final class Drive {
        public static final int leftEncoderSourceA = 0;
        public static final int leftEncoderSourceB = 0;
        public static final int RightEncoderSourceA = 0;
        public static final int RightEncoderSourceB = 0;
    }
}
