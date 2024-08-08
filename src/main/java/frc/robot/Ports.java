package frc.robot;

public final class Ports {
    public static final class Shooter {
        public static final int motorPort = 0;
    }

    public static final class Intake {
        public static final int beamBreakEntrancePort = 1;
        public static final int rollerPort = 2;
        public static final int pivotPort = 3;
    }

    public static final class Elevator {
        public static final int elevatorPort = 4;
    }

    public static final class Drive {
        public static final int leftLeaderID = 9;
        public static final int rightLeaderID = 10;
        public static final int leftFollowerID = 11;
        public static final int rightFollowerID = 12;
    }

    public static class Operator {
        public static final int driverControllerPort = 13;
        public static final int operatorControllerPort = 14;
    }
}
