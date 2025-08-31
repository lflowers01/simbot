package frc.robot;


public class Constants {
    class constDrivetrain {

    }

    public class constElevator {
        public static double IDLE_HEIGHT  = 1.0414;

        public static int LEAD_MOTOR_ID = 10;
        public static int FOLLOW_MOTOR_ID = 11;


        public static double KG = 1.3;
        public static double KS = 0.15;
        public static double KV = 0.005;
        public static double KA = 0.00;

        public static double KP = 0.9;
        public static double KI = 0.0;
        public static double KD = 0.27;

        public static double motionVelocity = 0.0;
        public static double motionAcceleration = 0.0;

        public static double EXPO_KV = 0.04;
        public static double EXPO_KA = 0.005;

        public static double IDLE = 1;
        public static double L1 = 1.325;
        public static double L2 = 1.65;
        public static double L3 = 1.975;
        public static double L4 = 2.3;

        public static double elevatorGearing = 0.16666666666;
        public static double carriageMass = 2.7582952;
        public static double elevatorDrumRadius = 0.0195326;
        public static double minElevatorHeightMeters = 0.99695;
        public static double maxElevatorHeightMeters = 2.38125;

        public static double ROTATIONS_PER_METER = 1.0 / (2 * Math.PI * elevatorDrumRadius);
    }

}
