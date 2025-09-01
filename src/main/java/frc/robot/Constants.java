package frc.robot;

import edu.wpi.first.math.util.Units;

public class Constants {
    class constDrivetrain {

    }

    public class constElevator {

        public static int LEAD_MOTOR_ID = 10;
        public static int FOLLOW_MOTOR_ID = 11;

        public static double KG = 0.0;
        public static double KS = 0.00;
        public static double KV = 0.05;
        public static double KA = 0.000;

        public static double KP = 3.37;
        public static double KI = 0.00;
        public static double KD = 0.06;

        public static double motionVelocity = 32.0; // m/s
        public static double motionAcceleration = 48.0; // m/s²

        public static double EXPO_KV = 0.04;
        public static double EXPO_KA = 0.005;

        public static double IDLE = 1;
        public static double L1 = 1.325;
        public static double L2 = 1.65;
        public static double L3 = 1.975;
        public static double L4 = 2.3;


        // Simulated elevator parameters
        public static double simulationTick = 0.02; // Update every 20 ms
        public static double elevatorGearing = 6; // 8:48 reduction
        public static double carriageMass =  Units.lbsToKilograms(6.081);
        public static double elevatorDrumRadius = Units.inchesToMeters(1.538/2); // 1.538 inches diameter
        public static double minElevatorHeightMeters = Units.inchesToMeters(39.25);
        public static double maxElevatorHeightMeters = Units.inchesToMeters(93.75);

        public static double ROTATIONS_PER_METER = elevatorGearing / (2 * Math.PI * elevatorDrumRadius);
    }

}
