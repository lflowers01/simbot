package frc.robot;

import com.ctre.phoenix6.configs.TalonFXConfiguration;

import edu.wpi.first.math.util.Units;

public class Constants {
    class constDrivetrain {

    }

    public class constElevator {
        public static double IDLE_HEIGHT  = 1.0414;

        public static int LEAD_MOTOR_ID = 10;
        public static int FOLLOW_MOTOR_ID = 11;

        public static double KG = 0.3;
        public static double KS = 0.4;
        public static double KV = 0.001;
        public static double KF = 0.1;
        public static double KA = 0.0;
        public static double KP = 3;
        public static double KI = 0;
        public static double KD = 0;

        public static double motionVelocity = 5;
        public static double motionAcceleration = 10;
        public static double EXPO_KV = 0.04;
        public static double EXPO_KA = 0.005;

        public static double IDLE = 0;
        public static double L1 = 0.5;
        public static double L2 = 1;
        public static double L3 = 1.5;
        public static double L4 = 2.5;

        public static double elevatorGearing = 10;
        public static double carriageMass = 1.76901;
        public static double elevatorDrumRadius = 0.1;
        public static double minElevatorHeightMeters = 0.5;
        public static double maxElevatorHeightMeters = 3;

        public static double ROTATIONS_PER_METER = 5;
    }

}
