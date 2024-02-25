package frc.robot;

public class Constants {
 
    // Auto
    public static double POSETOLERANCE = .1;
    public static double ANGLETOLERANCE = 2;

    public enum RobotState {

        NO_RING,
        RING_DETECTED,
        RING_COLLECTED,
        VALID_TARGET
    
    }

    public static int[] ShootAngle = {
        55, // 0 Meters 
        55, 
        55, 
        55, 
        55, 
        55, 
        55, 
        55,
        54,
        53,
        48,  // 1 Meter
        48,
        48,
        48,
        47, //50
        47,
        44, //47
        42,
        39, //42
        37,
        36,  // 2 Meter
        36,
        35,
        34,
        33, // 34
        32,
        31,
        30,
        29,
        29,
        29,  // 3 Meter
        29,
        29,
        29,
        29,
        29,
        29,
        29,
        29,
        29,
        29,  // 4 Meter
    }; 
 
    public class Limits {

        public static final int TopAngleLimit = 60;
        public static final int BottomAngleLimit = -60;

    }

    public class CAN_IDs {

        public static final int FrontLeftDriveID = 1;
        public static final int FrontRightDriveID = 3;
        public static final int BackLeftDriveID = 5;
        public static final int BackRightDriveID = 7;

        public static final int FrontLeftTurnID = 2;
        public static final int FrontRightTurnID = 4;
        public static final int BackLeftTurnID = 6;
        public static final int BackRightTurnID = 8;

        public static final int FrontLeftEncoderID = 11;
        public static final int FrontRightEncoderID = 13;
        public static final int BackLeftEncoderID = 15;
        public static final int BackRightEncoderID = 17;


        public static final int LeftIntakeID = 21;
        public static final int RightIntakeID = 22;
        public static final int FrontIntakeID = 23;
        public static final int RearFlyID = 24;
        public static final int FrontFlyID = 25;

        public static final int BottomShootingID = 26;
        public static final int CentralShootingID = 27;
        public static final int TopShootingID = 28;

        public static final int RightHookID = 29;
        public static final int LeftHookID = 30;
        public static final int AngleID = 31;


        public static final int PigeonID = 41;
        public static final int CanivoreID = 42;
        public static final int PowerDistrubutionID = 43;
        

        public final String CANBUS_NAME = "FRC 1599";

    }
 
    public class Colors {

        // Fixed Patterns
        public static double Rainbow = -.99;
        public static double Ocean = -.95;
        public static double Lava = -.93;
        public static double Forest = -.91;

        // Color Patterns
        public static double ChaseColor = .01;
        public static double HeartbeatColor = .05;

        // Solid Colors
        public static double HotPink = .57;
        public static double DarkRed = .59;
        public static double Red = .61;
        public static double RedOrange = .63;
        public static double Orange = .65;
        public static double Gold = .67;
        public static double Yellow = .69;
        public static double LawnGreen = .71;
        public static double Lime = .73;
        public static double DarkGreen = .75;
        public static double Green = .77;
        public static double BlueGreen = .79;
        public static double Aqua = .81;
        public static double SkyBlue = .83;
        public static double DarkBlue = .85;
        public static double Blue = .87;
        public static double BlueViolet = .89;
        public static double Violet = .91;
        public static double White = .93;
        public static double Grey = .95;
        public static double DarkGrey = .97;
        public static double Black = .99;
        public static double BlueShot = -.83;
        public static double RedShot = -.85; 
        public static double RainbowRainbowPalette = -.99;
        public static double RedLarsonScanner = -.35;
        public static double GrayLarsonScanner = -.33;
            }
        
}
