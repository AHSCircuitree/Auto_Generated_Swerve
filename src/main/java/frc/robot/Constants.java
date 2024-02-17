package frc.robot;

public class Constants {
 
    // Auto
    public static double POSETOLERANCE = .1;
    public static double ANGLETOLERANCE = 2;

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

    public class WayPoints {

        // Initial Points
        public static double FieldCenter[] = {0, 0, 0};
        public static double BlueStartingLeft[] = {6.782, 0.610, -120};
        public static double BlueStartingCenter[] = {5.639, 1.372, 178};
        public static double BlueStartingRight[] = {4.369, 0.610, 120};
        public static double RedStartingLeft[] = {4.369, 16.002, 60};
        public static double RedStartingCenter[] = {5.639, 15.189, 0};
        public static double RedStartingRight[] = {6.782, 16.002, -60};

        // Blue Side Rings
        public static double BlueThreeShootStart1[] = {7.5104, 1.5654, -120};
        public static double BlueThreeShootStart2[] = {7.5104, 2, -90};
        public static double BlueLeftRing[] = {7.0104, 2.8, -90};
        public static double BlueLeftRingShoot[] = {7.0104, 2.9654, -160};
        public static double BlueCenterRing[] = {5.5626, 2.8, 178};
        public static double BlueAboveRightRing[] = {3.9894, 1.8, 90};
        public static double BlueRightRing[] = {3.9494, 2.6654, 90};
        public static double BlueRightRingShoot1[] = {4.2894, 2.2, 90};
        public static double BlueRightRingShoot2[] = {4.2894, 2.2, 150};

        // Blue Right Steal
        public static double BlueStealShootingLine[] = {3.282, 2.110, 120};
        public static double RightBlueStageLine[] = {0.7874, 5.8674, 120};
        public static double MiddleRing5FromBlue[] = {0.5874, 8.2, 90};
        public static double MiddleRing4FromBlue[] = {2.6384, 8.2, 90};

        // Blue Left Steal
        public static double BlueLeftStealLineUp1[] = {7, 2, -120};
        public static double BlueLeftStealRing1[] = {7.0104, 2.8 + .2, -90};
        public static double BlueLeftStealRingShoot[] = {7.0104, 2.8 + .2, -150};
        public static double BlueLeftStealLineUp2[] = {7.46, 5.82, -90};
        public static double BlueLeftStealRing2[] = {7.46, 8.2 + .2, -90};
        public static double BlueLeftStealLineUp3[] = {6.14, 5.82, -90};
        public static double BlueLeftStealRing3[] = {5.76, 8.2 + .2, -90};
      

        //Red Side Rings
        public static double RedLeftRing[] = {4.0894,13.9192,0};
        public static double RedCenterRing[] = {5.588,13.9192,0};
        public static double RedRightRing[] = {7.0104,13.9192,0};

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
