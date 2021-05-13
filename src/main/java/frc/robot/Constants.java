package frc.robot;

import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.kinematics.SwerveDriveKinematics;
import edu.wpi.first.wpilibj.util.Units;

public final class Constants {
    public static final class SwerveDrive {
        // Length and Width of robot in inches
        public static final double L = 25;
        public static final double W = 23;

        // TODO: Tune these PID values for your robot
        public static final double kDriveP = 1.71; // 15.0 1.71
        public static final double kDriveI = 0.01; // 0.01
        public static final double kDriveD = 0.10; // 0.10
        public static final double kDriveF = 0.20; // 0.20

        public static final double kAngleP = 0.5; // 1.0 0.00374
        public static final double kAngleI = 0.0; // 0.0
        public static final double kAngleD = 0.1; // 0.0

        // CANCoder has 4096 ticks/rotation
        public static double kEncoderTicksPerRotation = 4096;

        // IDs for Drive Motors
        public static final int frontLeftMotorID    = 1;
        public static final int frontRightMotorID   = 5;
        public static final int backLeftMotorID     = 11;
        public static final int backRightMotorID    = 7;

        // Talon SRX Turn Motor CAN ID
        public static final int frontLeftTurnTalonID    = 2;
        public static final int frontRightTurnTalonID   = 4;
        public static final int backLeftTurnTalonID     = 10;
        public static final int backRightTurnTalonID    = 9;

        // CANCoder ID
        public static final int frontLeftEncoderID  = 3;
        public static final int frontRightEncoderID = 6;
        public static final int backLeftEncoderID   = 12;
        public static final int backRightEncoderID  = 8;

        // Offset of cancoder to make encoders face forward
        // this is where you put the angle offsets you got from the smart dashboard
        public static final int frontLeftEncoderOffset  = -49;  // 1850 60 300 60 120 61 -49
        public static final int frontRightEncoderOffset = 48;  // 3675 144 221 270 46 140 48
        public static final int backLeftEncoderOffset   = -148;  // 550 221 144 221 78 218 -148
        public static final int backRightEncoderOffset  = 75;  // 1600 89 276 271 89 178 89 75

        //these are limits you can change!!!
        public static final double kMaxSpeed        = Units.feetToMeters(13.6); // 20 feet per second  13.6
        public static final double kMaxAngularSpeed = Math.PI; // 1/2 rotation per second

        private static final int kKinematics = 5;

        public static final SwerveDriveKinematics kinematics = new SwerveDriveKinematics(
            new Translation2d(
                Units.inchesToMeters(kKinematics),
                Units.inchesToMeters(kKinematics)
            ),
            new Translation2d(
                Units.inchesToMeters(kKinematics),
                Units.inchesToMeters(-kKinematics)
            ),
            new Translation2d(
                Units.inchesToMeters(-kKinematics),
                Units.inchesToMeters(kKinematics)
            ),
            new Translation2d(
                Units.inchesToMeters(-kKinematics),
                Units.inchesToMeters(-kKinematics)
            )
        );

        public static final SwerveDriveKinematics normalizedKinematics = new SwerveDriveKinematics(
            new Translation2d(
                Units.inchesToMeters(kKinematics),
                Units.inchesToMeters(kKinematics)
            ),
            new Translation2d(
                Units.inchesToMeters(kKinematics),
                Units.inchesToMeters(kKinematics)
            ),
            new Translation2d(
                Units.inchesToMeters(kKinematics),
                Units.inchesToMeters(kKinematics)
            ),
            new Translation2d(
                Units.inchesToMeters(kKinematics),
                Units.inchesToMeters(kKinematics)
            )
        );

    }

    public static final class Playstation {
        
        // Driver Controls
        public static final Integer USBID = 0;

        // Axis
        public static final Integer LeftXAxis   = 0;
        public static final Integer LeftYAxis   = 1;
        public static final Integer RightXAxis  = 2;
        public static final Integer RightYAxis  = 5;

        // Trigger
        public static final Integer LeftTrigger     = 3;
        public static final Integer RightTrigger    = 4;

        // Bumper
        public static final Integer LeftBumper  = 5;
        public static final Integer RightBumper = 6;

        // Buttons
        public static final Integer SquareButton    = 1;
        public static final Integer XButton         = 2;
        public static final Integer CircleButton    = 3;
        public static final Integer TriangleButton  = 4;

        public static final Integer LeftTriggerButton   = 7;
        public static final Integer RightTriggerButton  = 8;

        public static final Integer LeftButton  = 9;
        public static final Integer RightButton = 10;

        public static final Integer LeftJoystickButton  = 11;
        public static final Integer RightJoystickButton = 12;
        public static final Integer MiddleButton        = 13;
        public static final Integer BigButton           = 14;

        // POV Button
        public static final Integer NorthPOVButton      = 0;
        public static final Integer NorthEastPOVButton  = 45;
        public static final Integer EastPOVButton       = 90;
        public static final Integer SouthEastPOVButton  = 135;
        public static final Integer SouthPOVButton      = 180;
        public static final Integer SouthWestPOVButton  = 225;
        public static final Integer WestPOVButton       = 270;
        public static final Integer NorthWestPOVButton  = 315;
    }
}
