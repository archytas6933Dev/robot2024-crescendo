package frc.robot;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;

public class Constants {
    public static final class Swerve {
        public static final double kWheelDiameterMeters = Units.inchesToMeters(4);
        public static final double kDriveMotorGearRatio = 1 / 5.8462;
        public static final double kTurningMotorGearRatio = 1 / 18.0;
        public static final double kDriveEncoderRot2Meter = kDriveMotorGearRatio * Math.PI * kWheelDiameterMeters;
        public static final double kTurningEncoderRot2Rad = kTurningMotorGearRatio * 2 * Math.PI;
        public static final double kDriveEncoderRPM2MeterPerSec = kDriveEncoderRot2Meter / 60;
        public static final double kTurningEncoderRPM2RadPerSec = kTurningEncoderRot2Rad / 60;
        public static final double kPTurning = 0.5;
    }

    public static final class Drive {

        public static final double kTrackWidth = Units.inchesToMeters(21);
        // Distance between right and left wheels
        public static final double kWheelBase = Units.inchesToMeters(25.5);
        // Distance between front and back wheels
        public static final SwerveDriveKinematics kDriveKinematics = new SwerveDriveKinematics(
                new Translation2d(kWheelBase / 2, -kTrackWidth / 2),
                new Translation2d(kWheelBase / 2, kTrackWidth / 2),
                new Translation2d(-kWheelBase / 2, -kTrackWidth / 2),
                new Translation2d(-kWheelBase / 2, kTrackWidth / 2));

        public static final int kFrontLeftDriveMotorPort = 22;
        public static final int kBackLeftDriveMotorPort = 23;
        public static final int kFrontRightDriveMotorPort = 21;
        public static final int kBackRightDriveMotorPort = 24;

        public static final int kFrontLeftTurningMotorPort = 12;
        public static final int kBackLeftTurningMotorPort = 13;
        public static final int kFrontRightTurningMotorPort = 11;
        public static final int kBackRightTurningMotorPort = 14;

        public static final boolean kFrontLeftTurningEncoderReversed = true;
        public static final boolean kBackLeftTurningEncoderReversed = true;
        public static final boolean kFrontRightTurningEncoderReversed = true;
        public static final boolean kBackRightTurningEncoderReversed = true;

        public static final boolean kFrontLeftDriveEncoderReversed = true;
        public static final boolean kBackLeftDriveEncoderReversed = true;
        public static final boolean kFrontRightDriveEncoderReversed = false;
        public static final boolean kBackRightDriveEncoderReversed = false;

        public static final int kFrontLeftDriveAbsoluteEncoderPort = 2;
        public static final int kBackLeftDriveAbsoluteEncoderPort = 3;
        public static final int kFrontRightDriveAbsoluteEncoderPort = 1;
        public static final int kBackRightDriveAbsoluteEncoderPort = 4;

        public static final boolean kFrontLeftDriveAbsoluteEncoderReversed = false;
        public static final boolean kBackLeftDriveAbsoluteEncoderReversed = false;
        public static final boolean kFrontRightDriveAbsoluteEncoderReversed = false;
        public static final boolean kBackRightDriveAbsoluteEncoderReversed = false;

        public static final double kFrontLeftDriveAbsoluteEncoderOffsetRad = 0.578+Math.PI;
        public static final double kBackLeftDriveAbsoluteEncoderOffsetRad = 4.982;
        public static final double kFrontRightDriveAbsoluteEncoderOffsetRad = 3.295+Math.PI;
        public static final double kBackRightDriveAbsoluteEncoderOffsetRad = 1.164;

        public static final double kPhysicalMaxSpeedMetersPerSecond = 20; // 10;
        public static final double kPhysicalMaxAngularSpeedRadiansPerSecond = 2 * Math.PI * 8; // 4;

        public static final double kTeleDriveMaxSpeedMetersPerSecond = kPhysicalMaxSpeedMetersPerSecond;// / 4;
        public static final double kTeleDriveMaxAngularSpeedRadiansPerSecond = kPhysicalMaxAngularSpeedRadiansPerSecond;// / 4;
        public static final double kTeleDriveMaxAccelerationUnitsPerSecond = 5; //3;
        public static final double kTeleDriveMaxAngularAccelerationUnitsPerSecond = 8; //3;
    }

    public static final class Auto {
        public static final double kMaxSpeedMetersPerSecond = Drive.kPhysicalMaxSpeedMetersPerSecond / 4;
        public static final double kMaxAngularSpeedRadiansPerSecond = //
                Drive.kPhysicalMaxAngularSpeedRadiansPerSecond / 10;
        public static final double kMaxAccelerationMetersPerSecondSquared = 3;
        public static final double kMaxAngularAccelerationRadiansPerSecondSquared = Math.PI / 4;
        public static final double kPXController = 1.5;
        public static final double kPYController = 1.5;
        public static final double kPThetaController = 3;

        public static final TrapezoidProfile.Constraints kThetaControllerConstraints = //
                new TrapezoidProfile.Constraints(
                        kMaxAngularSpeedRadiansPerSecond,
                        kMaxAngularAccelerationRadiansPerSecondSquared);
    }

    public static final class Control {
        public static final int kDriverControllerPort = 0;
        public static final int kOperatorControllerPort = 1;


        public static int ABUTTON = 1;
        public static int BBUTTON = 2;
        public static int XBUTTON = 3;
        public static int YBUTTON = 4;
        public static int LBBUTTON = 5;
	    public static int RBBUTTON = 6;
        public static int BACKBUTTON = 7;
        public static int STARTBUTTON = 8;
        public static int LEFTJOYCLICK = 9;
        public static int RIGHTJOYCLICK = 10;

        
        public static final int LEFT_Y_AXIS = 1;
        public static final int LEFT_X_AXIS = 0;
        public static final int LEFT_TRIGGER = 2;
        public static final int RIGHT_TRIGGER = 3;
        public static final int RIGHT_X_AXIS = 4;
        public static final int RIGHT_Y_AXIS = 5;

        public static final int DAXISN = 0;
        public static final int DAXISS = 180;
        public static final int DAXISE = 90;
        public static final int DAXISW = 270;      


        public static final double kDeadband = 0.1;
    }
    public static final class Shooter {
        public static final boolean EXISTS = true;
        public static final double MOTOR_VELOCITY_F = 0.005; //0.011;
        public static final double MOTOR_VELOCITY_P = 0.001; //0.25
        public static final double MOTOR_VELOCITY_I = 0.00015;
        public static final double MOTOR_VELOCITY_D = 2.0;
        public static final int MOTOR_ID = 0;
        public static final int FOLLOWER_ID = 0;
        public static final long SHOTTOTALTIME = 1000; // in milliseconds
        public static final double SHOT_TOLERANCE = 500;

        public static final double TILT_POSITION_F = 0; //0.011;
        public static final double TILT_POSITION_P = 0.03; //0.25
        public static final double TILT_POSITION_I = 0;
        public static final double TILT_POSITION_D = 0;

        public static final double TILT_THRESHOLD = 100;

        public static final double TILT_LOW = 300000;
        public static final double TILT_MEDIUM = 250000;
        public static final double TILT_HIGH = 200000;
        public static final double TILT_CLIMB = 50000;//100000


        public static final double HIGH_OFFSET = -11;
        public static final double MEDIUM_OFFSET = -3.0; //-1.5
        public static final double LOW_OFFSET = 4;


        public static final double SHOT_MEDIUM = 14000;
        public static final double AUTO_SHOT_Y = -8; //-14

        public static final double XDeadband = 2; //3
        public static final double YDeadband = 2; //3


        public static final long TARGET_STALE = 100;


        public static final double kSVolts = 0;
        public static final double kVVoltSecondsPerRotation = 0;

    }
    public static final class Intake {
        public static final boolean EXISTS = true;
        public static final int MOTOR_ID = 51;
        public static final double MOTOR_POSITION_F = 0.0;
        public static final double MOTOR_POSITION_P = 0.25;
        public static final double MOTOR_POSITION_I = 0.0;
        public static final double MOTOR_POSITION_D = 5.0;

       public static final double MOTOR_VELOCITY_F = 0.055; //0.011;
       public static final double MOTOR_VELOCITY_P = 0.025; //0.25
       public static final double MOTOR_VELOCITY_I = 0.0;
       public static final double MOTOR_VELOCITY_D = 0.0;

        public static final double AUTO_NOTE_Y = 40;

        public static final long NOTE_STALE = 500;
        public static final long GRABBED_STALE = 500;
        public static final long SHOT_STALE = 500;
        public static final long INTAKE_STALE = 1000;


        public static final double INTAKE_SPEED = -4500;
        public static final double FEED_SPEED = -7000;
        public static final double SPIT_SPEED = 7000;
        public static final int SWITCH1_ID = 1;
        public static final int SWITCH2_ID = 0;
    }
    public static final class Climber{
        public static final boolean EXISTS = true;
        public static final int MOTOR_ID = 30;
        public static final double SPEED = 1;
    }
}
