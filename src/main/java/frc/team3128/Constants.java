package frc.team3128;

import java.util.HashMap;

import com.pathplanner.lib.path.PathConstraints;

import common.core.controllers.PIDFFConfig;
import common.core.swerve.SwerveConversions;
import common.core.swerve.SwerveModuleConfig;
import common.core.swerve.SwerveModuleConfig.SwerveMotorConfig;
import common.hardware.camera.Camera;
import common.hardware.motorcontroller.NAR_Motor.MotorConfig;
import common.hardware.motorcontroller.NAR_Motor.Neutral;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;


public class Constants {

    public static class AutoConstants {

        public static final double slowSpeed = 1.5;
        public static final double slowAcceleration = 2;

        public static final PathConstraints constraints = new PathConstraints(
            SwerveConstants.maxSpeed, SwerveConstants.maxAcceleration, SwerveConstants.maxAngularVelocity, SwerveConstants.maxAngularAcceleration); 

        /* Translation PID Values */
        public static final double translationKP = 3;
        public static final double translationKI = 0;
        public static final double translationKD = 0;
      
        /* Rotation PID Values */
        public static final double rotationKP = 2;
        public static final double rotationKI = 0;
        public static final double rotationKD = 0;

        public static final double ANGLE_THRESHOLD = 8; //7, 9
        public static final double VELOCITY_THRESHOLD = 4; //6, 3
        public static final double RAMP_THRESHOLD = 9; //8, 10
        public static final double DRIVE_SPEED = Units.inchesToMeters(20); //30, 40

    }

    public static class SwerveConstants {
        public static final int pigeonID = 30; 

        /* Drivetrain Constants */
        public static final double bumperLength = Units.inchesToMeters(5);
        public static final double trackWidth = Units.inchesToMeters(20.75); //Hand measure later
        public static final double wheelBase = Units.inchesToMeters(20.75); //Hand measure later
        public static final double robotLength = bumperLength + trackWidth;
        public static final double wheelDiameter = Units.inchesToMeters(4);
        public static final double wheelCircumference = wheelDiameter * Math.PI;

        public static final double closedLoopRamp = 0.0;

        public static final double driveGearRatio = 425.0 / 72.0;
        public static final double angleGearRatio = (150.0 / 7.0); 

        public static final SwerveDriveKinematics swerveKinematics = new SwerveDriveKinematics(
                new Translation2d(wheelBase / 2.0, trackWidth / 2.0),
                new Translation2d(wheelBase / 2.0, -trackWidth / 2.0),
                new Translation2d(-wheelBase / 2.0, trackWidth / 2.0),
                new Translation2d(-wheelBase / 2.0, -trackWidth / 2.0)); 

        /* Swerve Current Limiting */
        public static final int angleLimit = 30; //30
        public static final int driveLimit = 40; //40;

        /* Turning PID Values */
        public static final double turnKP = 5;
        public static final double turnKI = 0;
        public static final double turnKD = 0;

        /* Angle Motor PID Values */
        // switched 364 pid values to SDS pid values
        public static final double angleKP = 0.15; // 0.6; // citrus: 0.3 //0.15
        public static final double angleKI = 0.0;
        public static final double angleKD = 0.0; // 12.0; // citrus: 0
        public static final double angleKF = 0.0;

        /* Drive Motor PID Values */
        public static final double driveKP = 4e-5; //4e-5, //0.05
        public static final double driveKI = 0.0;
        public static final double driveKD = 0.0;
        public static final double driveKF = 0.0;

        /* Drive Motor Characterization Values */
        public static final double driveKS = 0.19255;//0.60094; // 0.19225;
        public static final double driveKV = 2.4366;//1.1559;  // 2.4366
        public static final double driveKA = 0; //0.12348; // 0.34415

        /* Swerve Profiling Values */
        // Theoretical: v = 4.96824, omega = 11.5
        // Real: v = 4.5, omega = 10
        // For safety, use less than theoretical and real values
        public static final double maxSpeed = 5; //meters per second - 16.3 ft/sec
        public static final double maxAttainableSpeed = maxSpeed * 0.85; //Stole from citrus.
        public static final double maxAcceleration = 3;
        public static final double maxAngularVelocity = 8; //3; //11.5; // citrus: 10 - Mason look at this later wtf
        public static final double maxAngularAcceleration = 2 * Math.PI; //I stole from citrus.

        /* Motor Inverts */
        public static final boolean driveMotorInvert = true;
        public static final boolean angleMotorInvert = true;

        /* Angle Encoder Invert */
        public static final boolean canCoderInvert = false;

        public static final MotorConfig driveMotorConfig = new MotorConfig(SwerveConversions.rotationsToMeters(1, wheelCircumference, driveGearRatio), 60, driveLimit, false, Neutral.COAST);

        public static final MotorConfig angleMotorConfig = new MotorConfig(SwerveConversions.rotationsToDegrees(1, angleGearRatio), 1, angleLimit, false, Neutral.COAST);

        public static final PIDFFConfig drivePIDConfig = new PIDFFConfig(driveKP, driveKI, driveKD, driveKS, driveKV, driveKA);

        public static final PIDFFConfig anglePIDConfig = new PIDFFConfig(angleKP, angleKI, angleKD);

        public static final SwerveModuleConfig Mod0 = new SwerveModuleConfig(
            0, 
            new SwerveMotorConfig(1, driveMotorConfig, drivePIDConfig),
            new SwerveMotorConfig(2, angleMotorConfig, anglePIDConfig),
            20,
            -9.31640625,
            canCoderInvert,
            maxSpeed);

        public static final SwerveModuleConfig Mod1 = new SwerveModuleConfig(
            1, 
            new SwerveMotorConfig(3, driveMotorConfig, drivePIDConfig),
            new SwerveMotorConfig(4, angleMotorConfig, anglePIDConfig),
            21,
            -64.423828125,
            canCoderInvert,
            maxSpeed);
        
        public static final SwerveModuleConfig Mod2 = new SwerveModuleConfig(
            2, 
            new SwerveMotorConfig(5, driveMotorConfig, drivePIDConfig),
            new SwerveMotorConfig(6, angleMotorConfig, anglePIDConfig),
            22,
            109.775390625,
            canCoderInvert,
            maxSpeed);
        
        public static final SwerveModuleConfig Mod3 = new SwerveModuleConfig(
            3, 
            new SwerveMotorConfig(7, driveMotorConfig, drivePIDConfig),
            new SwerveMotorConfig(8, angleMotorConfig, anglePIDConfig),
            23,
            -108.45703125,
            canCoderInvert,
            maxSpeed);
    }


    public static class VisionConstants {

        public static final double POSE_THRESH = 100;

        public static final Matrix<N3,N1> SVR_STATE_STD = VecBuilder.fill(0.1,0.1,Units.degreesToRadians(3));
 
        public static final Matrix<N3,N1> SVR_VISION_MEASUREMENT_STD = VecBuilder.fill(1,1,Units.degreesToRadians(10));

        public static final HashMap<Integer,Pose2d> APRIL_TAG_POS = new HashMap<Integer,Pose2d>();

        static {
            APRIL_TAG_POS.put(1, new Pose2d(
                new Translation2d(Units.inchesToMeters(610.77), Units.inchesToMeters(42.19)),
                Rotation2d.fromDegrees(180))
            );
            APRIL_TAG_POS.put(2, new Pose2d(
                new Translation2d(Units.inchesToMeters(610.77), Units.inchesToMeters(108.19)),
                Rotation2d.fromDegrees(180))
            );
            APRIL_TAG_POS.put(3, new Pose2d(
                new Translation2d(Units.inchesToMeters(610.77), Units.inchesToMeters(174.19)),
                Rotation2d.fromDegrees(180))
            );
            APRIL_TAG_POS.put(4, new Pose2d(
                new Translation2d(Units.inchesToMeters(636.96), Units.inchesToMeters(265.74)),
                Rotation2d.fromDegrees(180))
            );
            APRIL_TAG_POS.put(5, new Pose2d(
                new Translation2d(Units.inchesToMeters(14.25), Units.inchesToMeters(265.74)),
                Rotation2d.fromDegrees(0))
            );
            APRIL_TAG_POS.put(6, new Pose2d(
                new Translation2d( Units.inchesToMeters(40.45), Units.inchesToMeters(174.19)),
                Rotation2d.fromDegrees(0))
            );
            APRIL_TAG_POS.put(7, new Pose2d(
                new Translation2d(Units.inchesToMeters(40.45), Units.inchesToMeters(108.19)),
                Rotation2d.fromDegrees(0))
            );
            APRIL_TAG_POS.put(8, new Pose2d(
                new Translation2d(Units.inchesToMeters(40.45), Units.inchesToMeters(42.19)),
                Rotation2d.fromDegrees(0))
            );
        } 
    }
    
    public static class FieldConstants{

        public static final double FIELD_X_LENGTH = Units.inchesToMeters(651.25); // meters
        public static final double FIELD_Y_LENGTH = Units.inchesToMeters(315.5); // meters

        public static Pose2d allianceFlip(Pose2d pose) {
            if (Robot.getAlliance() == Alliance.Red) {
                return flip(pose);
            }
            return pose;
        }

        public static Translation2d allianceFlip(Translation2d translation) {
            if (Robot.getAlliance() == Alliance.Red) {
                return flipTranslation(translation);
            }
            return translation;
        }

        public static Rotation2d allianceFlip(Rotation2d rotation) {
            if (Robot.getAlliance() == Alliance.Red) {
                return flipRotation(rotation);
            }
            return rotation;
        }

        public static Pose2d flip(Pose2d pose) {
            return new Pose2d(flipTranslation(pose.getTranslation()), flipRotation(pose.getRotation()));
        }

        public static Translation2d flipTranslation(Translation2d translation) {
            return new Translation2d (
                FIELD_X_LENGTH - translation.getX(),
                translation.getY()
            );
        }

        public static Rotation2d flipRotation(Rotation2d rotation) {
            return Rotation2d.fromDegrees(MathUtil.inputModulus(180 - rotation.getDegrees(), -180, 180));
        }
    }

    public static class PivotConstants{

        public static final double kP = 0;//change
        public static final double kI = 0;//change
        public static final double kD = 0;//change

        public static final double kS = 0;//change
        public static final double kV = 0;//change
        public static final double kA = 0;
        public static final double kG = 0;//change

        public static final double GEAR_RATIO = 1.0 / 175.0;//change
        public static final double ROTATION_TO_DEGREES = 360;
        public static final double ANGLE_OFFSET = 0;

        public static final double maxVelocity = 0;//change
        public static final double maxAccelerration = 0;//change

        public static final int MIN_ANGLE = -90;
        public static final int MAX_ANGLE = 90;

        public static final int PIVOT_TOLERANCE = 1;//change prob

        public static final int PIVOT_ID = 0;//change

        public static final int PIVOT_MOTOR_ID = 0;
    }

    public static class WristConstants{

        public static final double kP = 0;//change
        public static final double kI = 0;//change
        public static final double kD = 0;//change

        public static final double kS = 0;//change
        public static final double kV = 0;//change
        public static final double kG = 0;//change

        public static final double GEAR_RATIO = 1 / 175.0;//change
        public static final double ROTATION_TO_DEGREES = 360;
        public static final double ANGLE_OFFSET = 0;

        public static final double maxVelocity = 0;//change
        public static final double maxAccelerration = 0;//change

        public static final int MIN_ANGLE = -90;
        public static final int MAX_ANGLE = 90;

        public static final int WRIST_TOLERANCE = 1;//change prob

        public static final int WRIST_ID = 0;//change

        public static final int WRIST_MOTOR_ID = 0;
    }

    public static class ShooterConstants {
        public static final PIDFFConfig PIDConstants = new PIDFFConfig(0, 0, 0, 0, 0, 0);
        public static final int LEFT_MOTOR_ID = 0;
        public static final int RIGHT_MOTOR_ID = 0;
        public static final double GEAR_RATIO = 1;
    }

    public static class LedConstants{
        public static final int CANDLE_ID = 52;
        
        public static final int WHITE_VALUE = 0; //leds used don't have a white value
        
        public static final int STARTING_ID = 8;
        public static final int PIVOT_COUNT = 200;
        public static final int PIVOT_COUNT_FRONT = 50; //change
        public static final int PIVOT_COUNT_BACK = 50; //change

        public static final double HOLDING_SPEED = 2;

        public static class RainbowAnimation {
            public static final double BRIGHTNESS = 1;
            public static final double SPEED = 1;

        }

        public enum Colors {
            OFF(0,0,0,false),
            CONE(255,255,0,false),
            CUBE(255,0,255,false),
            HOLDING(255,0,0,false),
    
            AUTO(0,0,0,true),
            SHELF(255, 105, 180, false),
            CHUTE(0,0,225,false);
    
            public final int r;
            public final int b;
            public final int g;
            public final boolean animation;
    
            Colors(int r, int g, int b,boolean animation) {
                this.r = r;
                this.g = g;
                this.b = b;
                this.animation = animation;
            }
    
        }
    }


}


