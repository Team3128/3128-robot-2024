package frc.team3128;

import static frc.team3128.Constants.SwerveConstants.controllerInputRotationAngle;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.function.Function;

import com.ctre.phoenix6.hardware.CANcoder;
import com.pathplanner.lib.path.PathConstraints;

import common.core.controllers.Controller;
import common.core.controllers.PIDFFConfig;
import common.core.controllers.Controller.Type;
import common.core.swerve.SwerveConversions;
import common.core.swerve.SwerveModuleConfig;
import common.core.swerve.SwerveModuleConfig.SwerveEncoderConfig;
import common.core.swerve.SwerveModuleConfig.SwerveMotorConfig;
import common.hardware.motorcontroller.NAR_CANSpark;
import common.hardware.motorcontroller.NAR_TalonFX;
import common.hardware.motorcontroller.NAR_CANSpark.ControllerType;
import common.hardware.motorcontroller.NAR_Motor.MotorConfig;
import common.hardware.motorcontroller.NAR_Motor.Neutral;
import common.hardware.motorcontroller.NAR_Motor.StatusFrames;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.team3128.subsystems.Swerve;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;


public class Constants {

    public static class AutoConstants {

        public static final double slowSpeed = 1.5;
        public static final double slowAcceleration = 2;

        public static final PathConstraints constraints = new PathConstraints(
            SwerveConstants.maxSpeed, SwerveConstants.maxAcceleration, SwerveConstants.maxAngularVelocity, SwerveConstants.maxAngularAcceleration); 

        /* Translation PID Values */
        public static final double translationKP = 2;
        public static final double translationKI = 0;
        public static final double translationKD = 0;
      
        /* Rotation PID Values */
        public static final double rotationKP = 5;
        public static final double rotationKI = 0;
        public static final double rotationKD = 0;

        public static final double ANGLE_THRESHOLD = 8; //7, 9
        public static final double VELOCITY_THRESHOLD = 4; //6, 3
        public static final double RAMP_THRESHOLD = 9; //8, 10
        public static final double DRIVE_SPEED = Units.inchesToMeters(20); //30, 40

    }

    public static class SwerveConstants {

        public static final int pigeonID = 15; 

        public static final double controllerInputRotationAngle = -90;

        /* Drivetrain Constants */
        public static final double bumperLength = Units.inchesToMeters(5);
        public static final double trackWidth = Units.inchesToMeters(20.75); //Hand measure later
        public static final double wheelBase = Units.inchesToMeters(20.75); //Hand measure later
        public static final double robotLength = Units.inchesToMeters(26.5) + bumperLength; // bumperLength + trackWidth;
        public static final double wheelDiameter = Units.inchesToMeters(4);
        public static final double wheelCircumference = wheelDiameter * Math.PI;

        public static final double closedLoopRamp = 0.0;

        public static final double driveGearRatio = 225.0 / 42.0;
        public static final double angleGearRatio = (300.0 / 14.0); 

        public static final SwerveDriveKinematics swerveKinematics = new SwerveDriveKinematics(
                new Translation2d(wheelBase / 2.0, trackWidth / 2.0),
                new Translation2d(wheelBase / 2.0, -trackWidth / 2.0),
                new Translation2d(-wheelBase / 2.0, trackWidth / 2.0),
                new Translation2d(-wheelBase / 2.0, -trackWidth / 2.0)); 

        /* Swerve Current Limiting */
        public static final int angleLimit = 30; //30
        public static final int driveLimit = 60; //40;
        public static final int stallLimit = 40;

        /* Angle Motor PID Values */
        // switched 364 pid values to SDS pid values
        public static final double angleKP = 0.15 * 30; // 0.6; // citrus: 0.3 //0.15
        public static final double angleKI = 0.0;
        public static final double angleKD = 0.0; // 12.0; // citrus: 0
        public static final double angleKF = 0.0;

        /* Drive Motor PID Values */
        public static final double driveKP = 4e-5; //4e-5, //0.05
        public static final double driveKI = 0.0;
        public static final double driveKD = 0.0;
        public static final double driveKF = 0.0;

        /* Drive Motor Characterization Values */
        public static final double driveKS = 0.19057;//0.60094; // 0.19225;
        public static final double driveKV = 2.01208;//1.1559;  // 2.4366
        public static final double driveKA = 0.15168; //0.12348; // 0.34415

        /* Swerve Profiling Values */
        // Theoretical: v = 4.96824, omega = 11.5
        // Real: v = 4.5, omega = 10
        // For safety, use less than theoretical and real values
        public static final double maxSpeed = 5.87;//4.8; //meters per second - 16.3 ft/sec
        public static final double maxAttainableSpeed = maxSpeed; //Stole from citrus.
        public static final double maxAcceleration = 5;
        public static final double maxAngularVelocity = 8; //3; //11.5; // citrus: 10
        public static final double maxAngularAcceleration = 2 * Math.PI; //I stole from citrus.

        /* Motor Inverts */
        public static final boolean driveMotorInvert = false;
        public static final boolean angleMotorInvert = true;

        /* Angle Encoder Invert */
        public static final boolean canCoderInvert = false;

        public static final MotorConfig driveMotorConfig = 
                            new MotorConfig(
                                SwerveConversions.rotationsToMeters(1, wheelCircumference, driveGearRatio), 
                                60, 
                                driveLimit,
                                12, 
                                driveMotorInvert, 
                                Neutral.BRAKE,
                                StatusFrames.VELOCITY
                            );

        public static final MotorConfig angleMotorConfig = 
                            new MotorConfig(
                                SwerveConversions.rotationsToDegrees(1, angleGearRatio), 
                                1, 
                                angleLimit, 
                                12,
                                angleMotorInvert, 
                                Neutral.BRAKE,
                                StatusFrames.POSITION
                            );

        public static final PIDFFConfig drivePIDConfig = new PIDFFConfig(driveKP, driveKI, driveKD, driveKS, driveKV, driveKA);

        public static final PIDFFConfig anglePIDConfig = new PIDFFConfig(angleKP, angleKI, angleKD);

        public static final SwerveModuleConfig Mod0 = new SwerveModuleConfig(
            0, 
            new SwerveMotorConfig(new NAR_TalonFX(1, "Drivetrain"), driveMotorConfig, drivePIDConfig),
            new SwerveMotorConfig(new NAR_TalonFX(2, "Drivetrain"), angleMotorConfig, anglePIDConfig),
            new SwerveEncoderConfig(new CANcoder(11, "Drivetrain"), 105.15, canCoderInvert),
            maxSpeed
        );

        public static final SwerveModuleConfig Mod1 = new SwerveModuleConfig(
            1, 
            new SwerveMotorConfig(new NAR_TalonFX(3, "Drivetrain"), driveMotorConfig, drivePIDConfig),
            new SwerveMotorConfig(new NAR_TalonFX(4, "Drivetrain"), angleMotorConfig, anglePIDConfig),
            new SwerveEncoderConfig(new CANcoder(12, "Drivetrain"), -62.4, canCoderInvert),
            maxSpeed
        );

        public static final SwerveModuleConfig Mod2 = new SwerveModuleConfig(
            2, 
            new SwerveMotorConfig(new NAR_TalonFX(5, "Drivetrain"), driveMotorConfig, drivePIDConfig),
            new SwerveMotorConfig(new NAR_TalonFX(6, "Drivetrain"), angleMotorConfig, anglePIDConfig),
            new SwerveEncoderConfig(new CANcoder(13, "Drivetrain"), -84.287, canCoderInvert),
            maxSpeed
        );
        
        public static final SwerveModuleConfig Mod3 = new SwerveModuleConfig(
            3, 
            new SwerveMotorConfig(new NAR_TalonFX(7, "Drivetrain"), driveMotorConfig, drivePIDConfig),
            new SwerveMotorConfig(new NAR_TalonFX(8, "Drivetrain"), angleMotorConfig, anglePIDConfig),
            new SwerveEncoderConfig(new CANcoder(14, "Drivetrain"), 167.6, canCoderInvert),
            maxSpeed
        );

        public static final Constraints translationConstraints = new Constraints(maxAttainableSpeed, AutoConstants.slowAcceleration);
        public static final PIDFFConfig translationConfig = new PIDFFConfig(2.5, 0, 0, 0.1, 0.01723, 0.0064);

        public static final Constraints rotationConstraints = new Constraints(Units.radiansToDegrees(maxAngularVelocity), Units.radiansToDegrees(maxAngularAcceleration));
        public static final PIDFFConfig rotationConfig = new PIDFFConfig(2, 0, 0, driveKS, driveKV, driveKA);

        public static final Controller translationController = new Controller(translationConfig, Type.POSITION);
        public static final double TRANSLATION_TOLERANCE = 0.02;

        public static final Controller rotationController = new Controller(rotationConfig, Type.POSITION);
        public static final double TURN_TOLERANCE = 0.01;

        static {
            translationController.setTolerance(TRANSLATION_TOLERANCE);
            translationController.setOutputRange(-maxAttainableSpeed, maxAttainableSpeed);
            translationController.setDisableAtSetpoint(true);
            
            rotationController.setTolerance(TURN_TOLERANCE);
            rotationController.setOutputRange(-maxAngularVelocity, maxAngularVelocity);
            rotationController.setDisableAtSetpoint(true);
            rotationController.enableContinuousInput(-180, 180);
        }

        public static final List<Rotation2d> snapToAngles = new ArrayList<>();
        static {
            snapToAngles.add(Rotation2d.fromDegrees(-180));
            snapToAngles.add(Rotation2d.fromDegrees(-90));
            snapToAngles.add(Rotation2d.fromDegrees(0));
            snapToAngles.add(Rotation2d.fromDegrees(90));
            snapToAngles.add(Rotation2d.fromDegrees(180));
        }
    }


    public static class VisionConstants {

        public static final double POSE_THRESH = 100;

        public static final Matrix<N3,N1> SVR_STATE_STD = VecBuilder.fill(0.1,0.1,Units.degreesToRadians(3));
 
        public static final Matrix<N3,N1> SVR_VISION_MEASUREMENT_STD = VecBuilder.fill(0.5,0.5,Units.degreesToRadians(5));

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
        public static final Pose2d SPEAKER = 
            new Pose2d(
                Units.inchesToMeters(324.5), 
                Units.inchesToMeters(315.5), 
                Rotation2d.fromDegrees(0)
            );
        
        public enum Note {
            NOTE1_1(2.89, 6.99),
            NOTE1_2(2.89, 5.55),
            NOTE1_3(2.89, 4.11),
            NOTE2_1(6.45, 7.44),
            NOTE2_2(6.45, 5.79),
            NOTE2_3(6.45, 4.10),
            NOTE2_4(6.45, 2.45),
            NOTE2_5(6.45, 0.77);

            private final Translation2d translation;
            private Note(double x, double y) {
                translation = new Translation2d(x, y);
            }
            public Translation2d getTranslation() {
                return translation;
            }
        }

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

        public static Translation2d orthogonalizeInputs(double x, double y) {
            return orthogonalizeInputs(new Translation2d(x, y));
        }

        public static Translation2d orthogonalizeInputs(Translation2d translation) {
            Rotation2d rotation = Rotation2d.fromDegrees(-90);
            if(Robot.getAlliance() == Alliance.Red || !Swerve.getInstance().fieldRelative) {
                rotation.unaryMinus();
            }
            return translation.rotateBy(rotation);
        }
    }

    public static class LimelightConstants {
        public static final double TX_THRESHOLD = 1;
        public static final double HORIZONTAL_OFFSET_GOAL = 0;
        public static final double PLATEAU_THRESHOLD = 32; //25 //roughly 0.5 seconds
        public static final double TIMEOUT = 0.5;
        public static final double KP = 0.1;
        public static final double KI = 0;
        public static final double KD = 0;

        //auto align
        public static final PIDFFConfig config = new PIDFFConfig(KP, KI, KD);
    }

    public static class LedConstants{
        public static final int CANDLE_ID = 52;
        
        public static final int WHITE_VALUE = 0; //leds used don't have a white value
        
        public static final double r_SPEED = 0.75;
        public static final double c_SPEED = 1;
        public static final int STARTING_ID = 8;
        public static final int PIVOT_COUNT = 200; //dunno what this is for
        public static final int PIVOT_FRONT = 40; //change
        public static final int PIVOT_BACK = 50; //change
        public static final int NUM_LED = PIVOT_FRONT - 10;
        public static final int SPARKING = 1;
        public static final double COOLING = 0.3;
        public static final double HOLDING_SPEED = 2;
        public static final double BRIGHTNESS = 1;
        public static final int OFFSET = 5 + 55;

        public static class RainbowAnimation {
            public static final double BRIGHTNESS = 1;
            public static final double SPEED = 1;

        }

        public enum Colors {
            OFF(0,0,0,false),
            ERROR(255, 0, 0, false),
            PIECE(0, 255, 0, false),
            CONFIGURED(0,255,0,false),
            BLUE(48, 122, 171, false),
            RED(171, 48, 97, false),
            PURPLE(255, 0, 255, false),
            GREEN(0, 255, 0, false),
            ORANGE(255, 50, 0, false),
    
            FLAME(0,0,0,true),
            CHARGE(255, 0, 0, true),
            DISCHARGE(0, 0, 0, true);
    
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


