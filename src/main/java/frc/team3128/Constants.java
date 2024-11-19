package frc.team3128;

import java.util.HashMap;

import com.pathplanner.lib.path.PathConstraints;

import common.core.controllers.Controller;
import common.core.controllers.PIDFFConfig;
import common.core.controllers.Controller.Type;
import common.core.swerve.SwerveConversions;
import common.core.swerve.SwerveModuleConfig;
import common.core.swerve.SwerveModuleConfig.SwerveMotorConfig;
import common.hardware.motorcontroller.NAR_CANSpark;
import common.hardware.motorcontroller.NAR_TalonFX;
import common.hardware.motorcontroller.NAR_TalonSRX;
import common.hardware.motorcontroller.NAR_CANSpark.ControllerType;
import common.hardware.motorcontroller.NAR_Motor.MotorConfig;
import common.hardware.motorcontroller.NAR_Motor.Neutral;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;


public class Constants {

    public static class AutoConstants {

        public static final double slowSpeed = 1.5;
        public static final double slowAcceleration = 2;

        // public static final PathConstraints constraints = new PathConstraints(
        //     SwerveConstants.maxSpeed, SwerveConstants.maxAcceleration, SwerveConstants.maxAngularVelocity, SwerveConstants.maxAngularAcceleration); 

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
        public static final Pose2d SPEAKER = new Pose2d(Units.inchesToMeters(324.5), Units.inchesToMeters(315.5), Rotation2d.fromDegrees(0));


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

    public static class FocalAimConstants {
        public static final double speakerLength = 1.043;
        public static final double speakerMidpointY = Units.inchesToMeters(218.29);//5.4;
        
        ; //6.151 - speakerLength / 2;
        public static final double focalPointX = 0.1; //0.229; //1.4583577128;
        public static final Translation2d speakerMidpointBlue = new Translation2d(0, speakerMidpointY);
        public static final Translation2d speakerMidpointRed = new Translation2d(FieldConstants.FIELD_X_LENGTH, speakerMidpointY);
        public static final Translation2d focalPointBlue = new Translation2d(focalPointX, speakerMidpointY);
        public static final Translation2d focalPointRed = new Translation2d(FieldConstants.FIELD_X_LENGTH - focalPointX, speakerMidpointY);
        public static final double angleOffset = 0;
        //testing: kV: drivetrain spinning consistently (ie. v1 = vel at  vel at 1 rad/sec v2=2 rad/sec). 1/(v2-v1) = kV
        //kS: plug kV into 1= kS + kV(v1)
        public static final double offset = 0.3;
        public static final double lowerBound = speakerMidpointY - offset;
        public static final double higherBound = speakerMidpointY + offset;
    }

    public static class ShooterConstants {
        public static final PIDFFConfig PIDConstants = new PIDFFConfig(0.0025, 0, 0, 0, 0.00179104, 0); // 0.00187623
        public static final double kF = 0.3582; //0.144578;
        public static final int LEFT_MOTOR_ID = 41;
        public static final int RIGHT_MOTOR_ID = 42;
        public static final double GEAR_RATIO = 1;
        public static final double MAX_RPM = 5500;
        public static final double MIN_RPM = 0;
        public static final double TOLERANCE = 150;
        public static final double AMP_RPM = 2500;
        public static final double RAM_SHOT_RPM = 4500;
        
        public static final double EDGE_FEED_RPM = 5000;
        public static final double EDGE_FEED_ANGLE = 35;
        public static final double MIDDLE_FEED_RPM = 4500;
        public static final double MIDDLE_FEED_ANGLE = 25;
        

        public static final double CURRENT_TEST_POWER = 0;
        public static final double CURRENT_TEST_PLATEAU = 0;
        public static final double CURRENT_TEST_TIMEOUT = 0;
        public static final double CURRENT_TEST_TOLERANCE = 0;
        public static final double CURRENT_TEST_EXPECTED_CURRENT = 0;

        public static final double SHOOTER_TEST_PLATEAU = 1;
        public static final double SHOOTER_TEST_TIMEOUT = 2.5;

        public static final double PROJECTILE_SPEED = 100; // m/s
    }

    public static class AmpWristConstants {
        public static final PIDFFConfig PIDConstants = new PIDFFConfig(0.25, 0, 0, 0.08, 0, 0.22);
        public static final double MAX_VELOCITY = 10000000;
        public static final double MAX_ACCELERATION = 1000000;
        public static final Constraints TRAP_CONSTRAINTS = new Constraints(MAX_VELOCITY, MAX_ACCELERATION);

        public static final int WRIST_MOTOR_ID = 53;
        public static final NAR_CANSpark WRIST_MOTOR = new NAR_CANSpark(WRIST_MOTOR_ID);
        public static final double GEAR_RATIO = 1.0 / 70.875;
        public static final double CURRENT_LIMIT = 20;
        public static final double POSITION_TOLERANCE = 1;

        public static final int ROLLER_MOTOR_ID = 52;
        public static final NAR_TalonSRX ROLLER_MOTOR = new NAR_TalonSRX(ROLLER_MOTOR_ID);
        public static final double AMP_POWER = 0.8;

        public static final double EXTEND_TIMEOUT = 1;
        public static final double RETRACTED_TIMEOUT = 1;

        public static final double ROLLER_TIMEOUT = 5;
        public static final double ROLLER_TEST_PLATEAU = 0.5;
        public static final double ROLLER_TEST_EXPECTED_CURRENT = 1.25;
    }
    public static class ClimberConstants {
        public static final PIDFFConfig PIDConstants = new PIDFFConfig(2, 0, 0, 0.18, 0, 0, 0.3);//240
        public static final double MAX_VELOCTIY = 10000000;
        public static final double MAX_ACCELERATION = 100000;
        public static final Constraints TRAP_CONSTRAINTS = new Constraints(MAX_VELOCTIY, MAX_ACCELERATION);
        public static final int LEFT_MOTOR_ID = 21;
        public static final int RIGHT_MOTOR_ID = 22;
        public static final double GEAR_RATIO = 1.0 / 15.0;
        public static final double WHEEL_CIRCUMFERENCE = Units.inchesToMeters(1.751) * Math.PI;
        public static final double POSITION_TOLERANCE = 0.5;
        public static final double PIVOT_CLIMBER_DIST = 28;
        public static final double POSITION_MINIMUM = 0;
        public static final double POSITION_MAXIMUM = 30;
        public static final double HEIGHT_OFFSET = 7; // 14 degrees ish
        public static final double ANGLE_OFFSET = 14;
        

        public static final double NEUTRAL_THRESHOLD = 1;
        public static final InterpolatingDoubleTreeMap climberHeightMap = new InterpolatingDoubleTreeMap();
        static {
            climberHeightMap.put(0.0 + 0.93, 25.0);
            climberHeightMap.put(0.25 + 0.93, 25.0);
            climberHeightMap.put(0.5 + 0.93, 20.0);
            climberHeightMap.put(0.75 + 0.93, 17.0);
            climberHeightMap.put(1.0 + 0.93, 15.0);
            climberHeightMap.put(1.25 + 0.93, 14.0);
            climberHeightMap.put(1.5 + 0.93, 13.0);
            climberHeightMap.put(1.75 + 0.93, 12.0);
            climberHeightMap.put(2.0 + 0.93, 11.25);
            climberHeightMap.put(2.25 + 0.93, 10.0);
            climberHeightMap.put(2.5 + 0.93, 9.7);
            climberHeightMap.put(2.75 + 0.93, 9.5);
            climberHeightMap.put(3.0 + 0.93, 9.5);
        }

        public static final double SETPOINT_TEST_TIMEOUT_EXTEND = 1;
        public static final double SETPOINT_TEST_TIMEOUT_RETRACT = 1;
    }

    public static class IntakeConstants {
        public static final PIDFFConfig PIDConstants = new PIDFFConfig(0.1, 0, 0, 0.2, 0, 0, 0.3625);
        public static final int PIVOT_MOTOR_ID = 31;
        public static final NAR_CANSpark PIVOT_MOTOR = new NAR_CANSpark(PIVOT_MOTOR_ID);
        public static final double GEAR_RATIO = 1.0 / 40.0;
        public static final double MAX_VELOCITY = 1000000;
        public static final double MAX_ACCELERATION = 100000;
        public static final Constraints TRAP_CONSTRAINTS = new Constraints(MAX_VELOCITY, MAX_ACCELERATION);
        public static final double POSITION_MINIMUM = 0;
        public static final double POSITION_MAXIMUM = 220;

        public static final int RIGHT_ROLLER_MOTOR_ID = 32;
        public static final int LEFT_ROLLER_MOTOR_ID = 33;
        public static final NAR_CANSpark RIGHT_ROLLER_MOTOR = new NAR_CANSpark(RIGHT_ROLLER_MOTOR_ID);
        public static final NAR_CANSpark LEFT_ROLLER_MOTOR = new NAR_CANSpark(LEFT_ROLLER_MOTOR_ID);
        public static final double ANGLE_TOLERANCE = 3;
        public static final int CURRENT_LIMIT = 40;
        public static final double STALL_CURRENT = 50;
        public static final double STALL_POWER = .05;
        public static final double OUTTAKE_POWER = -1;
        public static final double INTAKE_POWER = 0.7 /0.75;
        public static final double AMP_POWER = -0.18 / 0.75;
        public static final double CURRENT_THRESHHOLD = 50; //find through testing

        public static final int ROLLER_MOTOR_ID = 2;
        public static final NAR_CANSpark ROLLER_MOTOR = new NAR_CANSpark(ROLLER_MOTOR_ID);

        public static final double CURRENT_TEST_POWER = OUTTAKE_POWER;
        public static final double CURRENT_TEST_PLATEAU = 1;
        public static final double CURRENT_TEST_TIMEOUT = 5;
        public static final double CURRENT_TEST_TOLERANCE = 40;
        public static final double CURRENT_TEST_EXPECTED_CURRENT = 12.5;

        public static final double SETPOINT_TEST_PLATEAU = 1;
        public static final double SETPOINT_TEST_TIMEOUT = 3;

        public static final double OUTTAKE_TIMEOUT = 0.35;

        public static final double INTAKE_TEST_TIMEOUT = 30;
    }

    public static class LimelightConstants {
        public static final double TX_THRESHOLD = 1;
        public static final double HORIZONTAL_OFFSET_GOAL = 0;
        public static final double PLATEAU_THRESHOLD = 5;
        public static final double TIMEOUT = 1;
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
            DISCHARGE(0, 0, 0, true),
            AMP(0,0,0,true);
    
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


