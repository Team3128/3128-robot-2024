package frc.team3128;

import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import static edu.wpi.first.wpilibj2.command.Commands.*;
import static frc.team3128.Constants.ShooterConstants.MAX_RPM;
import static frc.team3128.commands.CmdManager.*;

import org.photonvision.PhotonPoseEstimator.PoseStrategy;

import frc.team3128.Constants.ShooterConstants;
import frc.team3128.Constants.LedConstants.Colors;
import frc.team3128.commands.CmdSwerveDrive;
import common.core.misc.NAR_Robot;
import common.hardware.camera.Camera;
import common.hardware.input.NAR_ButtonBoard;
import common.hardware.input.NAR_XboxController;
import common.hardware.input.NAR_XboxController.XboxButton;
import common.hardware.motorcontroller.NAR_CANSpark;
import common.utility.Log;
import common.utility.narwhaldashboard.NarwhalDashboard;
import common.utility.shuffleboard.NAR_Shuffleboard;
import common.utility.sysid.CmdSysId;
import common.utility.tester.Tester;
import common.utility.tester.Tester.UnitTest;
import frc.team3128.subsystems.AmpMechanism;
import frc.team3128.subsystems.Climber;
import frc.team3128.subsystems.Intake;
import frc.team3128.subsystems.Leds;
import frc.team3128.subsystems.Shooter;
import frc.team3128.subsystems.Swerve;
import java.util.LinkedList;

/**
 * Command-based is a "declarative" paradigm, very little robot logic should
 * actually be handled in the {@link Robot} periodic methods (other than the
 * scheduler calls). Instead, the structure of the robot (including subsystems,
 * commands, and button mappings) should be declared here.
 */
public class RobotContainer {

    private Swerve swerve;
    private Shooter shooter;
    private AmpMechanism ampMechanism;
    private Climber climber;
    private Intake intake;
    private Leds leds;

    private NAR_ButtonBoard judgePad;
    private NAR_ButtonBoard buttonPad;

    public static NAR_XboxController controller;

    private NarwhalDashboard dashboard;

    public RobotContainer() {
        NAR_CANSpark.maximumRetries = 3;

        NAR_Robot.logWithAdvantageKit = true;
        NAR_Shuffleboard.WINDOW_WIDTH = 10;

        swerve = Swerve.getInstance();
        shooter = Shooter.getInstance();
        ampMechanism = AmpMechanism.getInstance();
        climber = Climber.getInstance();
        intake = Intake.getInstance();
        leds = Leds.getInstance();

        shooter.addShooterTests();
        climber.addClimberTests();
        intake.addIntakeTests();

        // judgePad = new NAR_ButtonBoard(1);
        controller = new NAR_XboxController(2);
        buttonPad = new NAR_ButtonBoard(3);

        //uncomment line below to enable driving
        CommandScheduler.getInstance().setDefaultCommand(swerve, new CmdSwerveDrive(controller::getLeftX,controller::getLeftY, controller::getRightX, true));
        configureButtonBindings();

        initRobotTest();
        
        DriverStation.silenceJoystickConnectionWarning(true);
        initCameras();
    }   

    private void configureButtonBindings() {
        controller.getButton(XboxButton.kB).onTrue(runOnce(()-> swerve.resetEncoders()));

        controller.getButton(XboxButton.kRightBumper).onTrue(rampUp(MAX_RPM, 25)).onFalse(shoot(MAX_RPM, 25)); //Ram Shot
        controller.getButton(XboxButton.kRightTrigger).onTrue(rampUpContinuous()).onFalse(autoShoot());     //Auto Shoot
        controller.getButton(XboxButton.kY).onTrue(rampUp(5000, 15)).onFalse(feed(5000, 15));   //Feed Shot
        // controller.getButton(XboxButton.kX).onTrue(intake.intakePivot.pivotTo(-55)).onFalse(ampShoot()); //Amp Shot
        // controller.getButton(XboxButton.kX).onTrue(intake.intakePivot.pivotTo(-87)).onFalse(ampShootAlt());

        controller.getButton(XboxButton.kA).onTrue(sequence(runOnce(()-> intake.isRetracting = false), intake.intakePivot.pivotTo(-150), climber.climbTo(Climber.Setpoint.EXTENDED))); //Extend Climber
        controller.getButton(XboxButton.kBack).onTrue(sequence(climber.setClimber(-0.35), waitSeconds(1), climber.setClimber(-1), waitUntil(()->climber.isClimbed()), climber.setClimber(0)));   //Retract Climber

        controller.getButton(XboxButton.kLeftTrigger).onTrue(intake.intake(Intake.Setpoint.EXTENDED));  //Extend Intake
        controller.getButton(XboxButton.kLeftBumper).onTrue(intake.retract(false));         //Retract Intake

        controller.getButton(XboxButton.kStart).onTrue(startEnd(()-> leds.setLedColor(Colors.AMP), ()-> leds.setDefaultColor()).withTimeout(1)); //Amp LED

        controller.getButton(XboxButton.kRightStick).onTrue(runOnce(()-> CmdSwerveDrive.setTurnSetpoint()));
        controller.getUpPOVButton().onTrue(runOnce(()-> {
            CmdSwerveDrive.setTurnSetpoint(Robot.getAlliance() == Alliance.Red ? 180 : 0);
        }));
        controller.getDownPOVButton().onTrue(runOnce(()-> {
            CmdSwerveDrive.setTurnSetpoint(Robot.getAlliance() == Alliance.Red ? 0 : 180);
        }));

        controller.getRightPOVButton().onTrue(runOnce(()-> {
            CmdSwerveDrive.setTurnSetpoint(Robot.getAlliance() == Alliance.Red ? 90 : 270);
        }));

        controller.getLeftPOVButton().onTrue(runOnce(()-> {
            CmdSwerveDrive.setTurnSetpoint(Robot.getAlliance() == Alliance.Red ? 270 : 90);
        }));

        // judgePad.getButton(1).onTrue(runOnce(()-> swerve.resetOdometry(new Pose2d(5, 5, Rotation2d.fromDegrees(180)))));

        // rightStick.getButton(1).onTrue(runOnce(()-> swerve.zeroGyro(0)));
        // rightStick.getButton(3).onTrue(new CmdSysId("Rotation", (Double radiansPerSec) -> swerve.drive(new Translation2d(), radiansPerSec, true), ()-> Units.radiansToDegrees(swerve.getRobotVelocity().omegaRadiansPerSecond), swerve));
        // rightStick.getButton(4).onTrue(runOnce(()-> swerve.resetOdometry(new Pose2d(1.35, FocalAimConstants.speakerMidpointY, Rotation2d.fromDegrees(180)))));
        // rightStick.getButton(5).onTrue(runOnce(()-> swerve.resetOdometry(new Pose2d(FIELD_X_LENGTH - 1.35, FocalAimConstants.speakerMidpointY, Rotation2d.fromDegrees(0)))));
        // rightStick.getButton(6).onTrue(swerve.turnInPlace(()-> 0)); 
        
        // rightStick.getButton(2).onTrue(shooter.setShooter(0.8)).onFalse(shooter.setShooter(0));
        // rightStick.getButton(3).onTrue(shooter.shoot(0));
        // rightStick.getButton(4).onTrue(climber.setClimber(0.2)).onFalse(climber.setClimber(0));
        // rightStick.getButton(5).onTrue(climber.setClimber(-0.2)).onFalse(climber.setClimber(0));
        // rightStick.getButton(6).onTrue(climber.climbTo(0));
        // rightStick.getButton(7).onTrue(climber.reset());
        // rightStick.getButton(8).onTrue(intake.setPivot(0.2)).onFalse(intake.setPivot(0));
        // rightStick.getButton(9).onTrue(intake.setPivot(-0.2)).onFalse(intake.setPivot(0));
        // rightStick.getButton(10).onTrue(intake.pivotTo(180));
        // rightStick.getButton(11).onTrue(intake.reset());
        // rightStick.getButton(12).onTrue(intake.setRoller(0.5)).onFalse(intake.setRoller(0));
        // rightStick.getButton(13).onTrue(intake.setRoller(IntakeConstants.OUTTAKE_POWER)).onFalse(intake.setRoller(0));
        // rightStick.getButton(7).onTrue(new CmdSysId("Swerve", (Double volts)-> swerve.setVoltage(volts), ()-> swerve.getVelocity(), swerve)).onFalse(runOnce(()-> swerve.stop(), swerve));
        // rightStick.getButton(8).onTrue(runOnce(()-> NAR_CANSpark.burnFlashAll()));


        buttonPad.getButton(1).onTrue(shooter.setShooter(-0.8)).onFalse(shooter.setShooter(0));
        buttonPad.getButton(2).onTrue(ampMechanism.runPivot(1)).onFalse(ampMechanism.runPivot(0));
        buttonPad.getButton(3).onTrue(climber.setClimber(-1)).onFalse(climber.setClimber(0));
        buttonPad.getButton(4).onTrue(shooter.setShooter(0.8)).onFalse(shooter.setShooter(0));
        buttonPad.getButton(5).onTrue(ampMechanism.runPivot(-1)).onFalse(ampMechanism.runPivot(0));
        buttonPad.getButton(6).onTrue(climber.setClimber(1)).onFalse(climber.setClimber(0));
        buttonPad.getButton(7).onTrue(shooter.shoot(0));
        buttonPad.getButton(8).onTrue(ampMechanism.pivotTo(0));
        buttonPad.getButton(9).onTrue(climber.climbTo(0));
        buttonPad.getButton(10).onTrue(neutral(false));
        
        buttonPad.getButton(11).onTrue(ampMechanism.reset(0));
        buttonPad.getButton(12).onTrue(climber.reset());

        buttonPad.getButton(13).onTrue(runOnce(()-> CommandScheduler.getInstance().cancelAll()));
        buttonPad.getButton(14).onTrue(runOnce(()-> swerve.zeroGyro(0)));
        buttonPad.getButton(15).onTrue(ampMechanism.runRollers(0.5)).onFalse(ampMechanism.runRollers(0));
        // buttonPad.getButton(16).onTrue(intake.intakeRollers.outtake()).onFalse(intake.intakeRollers.runManipulator(0));
        buttonPad.getButton(16).onTrue(intake.outtake());
    }

    @SuppressWarnings("unused")
    public void initCameras() {
        Camera.configCameras(AprilTagFields.k2024Crescendo, PoseStrategy.LOWEST_AMBIGUITY, (pose, time) -> swerve.addVisionMeasurement(pose, time), () -> swerve.getPose());
        Camera.setDistanceThreshold(3.5);
        Camera.setAmbiguityThreshold(0.2);

        final LinkedList<Double> blacklist = new LinkedList<Double>();
        blacklist.add(1.0);
        blacklist.add(2.0);
        blacklist.add(9.0);
        blacklist.add(10.0);
        // Camera.setIgnoredTags(blacklist);

        final Camera camera = new Camera("FRONT_LEFT", Units.inchesToMeters(10.055), Units.inchesToMeters(9.79), Units.degreesToRadians(30), Units.degreesToRadians(-28.125), 0);
        final Camera camera2 = new Camera("FRONT_RIGHT", Units.inchesToMeters(10.055), -Units.inchesToMeters(9.79), Units.degreesToRadians(-30), Units.degreesToRadians(-28.125), 0);
        final Camera camera3 = new Camera("LEFT", Units.inchesToMeters(-3.1), Units.inchesToMeters(12.635), Units.degreesToRadians(90), Units.degreesToRadians(-10), 0);
        final Camera camera4 = new Camera("RIGHT", Units.inchesToMeters(-3.1), Units.inchesToMeters(-12.635), Units.degreesToRadians(-90), Units.degreesToRadians(0), 0);
    }

    public void initDashboard() {
        dashboard = NarwhalDashboard.getInstance();
        dashboard.addUpdate("time", ()-> Timer.getMatchTime());
        dashboard.addUpdate("voltage",()-> RobotController.getBatteryVoltage());
        dashboard.addUpdate("robotX", ()-> swerve.getPose().getX());
        dashboard.addUpdate("robotY", ()-> swerve.getPose().getY());
        dashboard.addUpdate("robotYaw", ()-> swerve.getPose().getRotation().getDegrees());
        dashboard.checkState("IntakeState", ()-> intake.getRunningState());
        dashboard.checkState("ClimberState", ()-> climber.getRunningState());
        dashboard.checkState("ShooterState", ()-> shooter.getRunningState());

        if (NAR_CANSpark.getNumFailedConfigs() > 0) {
            Log.info("Colors", "Errors configuring: " + NAR_CANSpark.getNumFailedConfigs());
            Leds.getInstance().setLedColor(Colors.ERROR);
        }
        else if (!swerve.isConfigured()) {
            Log.info("Colors", "Swerve Not Configured");
            Leds.getInstance().setLedColor(Colors.RED);
        }
        else {
            Log.info("Colors", "Errors configuring: " + NAR_CANSpark.getNumFailedConfigs());
            Leds.getInstance().setLedColor(Colors.CONFIGURED);
        }
    }

    private void initRobotTest() {
        Tester tester = Tester.getInstance();
        tester.addTest("Robot", tester.getTest("Intake"));
        tester.addTest("Robot", tester.getTest("Shooter"));
        tester.addTest("Robot", tester.getTest("Climber"));
        tester.addTest("Robot", new UnitTest("Shoot", shoot(2500, 25)));
        tester.getTest("Robot").setTimeBetweenTests(1);
    }
}
