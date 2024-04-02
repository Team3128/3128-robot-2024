package frc.team3128;

import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.button.Trigger;

import static edu.wpi.first.wpilibj2.command.Commands.*;
import static frc.team3128.Constants.ShooterConstants.MAX_RPM;
import static frc.team3128.commands.CmdManager.*;

import org.photonvision.PhotonPoseEstimator.PoseStrategy;

import frc.team3128.Constants.LedConstants.Colors;
import frc.team3128.commands.CmdSwerveDrive;
import common.core.swerve.SwerveModule;
import common.hardware.camera.Camera;
import common.hardware.input.NAR_ButtonBoard;
import common.hardware.input.NAR_XboxController;
import common.hardware.input.NAR_XboxController.XboxButton;
import common.hardware.motorcontroller.NAR_CANSpark;
import common.utility.Log;
import common.utility.narwhaldashboard.NarwhalDashboard;
import common.utility.narwhaldashboard.NarwhalDashboard.State;
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
import frc.team3128.subsystems.Intake.Setpoint;

import java.util.ArrayList;

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

    // private NAR_ButtonBoard judgePad;
    private NAR_ButtonBoard buttonPad;

    public static NAR_XboxController controller;

    private NarwhalDashboard dashboard;

    private static ArrayList<Camera> sideCams = new ArrayList<Camera>();

    public RobotContainer() {
        NAR_CANSpark.maximumRetries = 3;

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
        controller.getButton(XboxButton.kB).onTrue(intake.intake(Setpoint.SOURCE));

        controller.getButton(XboxButton.kRightBumper).onTrue(moveShoot());
        controller.getButton(XboxButton.kRightTrigger).onTrue(rampUp(MAX_RPM, 12.5)).onFalse(shootDist());     //Auto Shoot
        controller.getButton(XboxButton.kY).onTrue(rampUpFeed(4000, 4000, 13)).onFalse(feed(4000, 13));   //Feed Shot
        controller.getButton(XboxButton.kX).onTrue(rampUpAmp()).onFalse(ampShoot()); //Amp Shot
        // controller.getButton(XboxButton.kX).onTrue(intake.intakePivot.pivotTo(-87)).onFalse(ampShootAlt());

        controller.getButton(XboxButton.kA).onTrue(sequence(runOnce(()-> intake.isRetracting = false), intake.intakePivot.pivotTo(-150), climber.climbTo(Climber.Setpoint.EXTENDED))); //Extend Climber
        controller.getButton(XboxButton.kBack).onTrue(sequence(climber.setClimber(-0.35), waitSeconds(1), climber.setClimber(-1), waitUntil(()->climber.isClimbed()), climber.setClimber(0)));   //Retract Climber

        controller.getButton(XboxButton.kLeftTrigger).onTrue(intake.intake(Intake.Setpoint.EXTENDED));  //Extend Intake
        controller.getButton(XboxButton.kLeftBumper).onTrue(intake.retract(false));         //Retract Intake

        controller.getButton(XboxButton.kStart).onTrue(intake.outtake()); //Amp LED

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

        buttonPad.getButton(1).onTrue(shooter.setShooter(1)).onFalse(shooter.setShooter(0));
        buttonPad.getButton(2).onTrue(ampMechanism.runPivot(0.2)).onFalse(ampMechanism.runPivot(0));
        buttonPad.getButton(3).onTrue(climber.setClimber(-0.5)).onFalse(climber.setClimber(0));
        buttonPad.getButton(4).onTrue(shooter.setShooter(1)).onFalse(shooter.setShooter(0));
        buttonPad.getButton(5).onTrue(ampMechanism.runPivot(-0.2)).onFalse(ampMechanism.runPivot(0));
        buttonPad.getButton(6).onTrue(climber.setClimber(0.5)).onFalse(climber.setClimber(0));
        buttonPad.getButton(7).onTrue(shooter.shoot(0));
        buttonPad.getButton(8).onTrue(intake.intakePivot.pivotTo(0));
        buttonPad.getButton(9).onTrue(climber.climbTo(0));
        buttonPad.getButton(10).onTrue(ampMechanism.reset(-90));
        
        buttonPad.getButton(11).onTrue(intake.intakePivot.reset(0));
        buttonPad.getButton(12).onTrue(climber.reset());
        // buttonPad.getButton(12).onTrue(runOnce(()->NAR_CANSpark.burnFlashAll()));

        // buttonPad.getButton(13).onTrue(runOnce(()-> CommandScheduler.getInstance().cancelAll()));
        // buttonPad.getButton(13).onTrue(ampMechanism.runRollers(AMP_POWER)).onFalse(ampMechanism.runRollers(0));
        // buttonPad.getButton(14).onTrue(intake.intakeRollers.runManipulator(OUTTAKE_POWER));
        // buttonPad.getButton(13).onTrue(intake.intakeRollers.intake()).onFalse(intake.intakeRollers.runManipulator(0));
        buttonPad.getButton(14).onTrue(runOnce(()-> swerve.zeroGyro(0)));

        // buttonPad.getButton(15).onTrue(runOnce(()-> autoAmpAlign().schedule()));
        buttonPad.getButton(15).onTrue(new CmdSysId("Swerve", (Double output)-> swerve.setVoltage(output), ()-> swerve.getVelocity(), swerve));
        buttonPad.getButton(16).onTrue(sequence(runOnce(()-> swerve.stop(), swerve), runOnce(()-> leds.setDefaultColor())));
        // buttonPad.getButton(15).onTrue(ampMechanism.runRollers(0.5)).onFalse(ampMechanism.runRollers(0));
        // buttonPad.getButton(13).onTrue(intake.intakeRollers.outtake()).onFalse(intake.intakeRollers.runManipulator(0));
        // buttonPad.getButton(16).onTrue(intake.intakeRollers.outtake()).onFalse(intake.intakeRollers.runManipulator(0));
        // buttonPad.getButton(16).onTrue(intake.outtake());

        new Trigger(()-> intake.intakeRollers.hasObjectPresent()).onTrue(runOnce(()-> leds.setLedColor(Colors.GREEN))).onFalse(runOnce(()-> leds.setDefaultColor()));
    }

    @SuppressWarnings("unused")
    public void initCameras() {
        Camera.disableAll();
        Camera.configCameras(AprilTagFields.k2024Crescendo, PoseStrategy.LOWEST_AMBIGUITY, (pose, time) -> swerve.addVisionMeasurement(pose, time), () -> swerve.getPose());
        Camera.setDistanceThreshold(3.5);
        Camera.setAmbiguityThreshold(0.2);
        Camera.overrideThreshold = 30;
        Camera.validDist = 0.5;
        // Camera.addIgnoredTags(13.0, 14.0);

        final Camera camera = new Camera("FRONT_LEFT", Units.inchesToMeters(10.055), Units.inchesToMeters(9.79), Units.degreesToRadians(30), Units.degreesToRadians(-28.125), 0);
        final Camera camera2 = new Camera("FRONT_RIGHT", Units.inchesToMeters(10.055), -Units.inchesToMeters(9.79), Units.degreesToRadians(-30), Units.degreesToRadians(-28.125), 0);
        // final Camera camera3 = new Camera("LEFT", Units.inchesToMeters(-3.1), Units.inchesToMeters(12.635), Units.degreesToRadians(90), Units.degreesToRadians(-10), 0);
        // final Camera camera4 = new Camera("RIGHT", Units.inchesToMeters(-3.1), Units.inchesToMeters(-12.635), Units.degreesToRadians(-90), Units.degreesToRadians(0), 0);

        // sideCams.add(camera3);
        // sideCams.add(camera4);
    }

    public static void toggleSideCams(boolean enable) {
        for (Camera camera : sideCams) {
            if (enable) camera.enable();
            else camera.disable();
        }
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
        dashboard.checkState("AmpMechanismState", ()-> ampMechanism.getRunningState());

        if (NAR_CANSpark.getNumFailedConfigs() > 0 || !isConnected()) {
            Log.recoverable("Colors", "Errors configuring: " + NAR_CANSpark.getNumFailedConfigs());
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

    public boolean isConnected() {
        for (SwerveModule module : swerve.getModules()) {
            if (module.getRunningState() != State.RUNNING) {
                Log.info("State Check", "Module " + module.moduleNumber +" failed.");
                return false;
            }
        }
        if (shooter.getRunningState() != State.RUNNING) {
            Log.info("State Check", "Shooter failed.");
            return false;
        }
        if (intake.getRunningState() != State.RUNNING) {
            Log.info("State Check", "Intake failed.");
            return false;
        }
        if (climber.getRunningState() != State.RUNNING) {
            Log.info("State Check", "Climber failed.");
            return false;
        }
        if (ampMechanism.getRunningState() != State.RUNNING) {
            Log.info("State Check", "AmpMechanism failed.");
            return false;
        }
        return true;
    }

    private void initRobotTest() {
        Tester tester = Tester.getInstance();
        tester.addTest("Robot", tester.getTest("Intake"));
        tester.addTest("Robot", tester.getTest("Shooter"));
        tester.addTest("Robot", tester.getTest("Climber"));
        tester.addTest("Robot", new UnitTest("Shoot", shoot(2500, 25)));
        tester.addTest("Robot", new UnitTest("Amp", sequence(intake.intake(Setpoint.EXTENDED), ampShoot())));
        tester.getTest("Robot").setTimeBetweenTests(0.5);
    }
}
