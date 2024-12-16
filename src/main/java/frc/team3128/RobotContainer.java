package frc.team3128;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import static edu.wpi.first.wpilibj2.command.Commands.sequence;
import static edu.wpi.first.wpilibj2.command.Commands.waitSeconds;

import common.hardware.input.NAR_XboxController;
import common.hardware.input.NAR_XboxController.XboxButton;
import common.hardware.limelight.Limelight;
import common.hardware.motorcontroller.NAR_CANSpark;
import common.hardware.motorcontroller.NAR_TalonFX;
import common.utility.Log;
import common.utility.narwhaldashboard.NarwhalDashboard;
import common.utility.shuffleboard.NAR_Shuffleboard;
import frc.team3128.autonomous.PathManager;
// import common.utility.tester.Tester.UnitTest;
// import frc.team3128.subsystems.Leds;
import frc.team3128.subsystems.Swerve;
// import frc.team3128.subsystems.Amper.Elevator;
// import frc.team3128.subsystems.Amper.Amper;
import frc.team3128.subsystems.Amper.AmperExposed;
import frc.team3128.subsystems.Amper.AmperStates;

/**
 * Command-based is a "declarative" paradigm, very little robot logic should
 * actually be handled in the {@link Robot} periodic methods (other than the
 * scheduler calls). Instead, the structure of the robot (including subsystems,
 * commands, and button mappings) should be declared here.
 */
public class RobotContainer {

    private Swerve swerve;
    // private Leds leds;
    // private Elevator elevator;
    // Amper amper;
    AmperExposed amperExposed;


    public static NAR_XboxController controller, controller2;

    private NarwhalDashboard dashboard;

    public static Limelight limelight;

    public RobotContainer() {
        NAR_CANSpark.maximumRetries = 2;
        NAR_TalonFX.maximumRetries = 2;

        NAR_Shuffleboard.WINDOW_WIDTH = 10;

        swerve = Swerve.getInstance();
        // amper = Amper.getInstance();
        amperExposed = AmperExposed.getInstance();
        // leds = Leds.getInstance();
        // elevator = new Elevator();


        controller = new NAR_XboxController(2);
        controller2 = new NAR_XboxController(3);
        // buttonPad = new NAR_ButtonBoard(4);
        

        //uncomment line below to enable driving
        CommandScheduler.getInstance().setDefaultCommand(swerve, swerve.getDriveCommand(controller::getLeftX,controller::getLeftY, controller::getRightX));
        
        DriverStation.silenceJoystickConnectionWarning(true);
        // initCameras();

        configureButtonBindings();

        var x = NAR_Shuffleboard.debug("Debug Test", "Debug Toggle", false, 0, 0);
        // NAR_Shuffleboard.addData("Debug Test", "Debug Value", x);
        // NAR_Shuffleboard.addData("Limelight", "ValidTarget", ()-> limelight.hasValidTarget(), 0, 0);
        // NAR_Shuffleboard.addData("Limelight", "TX", ()-> limelight.getValue(LimelightKey.HORIZONTAL_OFFSET), 0, 1);
        PathManager.getInstance().topOnePiece().schedule();
    }   

    private void configureButtonBindings() {
        // controller.getButton(XboxButton.kX)
        //     .onTrue(Commands.runOnce(() -> Swerve.getInstance().resetGyroTo(0)));
        // controller.getButton(XboxButton.kRightTrigger)
        //     .onTrue(Commands.runOnce(() -> Swerve.getInstance().rotateTo(new Translation2d())));
        // controller.getButton(XboxButton.kX)
        //     .onTrue(sequence(
        //         Commands.runOnce(()-> amper.setState(AmperStates.EXTENDED))
        //     ))
        //     .onFalse(sequence(
        //         Commands.runOnce(()-> amper.setState(AmperStates.IDLE))
        //     ));

        controller.getButton(XboxButton.kY)
            .onTrue(Commands.runOnce(()-> amperExposed.pidTo(AmperStates.EXTENDED)))
            .onFalse(Commands.runOnce(()-> amperExposed.pidTo(AmperStates.IDLE))
        );

        controller.getButton(XboxButton.kB)
            .onTrue(Commands.runOnce(()-> amperExposed.setState(AmperStates.PRIMED)))
            .onFalse(Commands.runOnce(()-> amperExposed.setState(AmperStates.EXTENDED))
                                .andThen(waitSeconds(1))
                                .andThen(()-> amperExposed.setState(AmperStates.IDLE))
        );
        
        // control
        // controller.getButton(XboxButton.kA)
        //     .onTrue(amper.pidTo(AmperStates.EXTENDED))
        //     .onFalse(amper.pidTo(AmperStates.IDLE));
    }

    public void initCameras() {
        // Camera1.setResources(() -> swerve.getYaw(), (pose,time)->swerve.addVisionMeasurement(pose, time), AprilTagFields.k2024Crescendo.loadAprilTagLayoutField(), ()->swerve.getPose());
        // Camera1.addIgnoredTags(14);
        // if (Robot.isReal()) {
        //     // final Camera camera = new Camera("FRONT_LEFT", Units.inchesToMeters(10.055), Units.inchesToMeters(9.79), Units.degreesToRadians(30), Units.degreesToRadians(-28.125), 0);
        //     // final Camera camera2 = new Camera("FRONT_RIGHT", Units.inchesToMeters(10.055), -Units.inchesToMeters(9.79), Units.degreesToRadians(-30), Units.degreesToRadians(-28.125), 0);
        //     // camera.setCamDistanceThreshold(3.5);
        //     // camera2.setCamDistanceThreshold(5);
        //     final Camera1 camera = new Camera1("FRONT_LEFT", Units.inchesToMeters(10.055), Units.inchesToMeters(9.79), Units.degreesToRadians(30), Units.degreesToRadians(-28.125), 0);
        //     final Camera1 camera2 = new Camera1("FRONT_RIGHT", Units.inchesToMeters(10.055), -Units.inchesToMeters(9.79), Units.degreesToRadians(-30), Units.degreesToRadians(-28.125), 0);
        // }

        // limelight = new Limelight("limelight-mason", 0, 0, 0, 0);
    }

    public void initDashboard() {
        // dashboard = NarwhalDashboard.getInstance();
        // dashboard.addUpdate("time", ()-> Timer.getMatchTime());
        // dashboard.addUpdate("voltage",()-> RobotController.getBatteryVoltage());
        // dashboard.addUpdate("robotX", ()-> swerve.getPose().getX());
        // dashboard.addUpdate("robotY", ()-> swerve.getPose().getY());
        // dashboard.addUpdate("robotYaw", ()-> swerve.getPose().getRotation().getDegrees());
        // dashboard.checkState("IntakeState", ()-> intake.getRunningState());
        // dashboard.checkState("ClimberState", ()-> climber.getRunningState());
        // dashboard.checkState("ShooterState", ()-> shooter.getRunningState());
        // dashboard.checkState("AmpMechanismState", ()-> ampMechanism.getRunningState());
        // dashboard.addUpdate("driveLimit", ()-> swerve.getdriveLimit());
        // dashboard.addUpdate("offset", ()-> swerve.getOffSet());

        // if (NAR_TalonFX.getNumFailedConfigs() + NAR_CANSpark.getNumFailedConfigs() > 0) {
        //     Log.recoverable("Colors", "Errors configuring: " + NAR_CANSpark.getNumFailedConfigs() + NAR_TalonFX.getNumFailedConfigs());
        //     Leds.getInstance().setLedColor(Colors.ERROR);
        // }
        // else if (!swerve.isConfigured()) {
        //     Log.info("Colors", "Swerve Not Configured");
        //     Leds.getInstance().setLedColor(Colors.RED);
        // }
        // else {
        //     Log.info("Colors", "No errors configuring");
        //     Leds.getInstance().setLedColor(Colors.CONFIGURED);
        // }
    }
}
