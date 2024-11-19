package frc.team3128;

import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.DriverStation.MatchType;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;

import static edu.wpi.first.wpilibj2.command.Commands.*;
import static frc.team3128.Constants.ShooterConstants.AMP_RPM;
import static frc.team3128.Constants.ShooterConstants.EDGE_FEED_ANGLE;
import static frc.team3128.Constants.ShooterConstants.EDGE_FEED_RPM;
import static frc.team3128.Constants.ShooterConstants.MAX_RPM;
import static frc.team3128.Constants.ShooterConstants.RAM_SHOT_RPM;
import static frc.team3128.commands.CmdManager.*;

import org.photonvision.PhotonPoseEstimator.PoseStrategy;

import frc.team3128.Constants.LedConstants.Colors;
import frc.team3128.commands.TankCommand;
import common.core.swerve.SwerveModule;
import common.hardware.camera.Camera;
import common.hardware.input.NAR_ButtonBoard;
import common.hardware.input.NAR_Joystick;
import common.hardware.input.NAR_XboxController;
import common.hardware.input.NAR_XboxController.XboxButton;
import common.hardware.motorcontroller.NAR_CANSpark;
import common.hardware.motorcontroller.NAR_TalonFX;
import common.utility.Log;
import common.utility.narwhaldashboard.NarwhalDashboard;
import common.utility.narwhaldashboard.NarwhalDashboard.State;
import common.utility.shuffleboard.NAR_Shuffleboard;
import common.utility.tester.Tester;
// import common.utility.tester.Tester.UnitTest;
import frc.team3128.subsystems.Intake;
import frc.team3128.subsystems.Tank;
import frc.team3128.subsystems.Intake.Setpoint;

/**
 * Command-based is a "declarative" paradigm, very little robot logic should
 * actually be handled in the {@link Robot} periodic methods (other than the
 * scheduler calls). Instead, the structure of the robot (including subsystems,
 * commands, and button mappings) should be declared here.
 */

public class RobotContainer {

    private Tank tank;
    private Intake intake;

    private NAR_Joystick joystick;

    public static NAR_XboxController controller;

    public static NAR_ButtonBoard buttonpad;

    private NarwhalDashboard dashboard;

    public RobotContainer() {
        NAR_CANSpark.maximumRetries = 3;
        NAR_TalonFX.maximumRetries = 1;

        NAR_Shuffleboard.WINDOW_WIDTH = 10;

        // intake.addIntakeTests();

        joystick = new NAR_Joystick(0); //change if necessary

        //uncomment line below to enable driving
        CommandScheduler.getInstance().setDefaultCommand(tank, new TankCommand(controller::getLeftY, controller::getRightX, true));

        // initRobotTest();
        
        DriverStation.silenceJoystickConnectionWarning(true);

        configureButtonBindings();
    }   

    private void configureButtonBindings() {
        // controller.getButton(XboxButton.kB).onTrue(rampUpFeed(MIDDLE_FEED_RPM, MIDDLE_FEED_RPM, 13)).onFalse(feed(MIDDLE_FEED_RPM, 13,MIDDLE_FEED_ANGLE));
        // controller.getButton(XboxButton.kY).onTrue(rampUpFeed(EDGE_FEED_RPM, EDGE_FEED_RPM, 13)).onFalse(feed(EDGE_FEED_RPM, 13, EDGE_FEED_ANGLE));   //Feed Shot
        //need to change level, intake, outtake, move
        controller.getButton(XboxButton.kLeftTrigger).onTrue(intake.outtake());  // Outtakes
        controller.getButton(XboxButton.kLeftBumper).onTrue(intake.intake(Setpoint.INTAKING));  // Intakes
        controller.getButton(XboxButton.kLeftBumper).toggleOnFalse(intake.intakePivot.pivotTo(Setpoint.LOWPOLE.angle));

        controller.getButton(XboxButton.kRightTrigger).onTrue(intake.intakePivot.pivotTo(intake.GetSetpointUp(intake.intakePivot.getSetpoint()).angle));  //1 up
        controller.getButton(XboxButton.kRightBumper).onTrue(intake.intakePivot.pivotTo(intake.GetSetpointDown(intake.intakePivot.getSetpoint()).angle));  //1 down

        controller.getButton(XboxButton.kStart).onTrue(intake.intakePivot.reset()); //reset

        //controller.getButton(XboxButton.kRightStick).onTrue(runOnce(()-> TankCommand.setTurnSetpoint(0))); //setTurnpoint originally did not have 0 as parameter (took nothing idk why)

        //pov buttons? tyler
        controller.getUpPOVButton().onTrue(runOnce(()-> {
            TankCommand.setTurnSetpoint(Robot.getAlliance() == Alliance.Red ? 180 : 0);
        }));
        controller.getDownPOVButton().onTrue(runOnce(()-> {
            TankCommand.setTurnSetpoint(Robot.getAlliance() == Alliance.Red ? 0 : 180);
        }));

        controller.getRightPOVButton().onTrue(runOnce(()-> {
            TankCommand.setTurnSetpoint(Robot.getAlliance() == Alliance.Red ? 90 : 270);
        }));

        controller.getLeftPOVButton().onTrue(runOnce(()-> {
            TankCommand.setTurnSetpoint(Robot.getAlliance() == Alliance.Red ? 270 : 90);
        }));

        buttonpad.getButton(12).onTrue(intake.intake(Intake.Setpoint.HIGHPOLE));
        // buttonPad example: buttonPad.getButton(12).onTrue(runOnce(()->NAR_CANSpark.burnFlashAll()));
    }

    public void initDashboard() {
        dashboard = NarwhalDashboard.getInstance();
        dashboard.addUpdate("time", ()-> Timer.getMatchTime());
        dashboard.addUpdate("voltage",()-> RobotController.getBatteryVoltage());
        dashboard.addUpdate("robotX", ()-> tank.getPose().getX());
        dashboard.addUpdate("robotY", ()-> tank.getPose().getY());
        dashboard.addUpdate("robotYaw", ()-> tank.getPose().getRotation().getDegrees());
        dashboard.checkState("IntakeState", ()-> intake.getRunningState());
        dashboard.addUpdate("driveLimit", ()-> tank.getdriveLimit());
        dashboard.addUpdate("offset", ()-> tank.getOffSet());

        if (NAR_TalonFX.getNumFailedConfigs() + NAR_CANSpark.getNumFailedConfigs() > 0 || !isConnected()) {
            Log.recoverable("Colors", "Errors configuring: " + NAR_CANSpark.getNumFailedConfigs() + NAR_TalonFX.getNumFailedConfigs());
        }
        else if (!tank.isConfigured()) {
            Log.info("Colors", "Tank Not Configured");
        }
        else {
            Log.info("Colors", "No errors configuring");
        }
    }

    public boolean isConnected() {
        for (SwerveModule module : tank.getModules()) {
            if (module.getRunningState() != State.RUNNING) {
                Log.info("State Check", "Module " + module.moduleNumber +" failed.");
                return false;
            }
        }

        if (intake.getRunningState() != State.RUNNING) {
            Log.info("State Check", "Intake failed.");
            return false;
        }

        return true;
    }

    // private void initRobotTest() {
    //     Tester tester = Tester.getInstance();
    //     tester.addTest("Robot", tester.getTest("Intake"));
    //     tester.addTest("Robot", tester.getTest("Shooter"));
    //     tester.addTest("Robot", tester.getTest("Climber"));
    //     tester.addTest("Robot", new UnitTest("Shoot", shoot(2500, 25)));
    //     tester.addTest("Robot", new UnitTest("Amp", sequence(intake.intake(Setpoint.EXTENDED), ampShoot())));
    //     tester.getTest("Robot").setTimeBetweenTests(0.5);
    // }
}
