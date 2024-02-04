package frc.team3128;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import static edu.wpi.first.wpilibj2.command.Commands.*;
import static frc.team3128.commands.CmdManager.*;

import frc.team3128.Constants.IntakeConstants;
import frc.team3128.commands.CmdSwerveDrive;
import common.hardware.input.NAR_ButtonBoard;
import common.hardware.input.NAR_Joystick;
import common.hardware.input.NAR_XboxController;
import common.hardware.input.NAR_XboxController.XboxButton;
import common.utility.narwhaldashboard.NarwhalDashboard;
import common.utility.shuffleboard.NAR_Shuffleboard;
import common.utility.sysid.CmdSysId;
import frc.team3128.subsystems.Climber;
import frc.team3128.subsystems.Intake;
import frc.team3128.subsystems.Shooter;
import frc.team3128.subsystems.Swerve;

/**
 * Command-based is a "declarative" paradigm, very little robot logic should
 * actually be handled in the {@link Robot} periodic methods (other than the
 * scheduler calls). Instead, the structure of the robot (including subsystems,
 * commands, and button mappings) should be declared here.
 */
public class RobotContainer {

    private Swerve swerve;
    private Shooter shooter;
    private Climber climber;
    private Intake intake;

    private NAR_Joystick rightStick;
    private NAR_ButtonBoard buttonPad;

    public static NAR_XboxController controller;

    private NarwhalDashboard dashboard;

    public RobotContainer() {
        NAR_Shuffleboard.WINDOW_WIDTH = 10;

        swerve = Swerve.getInstance();
        shooter = Shooter.getInstance();
        climber = Climber.getInstance();
        intake = Intake.getInstance();

        rightStick = new NAR_Joystick(1);
        controller = new NAR_XboxController(2);
        buttonPad = new NAR_ButtonBoard(3);

        //uncomment line below to enable driving
        CommandScheduler.getInstance().setDefaultCommand(swerve, new CmdSwerveDrive(controller::getLeftX,controller::getLeftY, controller::getRightX, true));
        configureButtonBindings();
        
        DriverStation.silenceJoystickConnectionWarning(true);
    }   

    private void configureButtonBindings() {
        controller.getButton(XboxButton.kB).onTrue(runOnce(()-> swerve.resetEncoders()));
        controller.getButton(XboxButton.kRightTrigger).onTrue(shoot());
        controller.getButton(XboxButton.kRightBumper).onTrue(shootRam());
        controller.getButton(XboxButton.kY).onTrue(shoot()); //TODO: AMP 
        controller.getButton(XboxButton.kB).onTrue(climber.climbTo(Climber.State.EXTENDED));
        controller.getButton(XboxButton.kStart).onTrue(climber.climbTo(Climber.State.RETRACTED)); 
        controller.getButton(XboxButton.kLeftTrigger).onTrue(intake.intake(Intake.State.EXTENDED)); 
        controller.getButton(XboxButton.kLeftBumper).onTrue(intake.retract()); 
        controller.getButton(XboxButton.kX).onTrue(shoot()); //TODO: Trap


        rightStick.getButton(1).onTrue(runOnce(()-> swerve.zeroGyro(0)));

        rightStick.getButton(2).onTrue(shooter.setShooter(0.8)).onFalse(shooter.setShooter(0));
        rightStick.getButton(3).onTrue(shooter.shoot(0));
        rightStick.getButton(4).onTrue(climber.setClimber(0.2)).onFalse(climber.setClimber(0));
        rightStick.getButton(5).onTrue(climber.setClimber(-0.2)).onFalse(climber.setClimber(0));
        rightStick.getButton(6).onTrue(climber.climbTo(0));
        rightStick.getButton(7).onTrue(climber.reset());
        rightStick.getButton(8).onTrue(intake.setPivot(0.2)).onFalse(intake.setPivot(0));
        rightStick.getButton(9).onTrue(intake.setPivot(-0.2)).onFalse(intake.setPivot(0));
        rightStick.getButton(10).onTrue(intake.pivotTo(180));
        rightStick.getButton(11).onTrue(intake.reset());
        rightStick.getButton(12).onTrue(intake.setRoller(0.5)).onFalse(intake.setRoller(0));
        rightStick.getButton(13).onTrue(intake.setRoller(IntakeConstants.OUTTAKE_POWER)).onFalse(intake.setRoller(0));
        rightStick.getButton(14).onTrue(new CmdSysId("Swerve", (Double volts)-> swerve.setVoltage(volts), ()-> swerve.getVelocity(), swerve)).onFalse(runOnce(()-> swerve.stop(), swerve));
    }
}
