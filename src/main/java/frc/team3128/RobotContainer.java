package frc.team3128;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.team3128.commands.CmdManager;
import frc.team3128.commands.CmdSwerveDrive;
import common.hardware.input.NAR_ButtonBoard;
import common.hardware.input.NAR_Joystick;
import common.hardware.input.NAR_XboxController;
import common.utility.narwhaldashboard.NarwhalDashboard;
import frc.team3128.subsystems.Manipulator;
import frc.team3128.subsystems.Pivot;
import frc.team3128.subsystems.Shooter;
import frc.team3128.subsystems.ShooterTrap;
import frc.team3128.subsystems.Swerve;
import frc.team3128.subsystems.Telescope;
import common.hardware.input.NAR_XboxController.XboxButton;
import frc.team3128.commands.CmdManager.*;
/**
 * Command-based is a "declarative" paradigm, very little robot logic should
 * actually be handled in the {@link Robot} periodic methods (other than the
 * scheduler calls). Instead, the structure of the robot (including subsystems,
 * commands, and button mappings) should be declared here.
 */
public class RobotContainer {
    private  XboxButton button;

    private Swerve swerve;
    private Manipulator manipulator;
    private Pivot pivot;
    private Shooter shooter;
    private ShooterTrap shooter2;
    private Telescope telescope;

    private NAR_Joystick rightStick;
    private NAR_ButtonBoard buttonPad;

    public static NAR_XboxController controller;
    

    private NarwhalDashboard dashboard;

    public RobotContainer() {

        swerve = Swerve.getInstance();
        manipulator = Manipulator.getInstance();
        pivot = Pivot.getInstance();
        shooter = Shooter.getInstance();
        shooter2 = ShooterTrap.getInstance();
        telescope = Telescope.getInstance();

        rightStick = new NAR_Joystick(1);
        controller = new NAR_XboxController(2);
        buttonPad = new NAR_ButtonBoard(3);
        
        //uncomment line below to enable driving
        CommandScheduler.getInstance().setDefaultCommand(swerve, new CmdSwerveDrive(controller::getLeftX,controller::getLeftY, controller::getRightX, true));
        configureButtonBindings();
        
        DriverStation.silenceJoystickConnectionWarning(true);
    }   

    private void configureButtonBindings() {
        controller.getButton(XboxButton.kA).onTrue(new InstantCommand(() -> shooter.intake())).onFalse(new InstantCommand(()->shooter.stopShooter()));
        controller.getButton(XboxButton.kB).onTrue(new InstantCommand(() -> shooter.outtake())).onFalse(new InstantCommand(()->shooter.stopShooter()));
        controller.getButton(XboxButton.kX).onTrue(new InstantCommand(() -> shooter.intake())).onFalse(new InstantCommand(()->shooter.stopShooter()));
        controller.getButton(XboxButton.kY).onTrue(new InstantCommand(() -> shooter.intake())).onFalse(new InstantCommand(()->shooter.stopShooter()));
        controller.getButton(XboxButton.kLeftTrigger).onTrue(new InstantCommand(() -> pivot.pivot(30))).onFalse(new InstantCommand(()->pivot.setPower(0)));
        controller.getButton(XboxButton.kRightTrigger).onTrue(new InstantCommand(() -> shooter.shoot(500))).onFalse(new InstantCommand(()->shooter.setPower(0)));
        controller.getButton(XboxButton.kLeftStick).onTrue(new InstantCommand(() -> shooter2.shoot(500))).onFalse(new InstantCommand(()->shooter2.setPower(0)));
        controller.getButton(XboxButton.kRightStick).onTrue(new InstantCommand(() -> telescope.moveTele(10))).onFalse(new InstantCommand(()->telescope.setPower(0)));
        controller.getButton(XboxButton.kLeftBumper).onTrue(CmdManager.Intake());
        controller.getButton(XboxButton.kRightBumper).onTrue(CmdManager.Outtake());
        
    }
}
