package frc.team3128;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.team3128.commands.CmdSwerveDrive;
import common.hardware.input.NAR_ButtonBoard;
import common.hardware.input.NAR_Joystick;
import common.hardware.input.NAR_XboxController;
import common.hardware.input.NAR_XboxController.XboxButton;
import common.utility.narwhaldashboard.NarwhalDashboard;
import common.utility.shuffleboard.NAR_Shuffleboard;
import frc.team3128.subsystems.Swerve;

/**
 * Command-based is a "declarative" paradigm, very little robot logic should
 * actually be handled in the {@link Robot} periodic methods (other than the
 * scheduler calls). Instead, the structure of the robot (including subsystems,
 * commands, and button mappings) should be declared here.
 */
public class RobotContainer {

    private Swerve swerve;

    private NAR_Joystick rightStick;
    private NAR_ButtonBoard buttonPad;

    public static NAR_XboxController controller;

    private NarwhalDashboard dashboard;

    public RobotContainer() {
        NAR_Shuffleboard.WINDOW_WIDTH = 10;

        swerve = Swerve.getInstance();

        rightStick = new NAR_Joystick(1);
        controller = new NAR_XboxController(2);
        buttonPad = new NAR_ButtonBoard(3);
        
        //uncomment line below to enable driving
        CommandScheduler.getInstance().setDefaultCommand(swerve, new CmdSwerveDrive(controller::getLeftX,controller::getLeftY, controller::getRightX, true));
        configureButtonBindings();
        
        DriverStation.silenceJoystickConnectionWarning(true);
    }   

    private void configureButtonBindings() {
        controller.getButton(XboxButton.kB).onTrue(Commands.runOnce(()-> swerve.resetEncoders()));

        rightStick.getButton(1).onTrue(Commands.runOnce(()-> swerve.zeroGyro(0)));

    }
}
