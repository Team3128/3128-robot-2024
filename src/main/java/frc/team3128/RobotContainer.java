package frc.team3128;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.team3128.commands.CmdSwerveDrive;
import frc.team3128.commands.SimpleSubsystemSysId;
import common.hardware.input.NAR_ButtonBoard;
import common.hardware.input.NAR_Joystick;
import common.hardware.input.NAR_XboxController;
import common.utility.narwhaldashboard.NarwhalDashboard;
import frc.team3128.subsystems.SimpleMotorSubsystem;
import frc.team3128.subsystems.Swerve;
import frc.team3128.util.FFCharacterization;

/**
 * Command-based is a "declarative" paradigm, very little robot logic should
 * actually be handled in the {@link Robot} periodic methods (other than the
 * scheduler calls). Instead, the structure of the robot (including subsystems,
 * commands, and button mappings) should be declared here.
 */
public class RobotContainer {

    private Swerve swerve;
    private SimpleMotorSubsystem motor;

    private NAR_Joystick rightStick;
    private NAR_ButtonBoard buttonPad;

    public static NAR_XboxController controller;

    private NarwhalDashboard dashboard;

    public RobotContainer() {

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
        rightStick.getButton(1).onTrue(new SimpleSubsystemSysId(motor, true, new FFCharacterization("Wrist"), (voltage)->motor.setVoltage(voltage), ()-> motor.getVelocity(), ()-> motor.getPosition(), -90, 90))
                                 .onFalse(new InstantCommand(()-> motor.stop()));
        rightStick.getButton(2).onTrue(new InstantCommand(()-> motor.resetPosiiton(0)));
        rightStick.getButton(3).onTrue(new InstantCommand(()-> motor.setVoltage(3))).onFalse(new InstantCommand(()-> motor.stop()));
    }
}
