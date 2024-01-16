package frc.team3128.commands;
import edu.wpi.first.wpilibj2.command.Command;
import static frc.team3128.Constants.FocalAimConstants.config;
import static frc.team3128.Constants.FocalAimConstants.constraints;
import java.util.function.DoubleSupplier;
import common.core.controllers.TrapController;
import common.core.commands.NAR_PIDCommand;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.team3128.Robot;
import frc.team3128.subsystems.Swerve;


/*
 * To DO:
 * 1. Turn in place using gyroscope to determine the angle at which the robot will aim at the speaker
 * 2. note the angle and position
 * 3. Choose a different position and attempt to convert the location to aim in the threshold of the speaker
 * 4. use the +/- threshold and crease a function to scale the robots location to the angle at which the robot should aim
 */
public class CmdFocalAim extends NAR_PIDCommand {
    private static TrapController controller;
    private Swerve swerve;
    private static DoubleSupplier measurement;
    private DoubleSupplier setpoint;
    private double rotation;
    
    public CmdFocalAim(DoubleSupplier setpoint)
    { 
        super(controller, measurement, setpoint, (double output)-> Swerve.getInstance().drive(new Translation2d(0,0), output, false));
        this.setpoint = setpoint;
        swerve = Swerve.getInstance();
        controller = new TrapController(config, constraints); //put into constants folder 
    }

    @Override
    public void initialize() {

    }

    @Override
    public void execute() 
    {
        //rotation needed calculated based off of the setpoint
        // rotation = Units.degreesToRadians(controller.calculate(swerve.getGyroRotation2d().getDegrees())); //calculate based off of setpoint
        // swerve.drive(new Translation2d(0, 0), rotation, true);
    }

    

    @Override
    public void end(boolean interrupted) {
        swerve.stop();
    }

    @Override
    public boolean isFinished() {
        return controller.atSetpoint();
    }
}

