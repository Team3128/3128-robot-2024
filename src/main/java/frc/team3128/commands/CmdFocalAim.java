package frc.team3128.commands;
import edu.wpi.first.wpilibj2.command.Command;
import static frc.team3128.Constants.FocalAimConstants.config;
import static frc.team3128.Constants.FocalAimConstants.constraints;
import java.util.function.DoubleSupplier;
import common.core.controllers.TrapController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import frc.team3128.subsystems.Swerve;


/*
 * To DO:
 * 1. Turn in place using gyroscope to determine the angle at which the robot will aim at the speaker
 * 2. note the angle and position
 * 3. Choose a different position and attempt to convert the location to aim in the threshold of the speaker
 * 4. use the +/- threshold and crease a function to scale the robots location to the angle at which the robot should aim
 */
public class CmdFocalAim extends Command {
    private TrapController controller;
    private Swerve swerve;
    private DoubleSupplier setpoint;
    private double rotation;

    public CmdFocalAim(DoubleSupplier setpoint)
    { 
        this.setpoint = setpoint;
        swerve = Swerve.getInstance();
        controller = new TrapController(config, constraints); //put into constants folder 
    }

    @Override
    public void initialize() {
        controller.setSetpoint(setpoint.getAsDouble()); //sets setpoint
    }

    @Override
    public void execute() 
    {
        //rotation needed calculated based off of the setpoint
        rotation = Units.degreesToRadians(controller.calculate(swerve.getGyroRotation2d().getDegrees())); //calculate based off of setpoint
        swerve.drive(new Translation2d(0, 0), rotation, true);
    }

    //focal aim command method thing, kinda just throwing it in here
    // public double getSetpoint(Pose2d focalPoint) {
    //     double coordRobotX = robotPosition.getTranslation().getX();
    //     double coordRobotY = robotPosition.getTranslation().getY();
    //     double coordFocalX = focalPoint.getTranslation().getX();
    //     double coordFocalY = focalPoint.getTranslation().getY();
    //     //distance
    //     double distance = Math.hypot(coordFocalX - coordRobotX, coordFocalY - coordRobotY);

    //     //angle
    //     double angleSetpoint = Math.atan((coordFocalY-coordRobotY)/(coordFocalX-coordRobotX));
    //     return angleSetpoint;
    //     }
    

    @Override
    public void end(boolean interrupted) {
        swerve.stop();
    }

    @Override
    public boolean isFinished() {
        return controller.atSetpoint();
    }
}

