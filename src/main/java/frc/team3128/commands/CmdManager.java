package frc.team3128.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import static edu.wpi.first.wpilibj2.command.Commands.*;
import frc.team3128.subsystems.Swerve;

public class CmdManager {
    private static Swerve swerve = Swerve.getInstance();
    private static Pose2d robotPosition;
    private static Pose2d focalPoint;
    // public static Command shoot() {
    //      new CmdFocalAim(()-> getDesiredAngle());
    //  }

    public static Command stop() {
        return runOnce(()-> swerve.stop(), swerve);
    }

    public static double getDesiredAngle() {
        double coordRobotX = robotPosition.getTranslation().getX();
        double coordRobotY = robotPosition.getTranslation().getY();
        double coordFocalX = focalPoint.getTranslation().getX();
        double coordFocalY = focalPoint.getTranslation().getY();
        double angleSetpoint = Math.atan((coordFocalY-coordRobotY)/(coordFocalX-coordRobotX));
        return angleSetpoint;
    }
}