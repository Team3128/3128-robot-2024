package frc.team3128.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;

public class CmdManager {
    private static double setpoint = 2;
     public static Command shoot() {
         new CmdFocalAim(()-> getDesiredAngle());
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