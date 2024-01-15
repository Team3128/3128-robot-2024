package frc.team3128.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;

public class CmdManager {
    private static double setpoint = 2;
     public static Command shoot() {
         new CmdFocalAim(()-> getDesiredAngle());
     }

    public static double getDesiredAngle() {
        return setpoint;
    }
}