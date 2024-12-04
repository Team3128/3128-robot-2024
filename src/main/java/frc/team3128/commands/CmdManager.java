package frc.team3128.commands;

import common.hardware.input.NAR_XboxController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ScheduleCommand;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import frc.team3128.Robot;
import frc.team3128.RobotContainer;
import frc.team3128.Constants.AutoConstants;
import frc.team3128.subsystems.Swerve;

import java.util.function.DoubleSupplier;

import com.pathplanner.lib.auto.AutoBuilder;

import static edu.wpi.first.wpilibj2.command.Commands.*;
import static frc.team3128.Constants.SwerveConstants.*;

public class CmdManager {

    private static Swerve swerve = Swerve.getInstance();

    private static NAR_XboxController controller = RobotContainer.controller;

    public static Command vibrateController(){
        return new ScheduleCommand(new StartEndCommand(()-> controller.startVibrate(), ()-> controller.stopVibrate()).withTimeout(1));
    }
}