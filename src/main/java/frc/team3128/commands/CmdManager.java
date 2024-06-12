package frc.team3128.commands;

import common.hardware.input.NAR_XboxController;
import common.utility.shuffleboard.NAR_Shuffleboard;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ScheduleCommand;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import frc.team3128.Robot;
import frc.team3128.RobotContainer;
import frc.team3128.Constants.AutoConstants;
import frc.team3128.Constants.FieldConstants;
import frc.team3128.Constants.ShooterConstants;
import frc.team3128.Constants.LedConstants.Colors;
import frc.team3128.subsystems.AmpMechanism;
import frc.team3128.subsystems.Climber;
import frc.team3128.subsystems.Intake;
import frc.team3128.subsystems.Leds;
import frc.team3128.subsystems.Shooter;
import frc.team3128.subsystems.Swerve;
import frc.team3128.subsystems.Climber.Setpoint;

import java.util.function.DoubleSupplier;

import com.pathplanner.lib.auto.AutoBuilder;
import static edu.wpi.first.wpilibj2.command.Commands.*;
import static frc.team3128.Constants.ShooterConstants.*;
import static frc.team3128.Constants.SwerveConstants.*;
import static frc.team3128.Constants.FocalAimConstants.*;

public class CmdManager {

    private static Swerve swerve = Swerve.getInstance();
    private static Intake intake = Intake.getInstance();
    private static Shooter shooter = Shooter.getInstance();
    private static Climber climber = Climber.getInstance();
    private static AmpMechanism ampMechanism = AmpMechanism.getInstance();

    private static NAR_XboxController controller = RobotContainer.controller;

    public static boolean climb = false;

    public static Command autoIntake() {
        return deadline(
            intake.intake(Intake.Setpoint.EXTENDED),
            repeatingSequence(
                waitUntil(()-> RobotContainer.limelight.hasValidTarget() && !intake.intakeRollers.hasObjectPresent()),
                new CmdAutoAlign(maxSpeed, true).asProxy()
            )
        );
    }

    public static Command vibrateController(){
        return new ScheduleCommand(new StartEndCommand(()-> controller.startVibrate(), ()-> controller.stopVibrate()).withTimeout(1));
    }

    public static Command shootDist() {
        return deadline(
            sequence(
                climber.climbTo(()-> climber.interpolate(swerve.getDist())),
                shooter.shoot(MAX_RPM),
                waitUntil(shooter::atSetpoint),
                climber.climbTo(() -> climber.interpolate(swerve.getDist())),
                waitUntil(climber::atSetpoint),
                intake.intakeRollers.outtakeNoRequirements(),
                waitSeconds(0.35),
                neutral(false)
            ),
            repeatingSequence(  
                runOnce(()-> CmdSwerveDrive.setTurnSetpoint(swerve.getTurnAngle(Robot.getAlliance() == Alliance.Red ? focalPointRed : focalPointBlue))),
                waitSeconds(0.1)
            )
        ).andThen(runOnce(()-> CmdSwerveDrive.disableTurn()));
    }

    public static Command autoShoot() {
        return sequence(
            // runOnce(()-> DriverStation.reportWarning("AutoShoot: CommandStarting", false)),
            parallel(
                rampUp().withTimeout(RAMP_TIME),
                swerve.turnInPlace(false).asProxy().withTimeout(1)
                // runOnce(()-> CmdSwerveDrive.setTurnSetpoint(swerve.getTurnAngle(Robot.getAlliance() == Alliance.Red ? focalPointRed : focalPointBlue))),
                // waitUntil(()-> CmdSwerveDrive.rController.atSetpoint())
            ),
            intake.intakeRollers.outtakeNoRequirements(),
            waitSeconds(0.35),
            neutral(false)
            // runOnce(()-> DriverStation.reportWarning("AutoShoot: CommandEnding", false))
        );
    }

    public static Command rampUpAmp() {
        return sequence(
            // runOnce(()-> autoAmpAlign().schedule()),
            climber.climbTo(Setpoint.AMP),
            shooter.shoot(ShooterConstants.AMP_RPM),
            waitUntil(()-> climber.atSetpoint()),
            ampMechanism.extend()
        );
    }

    public static Command rampUpAmp(DoubleSupplier climberHeight) {
        return sequence(
            // runOnce(()-> autoAmpAlign().schedule()),
            climber.climbTo(climberHeight),
            shooter.shoot(ShooterConstants.AMP_RPM),
            waitUntil(()-> climber.atSetpoint()),
            ampMechanism.extend()
        );
    }

    public static Command ampShoot() {
        return sequence (
            climber.climbTo(Setpoint.AMP),
            shooter.shoot(ShooterConstants.AMP_RPM),
            waitUntil(()-> climber.atSetpoint()),
            ampMechanism.extend(),
            waitUntil(()-> ampMechanism.atSetpoint() && shooter.atSetpoint()),
            intake.intakeRollers.outtake(),
            waitSeconds(0.9),
            ampMechanism.retract(),
            waitUntil(()-> ampMechanism.atSetpoint()),
            neutral(false)
        );
    }

    public static Command ampShoot(DoubleSupplier climberHeight) {
        return sequence (
            climber.climbTo(climberHeight),
            shooter.shoot(ShooterConstants.AMP_RPM),
            waitUntil(()-> climber.atSetpoint()),
            ampMechanism.extend(),
            waitUntil(()-> ampMechanism.atSetpoint() && shooter.atSetpoint()),
            intake.intakeRollers.outtake(),
            waitSeconds(0.9),
            ampMechanism.retract(),
            waitUntil(()-> ampMechanism.atSetpoint()),
            neutral(false)
        );
    }

    public static Command readyOrbitAmp() {
        DoubleSupplier angle1 = NAR_Shuffleboard.debug("Amp", "angle1", 30, 0, 0);
        return intake.intakePivot.pivotTo(angle1);
    }

    public static Command orbitAmp() {
        DoubleSupplier angle2 = NAR_Shuffleboard.debug("Amp", "angle2", 60, 1, 0);
        DoubleSupplier power = NAR_Shuffleboard.debug("Amp", "power", -0.8, 2, 0);
        return sequence(
            parallel(
                intake.intakePivot.pivotTo(angle2),
                intake.intakeRollers.runManipulator(power)
            ),
            waitUntil(()->intake.intakePivot.atSetpoint()),
            parallel(
                intake.intakePivot.pivotTo(0),
                intake.intakeRollers.runManipulator(0)
            )
        );
    }

    public static Command rampRam() {
        return sequence(
            climber.climbTo(Climber.Setpoint.RAMSHOT),
            shooter.shoot(RAM_SHOT_RPM, RAM_SHOT_RPM)
        );
    }

    public static Command ramShot() {
        return sequence(
            rampRam(),
            waitUntil(()-> climber.atSetpoint() && shooter.atSetpoint()),
            intake.intakeRollers.outtakeNoRequirements(),
            waitSeconds(0.35),
            neutral(false)
        );
    }

    public static Command moveShoot() {
        return sequence(
            deadline(
                sequence(
                    runOnce(()-> CmdSwerveDrive.setTurnSetpoint(Robot.getAlliance() == Alliance.Red ? 0 : 180)),
                    shooter.shoot(RAM_SHOT_RPM, RAM_SHOT_RPM),
                    waitUntil(()-> swerve.crossedPodium()),
                    either(
                        either(
                            sequence(
                                runOnce(()-> CmdSwerveDrive.setTurnSetpoint(Robot.getAlliance() == Alliance.Red ? 10 : 170)),
                                waitUntil(()-> swerve.getPose().getY() < higherBound)
                            ), 
                            sequence(
                                runOnce(()-> CmdSwerveDrive.setTurnSetpoint(Robot.getAlliance() == Alliance.Red ? -10 : -170)),
                                waitUntil(()-> swerve.getPose().getY() > lowerBound)
                            ), 
                            ()-> swerve.getPose().getY() > speakerMidpointY),
                        none(),
                        ()-> (swerve.getPose().getY() < lowerBound || swerve.getPose().getY() > higherBound)
                    ),
                    waitUntil(()-> shooter.atSetpoint())
                ),
                repeatingSequence(
                    climber.climbTo(()-> climber.interpolate(swerve.getDistHorizontal())),
                    waitSeconds(0.25)
                )
            ),
            intake.intakeRollers.outtakeNoRequirements(),
            waitSeconds(0.35),
            neutral(false)
        );
    }

    public static Command shoot(double rpm, double height){
        return sequence(
            // runOnce(()-> DriverStation.reportWarning("Shoot: CommandStarting", false)),
            rampUp(rpm, height).withTimeout(RAMP_TIME),
            intake.intakeRollers.outtakeNoRequirements(),
            waitSeconds(0.35),
            neutral(false)
            // runOnce(()-> DriverStation.reportWarning("Shoot: CommandEnding", false))
        );
    }

    public static Command feed(double rpm, double height, double angle){
        return sequence(
            parallel(
                swerve.turnInPlace(()-> Robot.getAlliance() == Alliance.Blue ? 180-angle : angle).asProxy().withTimeout(1),
                rampUp(rpm, height),
                runOnce(()-> shooter.startPID(rpm, rpm))
            ),
            intake.intakeRollers.outtakeNoRequirements(),
            waitSeconds(0.25),
            neutral(false)
        );
    }

    public static Command rampUp() {
        return rampUp(ShooterConstants.MAX_RPM, ()-> climber.interpolate(swerve.getDist()));
    }

    public static Command rampUpFeed(double leftRpm, double rightRpm, double height) {
        return sequence(
            runOnce(()-> shooter.startPID(leftRpm, rightRpm)),
            climber.climbTo(height)
        );
    }

    public static Command rampUp(double rpm, double height){
        return rampUp(rpm, ()-> height);
    }

    public static Command rampUp(double rpm, DoubleSupplier height) {
        return sequence(
            climber.climbTo(height),
            shooter.shoot(rpm),
            waitUntil(()-> climber.atSetpoint() && shooter.atSetpoint())
        );
    }

    public static Command rampUpContinuous() {
        return rampUpContinuous(ShooterConstants.MAX_RPM, ()-> climber.interpolate(swerve.getPredictedDistance()));
    }

    public static Command rampUpContinuous(double rpm, DoubleSupplier height) {
        return sequence(
            shooter.shoot(rpm),
            repeatingSequence(
                climber.climbTo(height),
                waitSeconds(0.1)
            )
        );
    }

    public static Command neutral(boolean shouldStall){
        return sequence(
            // runOnce(()-> DriverStation.reportWarning("Neutral: CommandStarting", false)),
            // TODO: test this
            // intake.intakeRollers.runNoRequirements(0),
            shooter.setShooter(0),
            climber.climbTo(Climber.Setpoint.RETRACTED),
            // runOnce(()-> climber.toggleBrake(false)),
            waitUntil(()-> climber.atSetpoint()),
            climber.setClimber(-0.5),
            waitSeconds(0.1),
            climber.setClimber(0),
            parallel(
                // intake.retract(shouldStall),
                sequence(
                    waitSeconds(0.5),
                    climber.reset()
                    // runOnce(()-> climber.toggleBrake(true))
                )
            )
            // runOnce(()-> DriverStation.reportWarning("Neutral: CommandEnding", false))
        );
    }

    public static Command autoAmpAlign(){
        double ampX = 14.64;
        double ampY = 7.70;
        Pose2d ampPos = new Pose2d(Robot.getAlliance() == Alliance.Red ? ampX : FieldConstants.FIELD_X_LENGTH - ampX, ampY,  Rotation2d.fromDegrees(90));

        return sequence(
            runOnce(()-> Leds.getInstance().setLedColor(Colors.AMP)),
            AutoBuilder.pathfindToPose(
                ampPos,
                AutoConstants.constraints,
                0.0, // Goal end velocity in meters/sec
                0.0 // Rotation delay distance in meters. This is how far the robot should travel before attempting to rotate.
            ),
            runOnce(()-> Leds.getInstance().setDefaultColor())
        ).beforeStarting(runOnce(()-> RobotContainer.toggleSideCams(false))).finallyDo((boolean interrupted)-> RobotContainer.toggleSideCams(true));
    }


}