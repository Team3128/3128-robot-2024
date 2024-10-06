package frc.team3128.autonomous;

import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;

import common.core.commands.NAR_PIDCommand;
import common.core.controllers.Controller;
import common.core.controllers.PIDFFConfig;
import common.hardware.limelight.Limelight;
import common.core.controllers.Controller.Type;
import common.utility.Log;
import common.utility.shuffleboard.NAR_Shuffleboard;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.pathfinding.LocalADStar;
import com.pathplanner.lib.pathfinding.Pathfinding;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;

import static edu.wpi.first.wpilibj2.command.Commands.*;
import static frc.team3128.Constants.AutoConstants.*;
import static frc.team3128.Constants.IntakeConstants.*;
import static frc.team3128.Constants.FocalAimConstants.focalPointBlue;
import static frc.team3128.Constants.FocalAimConstants.focalPointRed;
import static frc.team3128.Constants.ShooterConstants.MAX_RPM;
import static frc.team3128.Constants.ShooterConstants.RAM_SHOT_RPM;
import static frc.team3128.Constants.SwerveConstants.*;
import static frc.team3128.Constants.FieldConstants.*;

import frc.team3128.Constants.AutoConstants;
import frc.team3128.Constants.FieldConstants.Note;
import frc.team3128.Constants.ShooterConstants;
import frc.team3128.Robot;
import frc.team3128.RobotContainer;
import frc.team3128.commands.CmdAutoAlign;
import frc.team3128.commands.CmdManager;
import frc.team3128.commands.CmdSwerveDrive;
// import frc.team3128.commands.NAR_PIDCommand;

import static frc.team3128.commands.CmdManager.rampUp;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import frc.team3128.subsystems.AmpMechanism;
import frc.team3128.subsystems.Climber;
import frc.team3128.subsystems.Intake;
import frc.team3128.subsystems.Shooter;
import frc.team3128.subsystems.Swerve;

/**
 * Store trajectories for autonomous. Edit points here. 
 * @author Daniel Wang
 */
public class Trajectories {

    //USED FOR HARDCODED SHOTS
    public enum ShootPosition {
        // find values
        WING(5.213),    //Change this for top 
        BOTTOM(7.5),    //Change this for bottom auto
        RAM(24.5);

        private final double height;
        ShootPosition(double height) {
            this.height = height;
        }
        public double getHeight() {
            return height;
        }
    }

    private static final Swerve swerve = Swerve.getInstance();
    private static final Climber climber = Climber.getInstance();
    private static final Shooter shooter = Shooter.getInstance();
    private static final Intake intake = Intake.getInstance();
    private static final Limelight limelight = RobotContainer.limelight;
    private static double vx = 0, vy = 0;
    private static boolean turning = false;
    private static BooleanSupplier hasNote = intake.intakeRollers::hasObjectPresent;
    private static final AutoPrograms autoPrograms = AutoPrograms.getInstance();

    public static void initTrajectories() {
        Pathfinding.setPathfinder(new LocalADStar());

        NamedCommands.registerCommand("Intake", intake.intakeAuto());
        NamedCommands.registerCommand("Shoot", either(autoShoot(0.75), runOnce(()->Log.info("test", "skip shot")), hasNote));
        NamedCommands.registerCommand("RamShoot", ramShotAuto());
        NamedCommands.registerCommand("BottomShoot", autoShootPreset(ShootPosition.BOTTOM));
        NamedCommands.registerCommand("WingRamp", rampUpAuto(ShootPosition.WING));
        NamedCommands.registerCommand("Align", align());
        NamedCommands.registerCommand("AlignCCW", alignSearch(true));
        NamedCommands.registerCommand("AlignCW", alignSearch(false));
        NamedCommands.registerCommand("Outtake", intake.intakeRollers.outtakeWithTimeout(0.25));
        NamedCommands.registerCommand("Retract", intake.retractAuto());
        NamedCommands.registerCommand("Neutral", neutralAuto());
        NamedCommands.registerCommand("AlignPreload", alignPreload(false));
        NamedCommands.registerCommand("Drop", shooter.shoot(MAX_RPM));
        NamedCommands.registerCommand("RamShootMax", autoShootPreset(ShootPosition.RAM));
        NamedCommands.registerCommand("MidShoot", midShotAuto());

        NamedCommands.registerCommand("RampUp-BottomShootClose", autoRampUp(getDistance(2.39, 3.43))); //should be 3.097
        NamedCommands.registerCommand("RampUp-Bottom", autoRampUp(getDistance(1.45, 4.10)));
        NamedCommands.registerCommand("RampUp-Note1.3", autoRampUp(getDistance(2.07, 4.10))); 
        // !!!REMIND SOMEEONE THAT BOTTOM TO NOTE 1.3 IS FAULTY
        NamedCommands.registerCommand("RampUp-Note1.2", autoRampUp(getDistance(2.46, 5.55)));
        NamedCommands.registerCommand("RampUp-Note1.1", autoRampUp(getDistance(2.58, 6.99)));
        NamedCommands.registerCommand("RampUp-Middle", autoRampUp(getDistance(1.50, 5.55)));
        NamedCommands.registerCommand("RampUp-BottomShoot", autoRampUp(getDistance(3.20, 2.71)));
        NamedCommands.registerCommand("RampUp-Top", autoRampUp(getDistance(1.39, 6.99)));

        NamedCommands.registerCommand("ShootNoRotate", shootNoRotate());


        AutoBuilder.configureHolonomic(
            swerve::getPose,
            swerve::resetOdometry,
            swerve::getRobotVelocity,
            Trajectories::drive,
            new HolonomicPathFollowerConfig(
                new PIDConstants(translationKP, translationKI, translationKD),
                new PIDConstants(rotationKP, rotationKI, rotationKD),
                maxAttainableSpeed,
                trackWidth,
                new ReplanningConfig(false, true)
            ),
            ()-> Robot.getAlliance() == Alliance.Red,
            swerve
        );
    }

    public static Command middleClose_4note() {
        return sequence(
            autoShootNoTurn(),
            findAndScore(allianceFlip(Note.NOTE1_1.getTranslation())),
            findAndScore(allianceFlip(Note.NOTE1_2.getTranslation())),
            findAndScore(allianceFlip(Note.NOTE1_3.getTranslation()))
        );
    }

    public static Command middle_5note() {
        return sequence(
            autoShootNoTurn(),
            findAndScore(allianceFlip(Note.NOTE1_2.getTranslation())),
            autoPrograms.getPath("only-note1.2-note2.3"),
            findNoTurn(allianceFlip(Note.NOTE2_3.getTranslation())),
            autoPrograms.getPath("note2.3-middle"),
            autoShootNoTurn().onlyIf(hasNote),
            findAndScore(allianceFlip(Note.NOTE1_1.getTranslation())),
            findAndScore(allianceFlip(Note.NOTE1_3.getTranslation()))
        );
    }

    public static Command top_4note() {
        return sequence(
            autoShootNoTurn(),
            autoPrograms.getPath("only-top-note2.1"),
            findNote2_1(),
            autoPrograms.getPath("note2.1-wing"),
            autoShoot(0.75).onlyIf(hasNote),
            autoPrograms.getPath("only-wing-note2.2"),
            findNote2_2(),
            autoPrograms.getPath("note2.2-wing"),
            autoShoot(0.75).onlyIf(hasNote),
            autoPrograms.getPath("only-wing-note2.3"),
            findNote2_3(),
            autoPrograms.getPath("note2.3-wing"),
            autoShoot(0.75).onlyIf(hasNote)
        );
    }

    // Find Note2_1, if it is there
    // If not, find Note2_2
    // (If Note2_2 is not there, find Note2_3)
    public static Command findNote2_1() {
        return either(
            findNoTurn(allianceFlip(Note.NOTE2_1.getTranslation())),
            sequence(
                turnInPlaceIntake(allianceFlip(Note.NOTE2_2.getTranslation())),
                findNote2_2()
            ),
            () -> limelight.hasValidTarget()
        );
    }

    // Find Note2_2, if it is there
    // If not, find Note2_3
    public static Command findNote2_2() {
        return either(
            findNoTurn(allianceFlip(Note.NOTE2_2.getTranslation())),
            sequence(
                turnInPlaceIntake(allianceFlip(Note.NOTE2_3.getTranslation())),
                findNote2_3()
            ),
            () -> limelight.hasValidTarget()
        );
    }

    // Find Note2_3, if it is there
    public static Command findNote2_3() {
        return findNoTurn(allianceFlip(Note.NOTE2_3.getTranslation())).onlyIf(() -> limelight.hasValidTarget());
    }
    
    // find() and then shoot
    public static Command findAndScore(Translation2d note) {
        return sequence(
            find(note),
            autoShootNoTurn().onlyIf(hasNote)
        );
    }

    // Points limelight toward the note and then findNoTurn()
    public static Command find(Translation2d note) {
        return sequence(
            turnInPlaceIntake(note).withTimeout(1),
            findNoTurn(note)
        );
    }

    // Assuming limelight is pointed towards note, aligns and intakes it
    // Also turns toward subwoofer
    public static Command findNoTurn(Translation2d note) {
        return sequence(
            sequence(
                // This aligns
                parallel(
                    intake.intakeAuto(),
                    new CmdAutoAlign(3, true).beforeStarting(() -> CmdAutoAlign.hasTimedOut = false)
                ),
                parallel(intake.retractAuto(), turnInPlace().withTimeout(0.75))
            ).onlyIf(() -> limelight.hasValidTarget() && note.minus(swerve.getPose().getTranslation()).getNorm() > TOO_CLOSE)
        );
    }

    public static void drive(ChassisSpeeds velocity) {
        if (!turning) swerve.drive(velocity);
        else {
            vx = velocity.vxMetersPerSecond;
            vy = velocity.vyMetersPerSecond;
        }
    }

    public static Command turnDegrees(boolean counterClockwise, double angle) {

        double setpoint = swerve.getYaw() + angle * (counterClockwise ? 1 : -1) * (Robot.getAlliance() == Alliance.Red ? -1 : 1);

        Controller c = new Controller(config, Type.POSITION);
        c.enableContinuousInput(-180, 180);
        // c.setMeasurementSource(()-> Swerve.getInstance().getYaw());
        c.setTolerance(TURN_TOLERANCE);
        return new NAR_PIDCommand(
            c, 
            ()-> swerve.getYaw(), //measurement
            ()-> setpoint, //setpoint
            (double output) -> {
                Swerve.getInstance().drive(new ChassisSpeeds(vx, vy, Units.degreesToRadians(output)));
                // NAR_Shuffleboard.addData("HElp", "help", output, 0, 0);
            },
            Swerve.getInstance()
        ).beforeStarting(runOnce(()-> {CmdSwerveDrive.disableTurn(); swerve.resetEncoders();}));
    }

    public static Command alignPreload(boolean counterClockwise) {
        return race(
            intake.intakeAuto(), 
            sequence(
                turnDegrees(false, 70).until(()-> limelight.hasValidTarget()),
                repeatingSequence(
                    runOnce(()-> CmdAutoAlign.hasTimedOut = false),
                    new CmdAutoAlign(3, false),
                    run(()-> swerve.drive(
                        new Translation2d(), 
                        maxAngularVelocity / 4.0 * (counterClockwise ? 1 : -1) * (Robot.getAlliance() == Alliance.Red ? -1 : 1), 
                        false)
                    ).until(()-> limelight.hasValidTarget())
                ).until(()-> intake.intakeRollers.hasObjectPresent())
            ).beforeStarting(runOnce(()->{turning = true;})).andThen(runOnce(()->{turning = false;})));
    }

    public static Command alignSearch(boolean counterClockwise) {
        return sequence(
            runOnce(()-> CmdAutoAlign.hasTimedOut = false),
            new CmdAutoAlign(3, false),
            turnDegrees(counterClockwise, 45).until(()-> limelight.hasValidTarget()),
            new CmdAutoAlign(3, false)
        ).until(()-> intake.intakeRollers.hasObjectPresent())
        .beforeStarting(runOnce(()->{turning = true;})).andThen(runOnce(()->{turning = false;}));
    }

    public static Command align() {
        return sequence(
            runOnce(()-> CmdAutoAlign.hasTimedOut = false),
            new CmdAutoAlign(3, false),
            waitSeconds(1).until(()-> limelight.hasValidTarget()),
            new CmdAutoAlign(3, false)
        ).until(()-> intake.intakeRollers.hasObjectPresent())
        .beforeStarting(runOnce(()->{turning = true;})).andThen(runOnce(()->{turning = false;}));
    }

    public static Command ramShotAuto() {
        return sequence(
            new InstantCommand(()->swerve.resetEncoders()),
            climber.climbTo(Climber.Setpoint.RAMSHOT),
            shooter.shoot(RAM_SHOT_RPM, RAM_SHOT_RPM),
            waitUntil(()-> climber.atSetpoint() && shooter.atSetpoint()).withTimeout(1.5),
            intake.intakeRollers.runNoRequirements(OUTTAKE_POWER),
            waitSeconds(0.35),
            shooter.shoot(MAX_RPM)
        );
    }

    public static Command midShotAuto(){
        return sequence(
            climber.climbTo(Climber.Setpoint.MIDSHOT),
            shooter.shoot(RAM_SHOT_RPM, RAM_SHOT_RPM),
            waitUntil(()-> climber.atSetpoint() && shooter.atSetpoint()).withTimeout(1.5),
            intake.intakeRollers.runNoRequirements(OUTTAKE_POWER),
            waitSeconds(0.35),
            shooter.shoot(MAX_RPM)
        );
    }

    public static Command turnInPlace(Translation2d point) {
        DoubleSupplier setpoint = ()-> swerve.getTurnAngle(point);
        return new NAR_PIDCommand(
            TURN_CONTROLLER, 
            ()-> swerve.getYaw(), //measurement
            setpoint, //setpoint
            (double output) -> {
                Swerve.getInstance().drive(new ChassisSpeeds(vx, vy, Units.degreesToRadians(output)));
                NAR_Shuffleboard.addData("HElp", "help", output, 0, 0);
            },
            swerve
        ).beforeStarting(runOnce(()-> CmdSwerveDrive.disableTurn()));
    }

    public static Command turnInPlaceIntake(Translation2d point) {
        DoubleSupplier setpoint = ()-> MathUtil.inputModulus(180 + swerve.getTurnAngle(point), -180, 180);
        return new NAR_PIDCommand(
            TURN_CONTROLLER, 
            ()-> swerve.getYaw(), //measurement
            setpoint, //setpoint
            (double output) -> {
                Swerve.getInstance().drive(new ChassisSpeeds(vx, vy, Units.degreesToRadians(output)));
                NAR_Shuffleboard.addData("HElp", "help", output, 0, 0);
            },
            swerve
        ).beforeStarting(runOnce(()-> CmdSwerveDrive.disableTurn()));
    }

    public static Command turnInPlace() {
        DoubleSupplier setpoint = ()-> swerve.getTurnAngle(Robot.getAlliance() == Alliance.Red ? focalPointRed : focalPointBlue);
        return new NAR_PIDCommand(
            TURN_CONTROLLER, 
            ()-> swerve.getYaw(), //measurement
            setpoint, //setpoint
            (double output) -> {
                Swerve.getInstance().drive(new ChassisSpeeds(vx, vy, Units.degreesToRadians(output)));
                NAR_Shuffleboard.addData("HElp", "Output", output, 0, 0);
                NAR_Shuffleboard.addData("HElp", "At Setpoint", TURN_CONTROLLER.getController().atSetpoint(),0, 1);
                NAR_Shuffleboard.addData("HElp", "Error", TURN_CONTROLLER.getController().getPositionError(),0, 1);
            },
            swerve
        ).beforeStarting(runOnce(()-> CmdSwerveDrive.disableTurn()));
    }

    public static Command rampUpAuto(ShootPosition pos) {
        return either(
            sequence(
                shooter.shoot(MAX_RPM),
                rampUp(()->pos.getHeight(), ShooterConstants.MAX_RPM).until(()-> climber.atSetpoint()),
                intake.retractAuto()
            ),
            none(),
            ()-> true
            // ()->intake.intakeRollers.hasObjectPresent()
        );
    }

    public static Command autoRampUp(double distance) {
        return CmdManager.rampUp(() -> climber.interpolate(distance), ShooterConstants.MAX_RPM);
    }

    public static Command shootNoRotate() {
        return either(
            sequence(
                either(intake.retractAuto(), none(), ()-> intake.intakePivot.isEnabled()),
                runOnce(()->{turning = true;}),
                CmdManager.rampUp(),
                // waitSeconds(1),
                runOnce(()->{turning = false;}),
                intake.intakeRollers.outtake(),
                //intake.intakeRollers.outtakeNoRequirements(),
                waitSeconds(0.1),
                neutralAuto()
            ),
            none(),
            ()->intake.intakeRollers.hasObjectPresent()
        );
    }

    public static double getDistance(double d, double e) {
        return Math.sqrt((d*d)+((5.4-e)*(5.4-e)));
        //gets distance of point based on blue's focal point (0, 5.4)
    }



    public static Command autoShootPreset(ShootPosition position) {
        return sequence(
            parallel (
                runOnce(()->{turning = true;}),
                rampUp(()->MAX_RPM, position.getHeight()),
                turnInPlace().withTimeout(0.5)
            ),
            runOnce(()->{turning = false;}),
            intake.intakeRollers.outtake(),
            waitSeconds(0.25),
            intake.intakeRollers.runManipulator(0)
        );
    }

    public static Command autoShootNoTurn() {
        return sequence(
            either(shooter.shoot(MAX_RPM), none(), ()-> shooter.isEnabled()),
            parallel(
                runOnce(()->{turning = true;}),
                sequence(
                    either(sequence(waitUntil(()-> intake.intakeRollers.hasObjectPresent()).withTimeout(0.25), intake.retractAuto()), none(), ()-> intake.intakePivot.isEnabled()),
                    rampUp().withTimeout(1)
                )
            ),
            runOnce(()->{turning = false;}),
            intake.intakeRollers.outtake(),
            waitSeconds(0.25),
            intake.intakeRollers.runManipulator(0)
        );
    }
 
    public static Command autoShoot(double turnTimeout) {
        return either(
            sequence(
                new InstantCommand(()->swerve.resetEncoders()),
                either(shooter.shoot(MAX_RPM), none(), ()-> shooter.isEnabled()),
                parallel(
                    runOnce(()->{turning = true;}),
                    sequence(
                        either(sequence(waitUntil(()-> intake.intakeRollers.hasObjectPresent()).withTimeout(0.25), intake.retractAuto()), none(), ()-> intake.intakePivot.isEnabled()),
                        rampUp()
                    ),
                    turnInPlace().withTimeout(turnTimeout)
                ),
                runOnce(()->{turning = false;}),
                intake.intakeRollers.outtake(),
                waitSeconds(0.25),
                intake.intakeRollers.runManipulator(0)
            ),
            none(),
            ()-> true
        );
    }

    public static Command neutralAuto() {
        return sequence(
            intake.intakeRollers.runNoRequirements(0),
            // Shooter.getInstance().setShooter(0),
            Climber.getInstance().climbTo(Climber.Setpoint.RETRACTED)
        );
    }

    public static Command getPathPlannerAuto(String name) {
        return new PathPlannerAuto(name);
    }

    public static Command getPathPlannerPath(String name) {
        return AutoBuilder.followPath(PathPlannerPath.fromPathFile(name));
    }

    public static Command resetAuto() {
        return sequence(
            intake.intakePivot.reset(0),
            climber.reset(),
            // runOnce(()-> swerve.zeroGyro(Robot.getAlliance() == Alliance.Red ? 0 : 180)),
            AmpMechanism.getInstance().reset(-90),
            runOnce(()-> swerve.resetEncoders()),
            runOnce(()-> Intake.getInstance().isRetracting = false)
        );
        
    }

    public static Command goToPoint(Pose2d pose) {
        return AutoBuilder.pathfindToPose(
                pose,
                AutoConstants.constraints,
                0.0, // Goal end velocity in meters/sec
                0.0 // Rotation delay distance in meters. This is how far the robot should travel before attempting to rotate.
            );
    }
    
}