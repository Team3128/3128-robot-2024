package frc.team3128.commands;
import static frc.team3128.Constants.LimelightConstants.*;
import common.hardware.limelight.Limelight;
import common.hardware.limelight.LimelightKey;
import common.utility.shuffleboard.NAR_Shuffleboard;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.team3128.RobotContainer;
import frc.team3128.subsystems.Intake;
import frc.team3128.subsystems.Swerve;

public class CmdAutoAlign extends WaitCommand {
    public static boolean hasTimedOut = false;
    private PIDController controller;
    private double plateauCount;
    private Swerve swerve;
    private Limelight limelight;
    private double maxSpeed;

    public CmdAutoAlign(double maxSpeed, double timeout, boolean shouldRequire) {
        super(timeout);
        swerve = Swerve.getInstance();
        limelight = RobotContainer.limelight;
        controller = new PIDController(KP, KI, KD);
        controller.setTolerance(TX_THRESHOLD);
        this.maxSpeed = maxSpeed;
        if (shouldRequire) addRequirements(swerve);
    }

    @Override
    public void initialize() {
        super.initialize();
        controller.setSetpoint(HORIZONTAL_OFFSET_GOAL);
    }

    @Override
    public void execute() {
        final double strafe = -controller.calculate(limelight.getValue(LimelightKey.HORIZONTAL_OFFSET));
        NAR_Shuffleboard.addData("Limelight", "Output", strafe, 2, 2);
        final double forward = -Math.sqrt(Math.max(Math.pow(maxSpeed, 2) - Math.pow(strafe, 2), 0));
        NAR_Shuffleboard.addData("Limelight", "Forward", forward, 2, 3);

        if (!limelight.hasValidTarget()) {
            plateauCount++;
            controller.reset();
            // swerve.stop();

            // Continue to go forwards even target not visible to prevent missing a note
            swerve.drive(new ChassisSpeeds(forward, 0, 0));
            return;
        }

        plateauCount = 0;
        swerve.drive(new ChassisSpeeds(forward, strafe, 0));
    }

    @Override
    public boolean isFinished() {
        if (super.isFinished()) hasTimedOut = true;
        return plateauCount > PLATEAU_THRESHOLD || Intake.getInstance().intakeRollers.hasObjectPresent() || hasTimedOut;
    }

    @Override
    public void end(boolean interrupted) {
        super.end(interrupted);
        swerve.stop();
    }

}