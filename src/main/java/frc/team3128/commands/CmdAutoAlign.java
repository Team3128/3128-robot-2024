package frc.team3128.commands;
import static frc.team3128.Constants.LimelightConstants.*;
import static frc.team3128.Constants.SwerveConstants.maxSpeed;
import common.core.controllers.Controller;
import common.core.controllers.Controller.Type;
import common.hardware.limelight.Limelight;
import common.hardware.limelight.LimelightKey;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.team3128.subsystems.Intake;
import frc.team3128.subsystems.Swerve;

public class CmdAutoAlign extends WaitCommand {
    private Controller controller;
    private double plateauCount;
    private Swerve swerve;
    private Limelight limelight;

    public CmdAutoAlign() {
        super(TIMEOUT);
        swerve = Swerve.getInstance();
        limelight = null;
        controller = new Controller(config, Type.VELOCITY);
        controller.setTolerance(TX_THRESHOLD);
        addRequirements(swerve);
    }

    @Override
    public void initialize() {
        super.initialize();
        controller.setSetpoint(HORIZONTAL_OFFSET_GOAL);
    }

    @Override
    public void execute() {
        if (!limelight.hasValidTarget()) {
            plateauCount++;
            controller.reset();
            return;
        }

        plateauCount = 0;
        final double xOutput = controller.calculate(limelight.getValue(LimelightKey.HORIZONTAL_OFFSET));
        final double yOutput = Math.min(Math.sqrt(Math.pow(maxSpeed, 2) - Math.pow(xOutput, 2)), 0);    

        swerve.drive(new Translation2d(xOutput, yOutput), 0, false);
    }

    @Override
    public boolean isFinished() {
        return plateauCount > PLATEAU_THRESHOLD || Intake.getInstance().intakeRollers.hasObjectPresent() || super.isFinished();

    }

    @Override
    public void end(boolean interrupted) {
        swerve.stop();
    }
    
}
