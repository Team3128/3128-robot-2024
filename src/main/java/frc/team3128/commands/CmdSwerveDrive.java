package frc.team3128.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.team3128.Robot;
import frc.team3128.subsystems.Swerve;
import static frc.team3128.Constants.SwerveConstants.*;

public class CmdSwerveDrive extends Command {
    private final Swerve swerve;

    private double rotation;
    private Translation2d translation;

    private final DoubleSupplier xAxis;
    private final DoubleSupplier yAxis;
    private final DoubleSupplier zAxis;

    private final SlewRateLimiter accelLimiter;

    private final PIDController rController;
    public static boolean enabled = false;
    public static double rSetpoint;
    
    public CmdSwerveDrive(DoubleSupplier xAxis, DoubleSupplier yAxis, DoubleSupplier zAxis, boolean fieldRelative) {
        this.swerve = Swerve.getInstance();
        addRequirements(swerve);

        this.xAxis = xAxis;
        this.yAxis = yAxis;
        this.zAxis = zAxis;

        accelLimiter = new SlewRateLimiter(maxAcceleration);
        rController = new PIDController(turnKP, 0, 0);
        rController.enableContinuousInput(0, 360);
        rController.setTolerance(0.5);

        swerve.fieldRelative = fieldRelative;
    }

    @Override
    public void execute() {
        // deadbands are taken care of in NAR_Joystick
        // TODO: add in slewratelimiter here
        translation = new Translation2d(xAxis.getAsDouble(), yAxis.getAsDouble()).times(swerve.throttle).times(maxAttainableSpeed);
        if (Robot.getAlliance() == Alliance.Red || !swerve.fieldRelative) {
            translation = translation.rotateBy(Rotation2d.fromDegrees(90));
        }
        else {
            translation = translation.rotateBy(Rotation2d.fromDegrees(-90));
        }

        final double zValue = -zAxis.getAsDouble();
        
        rotation = Math.copySign(Math.pow(zValue, 3/2), zValue) * maxAngularVelocity * swerve.throttle; 

        if (Math.abs(rotation) > maxAngularVelocity * swerve.throttle / 4.0) {
            enabled = false;
        }
        if (enabled) {
            rotation = Units.degreesToRadians(rController.calculate(swerve.getGyroRotation2d().getDegrees(), rSetpoint));
            if (rController.atSetpoint()) {
                rotation = 0;
            }
        }

        Rotation2d driveAngle = translation.getAngle();
        double slowedDist = accelLimiter.calculate(translation.getNorm());
        // translation = new Translation2d(slowedDist, driveAngle);

        SmartDashboard.putBoolean("fieldOriented",swerve.fieldRelative);
        SmartDashboard.putNumber("yAXIS",yAxis.getAsDouble());
        SmartDashboard.putNumber("xAXIS",xAxis.getAsDouble());
        swerve.drive(translation, rotation, swerve.fieldRelative);

    }

    public static void setTurnSetpoint() {
        final double currentRotation = MathUtil.inputModulus(Swerve.getInstance().getYaw(), 0, 360);
        if (currentRotation <= 45) {
            rSetpoint = 0;
        }
        else if (currentRotation <= 135) {
            rSetpoint = 90;
        }
        else if (currentRotation <= 225) {
            rSetpoint = 180;
        }
        else if (currentRotation <= 315) {
            rSetpoint = 270;
        }
        else {
            rSetpoint = 360;
        }
        enabled = true;
    }

    @Override
    public void end(boolean interrupted) {
        swerve.stop();
    }
}