package frc.team3128.subsystems;

import static frc.team3128.Constants.VisionConstants.SVR_STATE_STD;
import static frc.team3128.Constants.VisionConstants.SVR_VISION_MEASUREMENT_STD;

import java.util.*;
import java.util.function.*;
import com.ctre.phoenix6.hardware.Pigeon2;
import common.core.controllers.Controller;
import common.core.swerve.SwerveBase;
import common.core.swerve.SwerveModule;
import common.utility.shuffleboard.NAR_Shuffleboard;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import frc.team3128.Constants.FieldConstants;
import frc.team3128.Constants.SwerveConstants;
import static frc.team3128.Constants.SwerveConstants.*;

public class Swerve extends SwerveBase {

    private static Swerve instance;

    private Pigeon2 gyro;

    public Supplier<Double> yaw;

    private final Controller translationController;
    private final Controller rotationController;

    private Translation2d translationSetpoint;
    private Supplier<Rotation2d> rotationSetpointSupplier;

    public static synchronized Swerve getInstance() {
        if (instance == null) {
            instance = new Swerve();
        }
        return instance;
    }

    private Swerve() {
        super(swerveKinematics, SVR_STATE_STD, SVR_VISION_MEASUREMENT_STD, Mod0, Mod1, Mod2, Mod3);
        chassisVelocityCorrection = false;
        Timer.delay(1);
        gyro = new Pigeon2(pigeonID);
        Timer.delay(1);
        gyro.getYaw().setUpdateFrequency(100);
        yaw = gyro.getYaw().asSupplier();

        gyro.optimizeBusUtilization();

        translationController = SwerveConstants.translationController;
        translationSetpoint = new Translation2d(0, 0);
        rotationController = SwerveConstants.rotationController;

        Rotation2d zeroRotation = new Rotation2d();
        rotationSetpointSupplier = () -> zeroRotation;

        translationController.disable();
        rotationController.disable();

        initShuffleboard();
        NAR_Shuffleboard.addData("Auto", "Setpoint", ()-> rotationController.atSetpoint());
        initStateCheck();
        setDefaultCommand(null);
    }

    @Override
    public double getYaw() {
        return yaw.get();
    }

    @Override
    public double getPitch() {
        return 0;
    }

    @Override
    public double getRoll() {
        return 0;
    }

    @Override
    public void resetGyroTo(double reset) {
        gyro.setYaw(reset);
    }

    public void driveOverridable(ChassisSpeeds velocity){
        if(translationController.isEnabled()){
            if(translationController.atSetpoint()){
                translationController.disable();
            }
            velocity.vxMetersPerSecond = translationController.calculate(getPose().getTranslation().getX(), translationSetpoint.getX());
            velocity.vyMetersPerSecond = translationController.calculate(getPose().getTranslation().getY(), translationSetpoint.getY());
        }

        if(rotationController.isEnabled()){
            if(rotationController.atSetpoint()){
                rotationController.disable();
            }
            velocity.omegaRadiansPerSecond = rotationController.calculate(getGyroRotation2d().getRadians(), rotationSetpointSupplier.get().getRadians());
        }

        super.drive(velocity);
    }

    public Command getDriveCommand(DoubleSupplier x, DoubleSupplier y, DoubleSupplier z){
        return new FunctionalCommand(
            ()-> {}, 
            ()-> driveOverridable(inputToChassisSpeeds(x, y, z)),
            (Boolean interrupted)-> stop(),
            ()-> false);
    }

    private ChassisSpeeds inputToChassisSpeeds(DoubleSupplier x, DoubleSupplier y, DoubleSupplier z){
        final Translation2d translation = FieldConstants.orthogonalizeInputs(x.getAsDouble(), y.getAsDouble()).times(maxAttainableSpeed);
        final double rotation = Math.pow(-z.getAsDouble(), 1.48) * maxAngularVelocity;
        return new ChassisSpeeds(translation.getX(), translation.getY(), rotation);
    }

    public void setPose(Pose2d pose){
        moveTo(pose.getTranslation());
        rotateTo(pose.getRotation());
    }

    public void moveTo(Translation2d translation) {
        translationSetpoint = translation;
        translationController.enable();
    }

    public void moveBy(Translation2d translation) {
        translationSetpoint = getPose().getTranslation().plus(translation);
        translationController.enable();
    }

    public void rotateTo(Rotation2d theta) {
        rotationSetpointSupplier = ()->theta;
        rotationController.enable();
    }
    
    public void rotateTo(Translation2d translation) {
        rotationSetpointSupplier = ()-> getAngleTo(translation);
        rotationController.enable();
    }

    public void rotateBy(Rotation2d dTheta) {
        Rotation2d curGyroRotation = getGyroRotation2d();
        rotationSetpointSupplier = ()-> curGyroRotation.plus(dTheta);
        rotationController.enable();
    }

    public void snapToAngle() {
        final Rotation2d gyroAngle = Swerve.getInstance().getGyroRotation2d();
        Rotation2d setpoint = Collections.min(
                            snapToAngles,
                            Comparator.comparing(
                                (Rotation2d angle) -> Math.abs(gyroAngle.minus(angle).getDegrees()))
                            );
        rotateTo(setpoint);
    }

    public boolean isConfigured() {
        for (final SwerveModule module : modules) {
            final double CANCoderAngle = module.getAbsoluteAngle().getDegrees();
            final double AngleMotorAngle = module.getAngleMotor().getPosition();
            if (CANCoderAngle == 0 || AngleMotorAngle == 0) return false;
        }
        return true;
    }

    public Pigeon2 getGyro() {
        return gyro;
    }

    @Override
    public void initShuffleboard(){
        super.initShuffleboard();
        NAR_Shuffleboard.addSendable("Commands", "Swerve Commands", this, 0, 0);
    }

}
    

