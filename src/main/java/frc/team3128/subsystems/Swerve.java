package frc.team3128.subsystems;

import static frc.team3128.Constants.VisionConstants.SVR_STATE_STD;
import static frc.team3128.Constants.VisionConstants.SVR_VISION_MEASUREMENT_STD;

import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import com.ctre.phoenix6.hardware.Pigeon2;

import common.core.commands.NAR_PIDCommand;
import common.core.controllers.ControllerBase;
import common.core.swerve.SwerveBase;
import common.core.swerve.SwerveModule;
import common.hardware.motorcontroller.NAR_Motor.Control;
import common.utility.shuffleboard.NAR_Shuffleboard;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;

import static edu.wpi.first.wpilibj2.command.Commands.*;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Tracer;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.team3128.Robot;
import frc.team3128.RobotContainer;
import frc.team3128.commands.CmdSwerveDrive;
import frc.team3128.subsystems.random.Pose2dSupplier;

import static frc.team3128.Constants.SwerveConstants.*;
import static frc.team3128.Constants.FieldConstants.PROJECTILE_SPEED;
import static frc.team3128.Constants.FocalAimConstants.*;

public class Swerve extends SwerveBase {

    private static Swerve instance;

    private Pigeon2 gyro;

    public double throttle = 1;

    public Supplier<Double> yaw, pitch, roll;
    
    public Pose2dSupplier location;
    public boolean reorient;
    
    public final Translation2d focalPoint = Robot.getAlliance() == Alliance.Red ? focalPointRed : focalPointBlue;
    public final Translation2d speakerMidpoint = Robot.getAlliance() == Alliance.Red ? speakerMidpointRed : speakerMidpointBlue;
    
    public static ControllerBase rController;

    public static synchronized Swerve getInstance() {
        if (instance == null) {
            instance = new Swerve();
        }
        return instance;
    }

    private Swerve() {
        super(swerveKinematics, SVR_STATE_STD, SVR_VISION_MEASUREMENT_STD, Mod0, Mod1, Mod2, Mod3);
        chassisVelocityCorrection = false;
        location = () -> getPose();
        gyro = new Pigeon2(pigeonID);
        yaw = gyro.getYaw().asSupplier();
        pitch = gyro.getPitch().asSupplier();
        roll = gyro.getRoll().asSupplier();
        reorient = false;
        initShuffleboard();
        NAR_Shuffleboard.addData("Testing", "Name", ()-> getDist(speakerMidpointRed), 0, 0);
        rController = TURN_CONTROLLER;
        rController.enableContinuousInput(0, 360);
        rController.setMeasurementSource(()-> getYaw());
        rController.setTolerance(0.5);

        rController.setkV(NAR_Shuffleboard.debug("Test", "kV", rController.getkV(), 3, 1));
        rController.setkA(NAR_Shuffleboard.debug("Test", "kA", rController.getkA(), 3, 2));
    }

    public void setVoltage(double volts) {
        for (final SwerveModule module : modules) {
            module.getAngleMotor().set(0, Control.Position);
            module.getDriveMotor().setVolts(volts);
        }
    }

    public double getVelocity() {
        var x = getRobotVelocity();
        return Math.hypot(x.vxMetersPerSecond, x.vyMetersPerSecond);
    }

    @Override
    public double getYaw() {
        return yaw.get();
    }

    @Override
    public double getPitch() {
        return pitch.get();
    }

    @Override
    public double getRoll() {
        return roll.get();
    }

    @Override
    public void zeroGyro(double reset) {
        gyro.setYaw(reset);
    }
    
    public boolean atAngleSetpoint(Translation2d point){
        return getYaw() == getDesiredAngle(point, location.getAsPose2d().getTranslation());
    }
    
    //my code starts here
    public double getDesiredAngle(Translation2d point1, Translation2d point2){
        return Math.toDegrees(Math.atan2(point1.getY() - point2.getY(), point1.getX() - point2.getX())) + angleOffset;
    }
    
    // public void toggleReorient(boolean newReorient){
    //     reorient = newReorient;
    // }
    
    // public boolean getReorient(){
    //     return reorient;
    // }
    
    public Command reorientSpeaker(double timeInterval){
        return reorient(timeInterval);
    }
    
    public Translation2d getDesiredPosition(ChassisSpeeds velocity, double time){
        final Translation2d translation = new Translation2d(velocity.vxMetersPerSecond * time , velocity.vyMetersPerSecond * time);
        return location.getAsPose2d().getTranslation().plus(translation);
    }
    
    public double getProjectileTime(Translation2d position){
        return position.getDistance(focalPoint)/PROJECTILE_SPEED;
    }
    
    public Translation2d calculateTarget(ChassisSpeeds velocity, Translation2d position){
        // double predX = velocity.vxMetersPerSecond * getProjectileTime(position);
        double predY = velocity.vyMetersPerSecond * getProjectileTime(position);
        Translation2d target = new Translation2d(focalPoint.getX(), focalPoint.getY() - predY);
        return target;
    }
    
    public double calculateRotation(double setpoint, double timeInterval){
        return Units.degreesToRadians(rController.calculate(getGyroRotation2d().getDegrees(), setpoint));
    }
    
    public Command reorient(double timeInterval){
        Translation2d shootingPoint = getDesiredPosition(getFieldVelocity(), timeInterval);
        Translation2d targetPoint = calculateTarget(getFieldVelocity(), shootingPoint);
        double robotAngle = getDesiredAngle(shootingPoint, targetPoint);
        double climberAngle = Climber.getInstance().interpolate(shootingPoint.getDistance(targetPoint));
        return deadline(
            waitSeconds(timeInterval),
            repeatingSequence(
                parallel(
                    Climber.getInstance().setAngle(climberAngle),
                    runOnce(() -> drive(new Translation2d(
                    RobotContainer.controller.getLeftX(),RobotContainer.controller.getLeftY()).times(maxAttainableSpeed), 
                    calculateRotation(robotAngle, timeInterval), true))
                )
            )
        );
    }
    
    public double calculateAngleCurrent(){
        Translation2d targetPoint = calculateTarget(getFieldVelocity(), location.getAsPose2d().getTranslation());
        double climberAngle = Climber.getInstance().interpolate(location.getAsPose2d().getTranslation().getDistance(targetPoint));
        return climberAngle;
    }
    
    //kind of scuffed
    public Command reorientNonConstant(Translation2d point, double timeInterval){
        Translation2d shootingPoint = getDesiredPosition(getFieldVelocity(), timeInterval);
        Translation2d targetPoint = calculateTarget(getFieldVelocity(), shootingPoint);
        double robotAngle = getDesiredAngle(shootingPoint, targetPoint);
        return sequence (
            deadline(
                waitSeconds(timeInterval),
                repeatingSequence(
                    runOnce(() -> drive(new Translation2d(
                     RobotContainer.controller.getLeftX(),RobotContainer.controller.getLeftY()).times(maxAttainableSpeed), 
                     calculateRotation(robotAngle, timeInterval), true))
                )
        ),
        runOnce(() -> Climber.getInstance().setAngle(calculateAngleCurrent()))
        );
    }


    public double getSpeakerDist() {
        return getDist(speakerMidpoint);
    }
    
    public double getDist(Translation2d point) {
        return getPose().getTranslation().getDistance(point) - robotLength / 2.0;
    }

    public double getTurnAngle() {
        return getDesiredAngle(location.getAsPose2d().getTranslation(), focalPoint);
    }

    public Command turnInPlace() {
        return turnInPlace(()-> getDesiredAngle(location.getAsPose2d().getTranslation(), focalPoint));
    }
    //legit have no clue what this is...
    public Command turnInPlace(DoubleSupplier setpoint) {
        return new NAR_PIDCommand(
            TURN_CONTROLLER, 
            ()-> getYaw(), //measurement
            setpoint, //setpoint
            (double output) -> {
                final double x = RobotContainer.controller.getLeftX();
                final double y = RobotContainer.controller.getLeftY();
                Translation2d translation = new Translation2d(x,y).times(maxAttainableSpeed);//why the heck are you setting your translation2d to your x and y values from your controller? you're turning in place...right
                if (Robot.getAlliance() == Alliance.Red || !fieldRelative) {
                    translation = translation.rotateBy(Rotation2d.fromDegrees(90));
                }
                else {
                    translation = translation.rotateBy(Rotation2d.fromDegrees(-90));
                }

                Swerve.getInstance().drive(translation, Units.degreesToRadians(output), true);
            },
            Swerve.getInstance()
        ).beforeStarting(runOnce(()-> CmdSwerveDrive.disableTurn())); //dunno what this means
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
    

