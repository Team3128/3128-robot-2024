package frc.team3128.subsystems;

import static frc.team3128.Constants.SwerveConstants.Mod0;
import static frc.team3128.Constants.SwerveConstants.Mod1;
import static frc.team3128.Constants.SwerveConstants.Mod2;
import static frc.team3128.Constants.SwerveConstants.Mod3;
import static frc.team3128.Constants.SwerveConstants.pigeonID;
import static frc.team3128.Constants.SwerveConstants.swerveKinematics;
import static frc.team3128.Constants.VisionConstants.SVR_STATE_STD;
import static frc.team3128.Constants.VisionConstants.SVR_VISION_MEASUREMENT_STD;

import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import com.ctre.phoenix6.hardware.Pigeon2;

import common.core.commands.NAR_PIDCommand;
import common.core.controllers.TrapController;
import common.core.swerve.SwerveBase;
import common.core.swerve.SwerveModule;
import common.hardware.motorcontroller.NAR_Motor.Control;
import common.utility.shuffleboard.NAR_Shuffleboard;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import frc.team3128.RobotContainer;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;

import static frc.team3128.Constants.SwerveConstants.*;
import static frc.team3128.Constants.VisionConstants.*;
import static frc.team3128.Constants.FocalAimConstants.*;

import java.util.function.Supplier;

public class Swerve extends SwerveBase {

    private static Swerve instance;

    private Pigeon2 gyro;

    public double throttle = 1;

    public Supplier<Double> yaw, pitch, roll;

    public static synchronized Swerve getInstance() {
        if (instance == null) {
            instance = new Swerve();
        }
        return instance;
    }

    private Swerve() {
        super(swerveKinematics, SVR_STATE_STD, SVR_VISION_MEASUREMENT_STD, Mod0, Mod1, Mod2, Mod3);
        chassisVelocityCorrection = false;
        gyro = new Pigeon2(pigeonID);
        yaw = gyro.getYaw().asSupplier();
        pitch = gyro.getPitch().asSupplier();
        roll = gyro.getRoll().asSupplier();
        initShuffleboard();

        // NAR_Shuffleboard.addData("Tab", "Tab1", ()-> getRobotVelocity().toString(), 1, 0);
        // NAR_Shuffleboard.addData("Tab", "Tab2", ()-> getRobotVelocity().toString(), 2, 0);
        // NAR_Shuffleboard.addData("Tab", "Tab3", ()-> getRobotVelocity().toString(), 3, 0);
        
        // try {
            // final Field field = SwerveBase.class.getDeclaredField("useShuffleboard");
        //     field.setAccessible(true);
        //     field.setBoolean(this, false);
        // } catch (NoSuchFieldException e) {
        //     // TODO Auto-generated catch block
        //     e.printStackTrace();
        // } catch (SecurityException e) {
        //     // TODO Auto-generated catch block
        //     e.printStackTrace();
        // } catch (IllegalArgumentException e) {
        //     // TODO Auto-generated catch block
        //     e.printStackTrace();
        // } catch (IllegalAccessException e) {
        //     // TODO Auto-generated catch block
        //     e.printStackTrace();
        // }
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

    public double getDist(Translation2d aimPoint) {
        return Swerve.getInstance().getPose().getTranslation().getDistance(aimPoint) + distanceOffset;
    }

    public double getTurnAngle(Translation2d aimPoint) {
        Translation2d pos = Swerve.getInstance().getPose().getTranslation();
        return Math.toDegrees(Math.atan2(aimPoint.getY() - pos.getY(), aimPoint.getX() - pos.getX()));
    }

    public Command CmdTurnInPlace(DoubleSupplier setpoint) {
        TrapController trap = new TrapController(config, constraints);
        trap.enableContinuousInput(0, 360);
        trap.setTolerance(0.5);
        return new NAR_PIDCommand(trap, 
        ()-> Swerve.getInstance().getYaw(), //measurement
        setpoint, //setpoint
        (double output) -> {
            final double x = RobotContainer.controller.getLeftX();
            final double y = RobotContainer.controller.getLeftY();
            final Translation2d translation2d = new Translation2d(x,y).times(maxAttainableSpeed);

            Swerve.getInstance().drive(translation2d, Units.degreesToRadians(output), false);
        },
        Swerve.getInstance());
    }

    public Pigeon2 getGyro() {
        return gyro;
    }
}
    

