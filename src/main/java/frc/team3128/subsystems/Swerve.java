package frc.team3128.subsystems;

import com.ctre.phoenix6.hardware.Pigeon2;

import common.core.swerve.SwerveBase;
import common.core.swerve.SwerveModule;
import common.hardware.motorcontroller.NAR_Motor.Control;
import common.utility.shuffleboard.NAR_Shuffleboard;

import static frc.team3128.Constants.SwerveConstants.*;
import static frc.team3128.Constants.VisionConstants.*;

import java.lang.reflect.Field;
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
        //     final Field field = SwerveBase.class.getDeclaredField("useShuffleboard");
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

}