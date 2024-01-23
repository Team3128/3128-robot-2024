package frc.team3128.commands;

import static frc.team3128.Constants.SwerveConstants.pigeonID;

import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import com.ctre.phoenix6.hardware.Pigeon2;

import common.utility.shuffleboard.NAR_Shuffleboard;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import frc.team3128.subsystems.Swerve;

public class CmdSetSpeed extends Command{
    //logs your setpoint from shuffleboard and sets a speed based off of that
    private DoubleSupplier setpoint;
    private Swerve swerve;
    private DoubleSupplier acceleration;
    private DoubleSupplier velocity;
    private Pigeon2 gyro;
    private DoubleSupplier position;
    public CmdSetSpeed() {
        swerve = Swerve.getInstance();
        gyro = new Pigeon2(pigeonID);
        gyro.setYaw(0);
    }

    @Override
    public void execute() {
        swerve.drive(new Translation2d(0,0), setpoint.getAsDouble(), false);
    }

    
    public void initShuffleboard() {
        setpoint = NAR_Shuffleboard.debug("gyroInfo", "setpoint", 0, 0, 0);
        velocity = ()-> Units.degreesToRadians(gyro.getRate()); //in degrees, then convert to rad
        acceleration = ()-> (Double)gyro.getAccelerationX().refresh().getValue();
        //future purposes to see whether it reaches setpoint
        // position = ()-> gyro.getRotation2d().getDegrees();
        position = ()-> gyro.getAngle();
        NAR_Shuffleboard.addData("gyroInfo", "position", ()-> position.getAsDouble(), 0, 0);
        NAR_Shuffleboard.addData("gyroInfo", "velocity", ()-> velocity.getAsDouble(), 1, 0);
        NAR_Shuffleboard.addData("gyroInfo", "acceleration", ()-> acceleration.getAsDouble(), 2, 0);
    
    
        }

    }
    

