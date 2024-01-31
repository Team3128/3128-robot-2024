package frc.team3128.commands;
import static frc.team3128.Constants.SwerveConstants.pigeonID;

import java.util.function.DoubleFunction;
import java.util.function.DoubleSupplier;
import com.ctre.phoenix6.hardware.Pigeon2;
import common.utility.shuffleboard.NAR_Shuffleboard;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import frc.team3128.subsystems.Swerve;

//gyro log + setting speed for swerve based on setpoint
public class CmdSetSpeed extends Command {
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
        //widget for adjusting speed, ie. 1rad/sec
        setpoint = NAR_Shuffleboard.debug("gyroInfo", "setpoint", 0, 0, 0);
        //gyro log below
        velocity = ()-> Units.degreesToRadians(gyro.getRate());
        acceleration = ()-> (Double)gyro.getAccelerationX().refresh().getValue();
        //future purposes to see whether it reaches setpoint
        position = ()-> gyro.getAngle();
        //originally registering as an object instead of DoubleSupplier, so conver to double then double supplier lol
        NAR_Shuffleboard.addData("gyroInfo", "position", ()-> position.getAsDouble(), 0, 0);
        NAR_Shuffleboard.addData("gyroInfo", "velocity", ()-> velocity.getAsDouble(), 1, 0);
        NAR_Shuffleboard.addData("gyroInfo", "acceleration", ()-> acceleration.getAsDouble(), 2, 0);
    
    
        }
    }
    

