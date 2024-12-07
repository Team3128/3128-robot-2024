package frc.team3128.subsystems;

import common.hardware.motorcontroller.NAR_CANSpark;
import common.hardware.motorcontroller.NAR_Motor;
import common.utility.narwhaldashboard.NarwhalDashboard;
import common.utility.narwhaldashboard.NarwhalDashboard.State;
import common.utility.shuffleboard.NAR_Shuffleboard;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.interfaces.Gyro;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj.RobotBase;

import static edu.wpi.first.wpilibj2.command.Commands.sequence;

import com.ctre.phoenix6.hardware.Pigeon2;

import common.core.misc.NAR_Robot;

public class Tank extends SubsystemBase {
    private NAR_CANSpark leftMotor, rightMotor;
    private DifferentialDrive drive;
    private DifferentialDriveOdometry odometry;
    private Pigeon2 gyro;
    private NarwhalDashboard dashboard;

    public Tank(NAR_CANSpark leftMotor, NAR_CANSpark rightMotor, Pigeon2 gyro) {
        this.leftMotor = leftMotor;
        this.rightMotor = rightMotor;
        this.gyro = gyro;


        this.drive = new DifferentialDrive(leftMotor.getMotor(), rightMotor.getMotor());
        this.odometry = new DifferentialDriveOdometry(gyro.getRotation2d(), leftMotor.getPosition(), rightMotor.getPosition());
        this.dashboard = NarwhalDashboard.getInstance();
    }

    public void helloWorld(){
        System.out.print("Hello World");
    }

    public void tankDrive(double leftSpeed, double rightSpeed) {
        drive.tankDrive(leftSpeed, rightSpeed);
    }

    public State getRunningState() {
        //how do you find the state?
        if (leftMotor.getState() == State.DISCONNECTED) return State.DISCONNECTED;
        if (rightMotor.getState() == State.DISCONNECTED) return State.DISCONNECTED;
        return State.RUNNING;
    }


    public void setPower(double leftVolts, double rightVolts) {
        double voltage = RobotController.getBatteryVoltage();
        if (voltage > 0) {
            tankDrive(leftVolts / voltage, rightVolts / voltage);
        } else {
            stop();
        }
    }


    public void resetPose(Pose2d poseMeters) {
        resetEncoders();
        odometry.resetPosition(gyro.getRotation2d(), 0, 0, poseMeters);
    }


    public void resetPose() {
        resetGyro();
        resetPose(new Pose2d(0, 0, gyro.getRotation2d()));
    }


    public void resetGyro() {
        gyro.reset();
    }


    public void resetEncoders() {
        leftMotor.resetPosition(0);
        rightMotor.resetPosition(0);
    }

    public void stop() {
        tankDrive(0, 0);
    }

    public void initDashboard() {
        NAR_Shuffleboard.addData("tank", "robotX", odometry.getPoseMeters().getX());
        NAR_Shuffleboard.addData("tank", "robotY", odometry.getPoseMeters().getY());
        NAR_Shuffleboard.addData("tank", "robotYaw", odometry.getPoseMeters().getRotation().getDegrees());
    }
}