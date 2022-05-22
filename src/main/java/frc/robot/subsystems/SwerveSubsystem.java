package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;

public class SwerveSubsystem extends SubsystemBase {
    private final SwerveModule frontLeft = new SwerveModule(
        DriveConstants.FRONT_LEFT_DRIVE_MOTOR_PORT,
        DriveConstants.FRONT_LEFT_TURNING_MOTOR_PORT,
        DriveConstants.FRONT_LEFT_DRIVE_MOTOR_REVERSED,
        DriveConstants.FRONT_LEFT_TURNING_MOTOR_REVERSED,
        DriveConstants.FRONT_LEFT_DRIVE_ABSOLUTE_ENCODER_PORT,
        DriveConstants.FRONT_LEFT_DRIVE_ABSOLUTE_ENCODER_OFFSET_RAD,
        DriveConstants.FRONT_LEFT_DRIVE_ABSOLUTE_ENCODER_REVERSED);

    private final SwerveModule frontRight = new SwerveModule(
        DriveConstants.FRONT_RIGHT_DRIVE_MOTOR_PORT,
        DriveConstants.FRONT_RIGHT_TURNING_MOTOR_PORT,
        DriveConstants.FRONT_RIGHT_DRIVE_MOTOR_REVERSED,
        DriveConstants.FRONT_RIGHT_TURNING_MOTOR_REVERSED,
        DriveConstants.FRONT_RIGHT_DRIVE_ABSOLUTE_ENCODER_PORT,
        DriveConstants.FRONT_RIGHT_DRIVE_ABSOLUTE_ENCODER_OFFSET_RAD,
        DriveConstants.FRONT_RIGHT_DRIVE_ABSOLUTE_ENCODER_REVERSED);
    
    private final SwerveModule backLeft = new SwerveModule(
        DriveConstants.BACK_LEFT_DRIVE_MOTOR_PORT,
        DriveConstants.BACK_LEFT_TURNING_MOTOR_PORT,
        DriveConstants.BACK_LEFT_DRIVE_MOTOR_REVERSED,
        DriveConstants.BACK_LEFT_TURNING_MOTOR_REVERSED,
        DriveConstants.BACK_LEFT_DRIVE_ABSOLUTE_ENCODER_PORT,
        DriveConstants.BACK_LEFT_DRIVE_ABSOLUTE_ENCODER_OFFSET_RAD,
        DriveConstants.BACK_LEFT_DRIVE_ABSOLUTE_ENCODER_REVERSED);

    private final SwerveModule backRight = new SwerveModule(
        DriveConstants.BACK_RIGHT_DRIVE_MOTOR_PORT,
        DriveConstants.BACK_RIGHT_TURNING_MOTOR_PORT,
        DriveConstants.BACK_RIGHT_DRIVE_MOTOR_REVERSED,
        DriveConstants.BACK_RIGHT_TURNING_MOTOR_REVERSED,
        DriveConstants.BACK_RIGHT_DRIVE_ABSOLUTE_ENCODER_PORT,
        DriveConstants.BACK_RIGHT_DRIVE_ABSOLUTE_ENCODER_OFFSET_RAD,
        DriveConstants.BACK_RIGHT_DRIVE_ABSOLUTE_ENCODER_REVERSED);

    private AHRS gyro = new AHRS(SPI.Port.kMXP);
    private final SwerveDriveOdometry odometer = new SwerveDriveOdometry(DriveConstants.DRIVE_KINEMATICS, new Rotation2d(0));

    public SwerveSubsystem() {
        new Thread(() -> {
            try {
                Thread.sleep(1000); //wait 1 second before resetting gyro to allow it to recalibrate first
                zeroHeading();
            } catch (Exception e) {
            }
        }).start();
    }

    public void zeroHeading() {
        gyro.reset();
    }

    public double getHeading() {
        return Math.IEEEremainder(gyro.getAngle(), 360); //keeps reading between -180 and 180 degrees
    }

    public Rotation2d getRotation2d() {
        return Rotation2d.fromDegrees(getHeading());
    }

    public Pose2d getPose() {
        return odometer.getPoseMeters();
    }

    public void resetOdometry(Pose2d pose) {
        odometer.resetPosition(pose, getRotation2d());
    }

    @Override
    public void periodic() {
        odometer.update(getRotation2d(), frontLeft.getState(), frontRight.getState(), backLeft.getState(), backRight.getState());
        SmartDashboard.putNumber("Robot Heading", getHeading());
        SmartDashboard.putString("Robot Location", getPose().getTranslation().toString());
    }

    public void stopModules() {
        frontLeft.stop();
        frontRight.stop();
        backLeft.stop();
        backRight.stop();
    }

    public void setModuleStates(SwerveModuleState[] desiredStates) {
        SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, DriveConstants.PHYSICAL_MAX_SPEED_METERS_PER_SECOND);
        frontLeft.setDesiredState(desiredStates[0]);
        frontRight.setDesiredState(desiredStates[1]);
        backLeft.setDesiredState(desiredStates[2]);
        backRight.setDesiredState(desiredStates[3]);
    }
}