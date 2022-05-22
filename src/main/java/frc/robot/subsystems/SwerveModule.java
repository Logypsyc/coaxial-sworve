package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.sensors.CANCoder;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.ModuleConstants;

public class SwerveModule {
    private final WPI_TalonFX driveMotor;
    private final WPI_TalonFX turningMotor;

    private final PIDController turningPIDController;

    private final CANCoder absoluteEncoder;
    private final boolean absoluteEncoderReversed;
    private final double absoluteEncoderOffsetRad;

    public SwerveModule (int driveMotorID, int turningMotorID, boolean driveMotorReversed, boolean turningMotorReversed, 
            int absoluteEncoderID, double absoluteEncoderOffset, boolean absoluteEncoderReversed) {
        
        this.absoluteEncoderOffsetRad = absoluteEncoderOffset;
        this.absoluteEncoderReversed = absoluteEncoderReversed;
        absoluteEncoder = new CANCoder(absoluteEncoderID);

        driveMotor = new WPI_TalonFX(driveMotorID);
        turningMotor = new WPI_TalonFX(turningMotorID);

        driveMotor.setInverted(driveMotorReversed);
        turningMotor.setInverted(turningMotorReversed);
        
        //it may be required to convert the units of the encoder values from the motors

        turningPIDController = new PIDController(ModuleConstants.kP_TURNING, 0, 0);
        turningPIDController.enableContinuousInput(-Math.PI, Math.PI);
        resetEncoders();
    }

    public double getDrivePosition() {
        return driveMotor.getSelectedSensorPosition();
    }

    public double getTurningPosition() {
        return turningMotor.getSelectedSensorPosition();
    }

    public double getDriveVelocity() {
        return driveMotor.getSelectedSensorVelocity();
    }

    public double getTurningVelocity() {
        return turningMotor.getSelectedSensorVelocity();
    }

    public double getAbsoluteEncoderRad() {
        double angle = absoluteEncoder.getPosition();
        angle *= 2.0 * Math.PI; //convert to radians
        angle -= absoluteEncoderOffsetRad; //subtract by offset to get actual angle
        return angle * (absoluteEncoderReversed ? -1.0 : 1.0); //multiply by -1 if it's reversed
    }

    public void resetEncoders() {
        driveMotor.setSelectedSensorPosition(0);
        turningMotor.setSelectedSensorPosition(getAbsoluteEncoderRad());
    }

    public SwerveModuleState getState() {
        return new SwerveModuleState(getDriveVelocity(), new Rotation2d(getTurningPosition()));
    }

    public void setDesiredState(SwerveModuleState state) {
        if (Math.abs(state.speedMetersPerSecond) < 0.001) { //check if the requested command has no driving velocity to ignore it
            stop();
            return;
        }


        state = SwerveModuleState.optimize(state, getState().angle); //optimize angle setpoint so you don't have to more more than 90 degrees
        driveMotor.set(state.speedMetersPerSecond / DriveConstants.PHYSICAL_MAX_SPEED_METERS_PER_SECOND);
        turningMotor.set(turningPIDController.calculate(getTurningPosition(), state.angle.getRadians()));
        SmartDashboard.putString("Swerve[" + absoluteEncoder.getPosition() + "] state", state.toString());
    }

    public void stop() {
        driveMotor.set(0);
        turningMotor.set(0);
    }
}
