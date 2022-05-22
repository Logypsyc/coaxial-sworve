package frc.robot.commands;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.subsystems.SwerveSubsystem;

public class SwerveJoystickCommand extends CommandBase {
    private final SwerveSubsystem swerveSubsystem;
    private final DoubleSupplier xSpeedFunction, ySpeedFunction, turningSpeedFunction;
    private final BooleanSupplier fieldOrientedFunction;
    private final SlewRateLimiter xLimiter, yLimiter, turningLimiter;
    

    public SwerveJoystickCommand(SwerveSubsystem swerveSubsystem, DoubleSupplier xSpeedFunction, DoubleSupplier ySpeedFunction, 
            DoubleSupplier turningSpeedFunction, BooleanSupplier fieldOrientedFunction) {
        this.swerveSubsystem = swerveSubsystem;
        this.xSpeedFunction = xSpeedFunction;
        this.ySpeedFunction = ySpeedFunction;
        this.turningSpeedFunction = turningSpeedFunction;
        this.fieldOrientedFunction = fieldOrientedFunction;
        this.xLimiter = new SlewRateLimiter(DriveConstants.TELEOP_MAX_ACCELERATION_UNITS_PER_SECOND);
        this.yLimiter = new SlewRateLimiter(DriveConstants.TELEOP_MAX_ACCELERATION_UNITS_PER_SECOND);
        this.turningLimiter = new SlewRateLimiter(DriveConstants.TELEOP_MAX_ANGULAR_ACCELERATION_UNITS_PER_SECOND);
        addRequirements(swerveSubsystem);
    }
    
    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
        //1. get joystick inputs
        double xSpeed = xSpeedFunction.getAsDouble();
        double ySpeed = ySpeedFunction.getAsDouble();
        double turningSpeed = turningSpeedFunction.getAsDouble();
        
        //2. apply deadband (if the joystick doesn't center back to exactly zero, we ignore any small inputs)
        xSpeed = Math.abs(xSpeed) > OIConstants.DEADBAND ? xSpeed : 0.0;
        ySpeed = Math.abs(ySpeed) > OIConstants.DEADBAND ? ySpeed : 0.0;
        turningSpeed = Math.abs(turningSpeed) > OIConstants.DEADBAND ? turningSpeed : 0.0;

        //3. make driving smoother;
        xSpeed = xLimiter.calculate(xSpeed) * DriveConstants.TELEOP_MAX_SPEED_METERS_PER_SECOND;
        ySpeed = yLimiter.calculate(ySpeed) * DriveConstants.TELEOP_MAX_SPEED_METERS_PER_SECOND;
        turningSpeed = turningLimiter.calculate(turningSpeed) * DriveConstants.TELEOP_MAX_ANGULAR_SPEED_RADIANS_PER_SECOND;

        //4. construct desired chassis speeds
        ChassisSpeeds chassisSpeeds;
        if (fieldOrientedFunction.getAsBoolean()) {
            //field-centric drive
            chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, turningSpeed, swerveSubsystem.getRotation2d());
            //the function will convert to the robot's local reference frame
        }
        else {
            //robot-centric drive
            chassisSpeeds = new ChassisSpeeds(xSpeed, ySpeed, turningSpeed);
        }

        //5. Convert chassis speeds to individual module states
        SwerveModuleState[] moduleStates = DriveConstants.DRIVE_KINEMATICS.toSwerveModuleStates(chassisSpeeds);

        //6. output each module state to wheels
        swerveSubsystem.setModuleStates(moduleStates);
    }

    @Override
    public void end(boolean interrupted) {
        swerveSubsystem.stopModules();
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
