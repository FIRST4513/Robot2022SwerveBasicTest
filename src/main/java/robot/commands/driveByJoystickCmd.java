// RobotBuilder Version: 4.0
//
// This file was generated by RobotBuilder. It contains sections of
// code that are automatically generated and assigned by robotbuilder.
// These sections will be updated in the future when you export to
// Java from RobotBuilder. Do not put any code or make any change in
// the blocks indicating autogenerated code or it will be lost on an
// update. Deleting the comments indicating the section will prevent
// it from being updated in the future.

// ROBOTBUILDER TYPE: Command.

package robot.commands;
import robot.RobotContainer;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import robot.Constants.DriveTrainConstants;
import robot.Constants.OIConstants;

import robot.subsystems.drivetrainSubSys;


public class driveByJoystickCmd extends CommandBase {

    private final drivetrainSubSys m_drivetrainSubSys;
    private Joystick m_joystick;

    public driveByJoystickCmd(drivetrainSubSys subsystem) {
        m_drivetrainSubSys = subsystem;
        addRequirements(m_drivetrainSubSys);

    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        m_joystick = RobotContainer.getInstance().getdriverJoy();
    }


    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {

        // Step 1 - Get joystick Inputs
        // Step 1a) Get joystick inputs from individual axis
            double xSpeed = m_joystick.getX();
            double ySpeed = -m_joystick.getY();
            double wheelAngle = m_joystick.getTwist();

            double throttle = (-m_joystick.getThrottle()/2) + 0.5; // convert from ( -1:1 ) to ( 0:1 ) 
            SmartDashboard.putNumber("Joy Drive ySpeed Raw 1", ySpeed);
            SmartDashboard.putNumber("Joy Drive xSpeed Raw 1", xSpeed);

        // Step 1b) Apply deadband (in case joystick doesn't return fully to Zero position)
            xSpeed = deadBand(xSpeed);  // Not Used in this program
            ySpeed = deadBand(ySpeed);
            wheelAngle = deadBand(wheelAngle);
            SmartDashboard.putNumber("Joy Drive ySpeed Raw 2", ySpeed);

        // Step 1c) Limit Speeds based on throttle setting
            xSpeed = xSpeed * throttle;  // Not Used in this program
            ySpeed = ySpeed * throttle;
            //wheelAngle = wheelAngle * throttle; //
            SmartDashboard.putNumber("Joy Drive ySpeed Raw 3", ySpeed);

        // Step 2a - Convert Joystick values to Field Velocity (Meters/Sec)
            xSpeed = xSpeed * DriveTrainConstants.kTeleDriveMaxSpeedMetersPerSecond;  // Not Used in this program
            ySpeed = ySpeed * DriveTrainConstants.kTeleDriveMaxSpeedMetersPerSecond;

        SmartDashboard.putNumber("Joy Drive ySpeed Meters", ySpeed);

        // Step 2b - Convert Joystick angle -1 to +1  into -PI cw to +PI ccw
            wheelAngle = (wheelAngle * -Math.PI);

        // Step 2b - Convert Joystick angle -1 to +1  into -180 cw to +180 ccw
        //    wheelAngle = (wheelAngle * -180.0);

        SmartDashboard.putNumber("Joy Wheel Angle Radians", wheelAngle);
        SmartDashboard.putNumber("Joy Wheel Angle Degrees", Math.toDegrees(wheelAngle));

        // Send raw data to swerve module (Speed Meters/Sec , Wheel Angle Radians)
        // 
        if (m_joystick.getRawButton(5)){
            m_drivetrainSubSys.setSingleModuleState(DriveTrainConstants.kFrontLeftDriveMotorPort, ySpeed, wheelAngle);
        } else if (m_joystick.getRawButton(6)){
            m_drivetrainSubSys.setSingleModuleState(DriveTrainConstants.kFrontRightDriveMotorPort, ySpeed, wheelAngle);
        } else if (m_joystick.getRawButton(3)){
            m_drivetrainSubSys.setSingleModuleState(DriveTrainConstants.kBackLeftDriveMotorPort, ySpeed, wheelAngle);
        } else if (m_joystick.getRawButton(4)){
            m_drivetrainSubSys.setSingleModuleState(DriveTrainConstants.kBackRightDriveMotorPort, ySpeed, wheelAngle);
        } else if (m_joystick.getRawButton(1)){
            m_drivetrainSubSys.setSingleModuleState(DriveTrainConstants.kFrontLeftDriveMotorPort, ySpeed, wheelAngle);
            m_drivetrainSubSys.setSingleModuleState(DriveTrainConstants.kFrontRightDriveMotorPort, ySpeed, wheelAngle);
            m_drivetrainSubSys.setSingleModuleState(DriveTrainConstants.kBackLeftDriveMotorPort, ySpeed, wheelAngle);
            m_drivetrainSubSys.setSingleModuleState(DriveTrainConstants.kBackRightDriveMotorPort, ySpeed, wheelAngle);
        } else {
            m_drivetrainSubSys.stopModules();
        }

        /*
        // Step 3 - Create a "Chassis Speeds" Object from field velocity targets and current Gyro Angle
        ChassisSpeeds chassisSpeeds;
        if(  m_joystick.getRawButton(OIConstants.kDriverFieldOrientedButtonIdx)){
            // Field Relative
            chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(
                    xSpeed, ySpeed, wheelAngle, m_drivetrainSubSys.getGyroHeadingRotation2d());
        } else {
            // Chassis Relative
            chassisSpeeds = new ChassisSpeeds(xSpeed, ySpeed, wheelAngle);
        }

        // Step 4 - Create a "Swerve Module States" Array object from the "chassis Speeds" object
        // This creates a SwerveModuleState Array of Swerve Drive States.
        // Each array element contains "Drive-Velocity" and "Turn-Angle" values. 
        SwerveModuleState[] moduleStates = DriveTrainConstants.kDriveKinematics.toSwerveModuleStates(chassisSpeeds);

        // Step 5 - Send "Swerve Module States" Array to Drivetrain Motors
        m_drivetrainSubSys.setModuleStates(moduleStates);
        */
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        m_drivetrainSubSys.stopModules();
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }

    public double deadBand(double value){
        // Deadband Calculation
        if((value <=  OIConstants.kDeadband) && (value >= - OIConstants.kDeadband)){
            return 0;
        }
        if (value > 0) {
            value=(value - OIConstants.kDeadband) * (1 + OIConstants.kDeadband);		// Scale Yvalue smoothly to + 1
        } else {
            value = - (-value - OIConstants.kDeadband) * (1 + OIConstants.kDeadband);	// Scale Yvalue smoothly to -1
        }
        return value;
    }


    @Override
    public boolean runsWhenDisabled() {
        // BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=DISABLED
        return false;

    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=DISABLED
    }
}
