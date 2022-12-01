 package robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.sensors.CANCoder;
import com.ctre.phoenix.sensors.CANCoderConfiguration;
import com.ctre.phoenix.sensors.SensorTimeBase;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import robot.Constants.DriveTrainConstants;
import robot.Constants.SwerveModuleConstants;

public class SwerveModule {
    // ---------------- Various Devices ------------------
    private final WPI_TalonFX driveMotor;
    private final WPI_TalonFX turningMotor;
    private final CANCoder absoluteEncoder;

    private final PIDController turningPidController;

    private final boolean absoluteEncoderReversed;
    private final double absoluteEncoderOffsetRad;

    private final String swerveModuleID;

    private double tgtSpeed=0, tgtAngle=0;

    // Constructor
    public SwerveModule(int driveMotorId, int turningMotorId, boolean driveMotorReversed, boolean turningMotorReversed,
                        int absoluteEncoderId, double absoluteEncoderOffset, boolean absoluteEncoderReversed) {

        this.absoluteEncoderOffsetRad = absoluteEncoderOffset;
        this.absoluteEncoderReversed = absoluteEncoderReversed;

        if (driveMotorId == DriveTrainConstants.kFrontLeftDriveMotorPort){
            this.swerveModuleID = "Front Left";
        } else if (driveMotorId == DriveTrainConstants.kBackLeftDriveMotorPort){ 
            this.swerveModuleID = "Back Left";
        } else if (driveMotorId == DriveTrainConstants.kFrontRightDriveMotorPort){ 
            this.swerveModuleID = "Front Right";
        } else if (driveMotorId == DriveTrainConstants.kBackRightDriveMotorPort){ 
            this.swerveModuleID = "Back Right";
        } else { 
            this.swerveModuleID = "error";
        }

        // Drive Motor Config
        driveMotor = new WPI_TalonFX(driveMotorId);
        driveMotor.setInverted(driveMotorReversed);

        // Turning Motor Config
        turningMotor = new WPI_TalonFX(turningMotorId);
        turningMotor.setInverted(turningMotorReversed);

        // CANCoder Absolute Turning Encoder Config
        // Set units of the CANCoder to radians and velocity being radians per second
        absoluteEncoder = new CANCoder(absoluteEncoderId);

        CANCoderConfiguration config = new CANCoderConfiguration();
         
        config.sensorCoefficient = 2 * Math.PI / 4096.0;    // To Return Radians
        config.unitString= "rad";
        config.sensorTimeBase = SensorTimeBase.PerSecond;
        absoluteEncoder.configAllSettings(config);  // Send the config to the Encoder

        // Set Up PID Controller
        turningPidController = new PIDController(SwerveModuleConstants.kPTurning, 0, 0);
        turningPidController.enableContinuousInput(-Math.PI, Math.PI);

        resetEncoders();
    }

    public double getDrivePosition() {
        return (driveMotor.getSelectedSensorPosition() * SwerveModuleConstants.kDriveEncoderDistancePerUnitMeters);
    }

    public double getDriveVelocity() {
        double vel = driveMotor.getSelectedSensorVelocity();    // This is the counts for the last 100ms
        vel = vel * 10.0 * SwerveModuleConstants.kDriveEncoderDistancePerUnitMeters;  // This is the velocity in Meters Per Second
        return vel;
    }
    
    //public double getTurningPosition() {
    //    // ??? What is the point of this ... The motor has gear reduction and is NOT Absolute ???
    //    return turningEncoder.getPosition();
    //}

    public double getTurningVelocityDegrees() {
        double rotVel = turningMotor.getSelectedSensorVelocity();    // This is the counts for the last 100ms
        rotVel = rotVel * 10.0 * SwerveModuleConstants.kTurningEncoderDegreesPerEncoderCount;  // This is the velocity in Degrees Per Second
        return rotVel;
    }

    public double getTurningVelocityRadians() {
        double rotVel = turningMotor.getSelectedSensorVelocity();    // This is the counts for the last 100ms
        rotVel = rotVel * 10.0 * SwerveModuleConstants.kTurningEncoderRadiansPerEncoderCount;  // This is the velocity in Radians Per Second
        return rotVel;
    }

    // ----------  Absolute Encoder Methods -----------

    public double getAbsoluteEncoderRadians() {
        // returns 0 to 2PI radians
        return absoluteEncoder.getAbsolutePosition();
    }

    public double getAbsoluteEncoderDegrees() {
        // returns 0 to 360 Degrees
        return Math.toDegrees(getAbsoluteEncoderRadians());
    }
    
    public double getWHeelCurrentAngleRadians(){
        // Returns offset adjusted angle
        // Also  changes scale from 0 to 2PI to (0 to + PI) for CCW  or (0 to - PI) for CW rotation
        double angle = getAbsoluteEncoderRadians() - absoluteEncoderOffsetRad;;
        if ( absoluteEncoderOffsetRad <= Math.PI){
            // Offset <= PI (180 degrees)
            if ( angle > Math.PI ){
                return (angle - (2 * Math.PI));
            }
        } else {
            // Offset > PI (180 degrees)
            if ( angle < -Math.PI ){
                return (angle + (2 * Math.PI));
            }
        }
        return angle;
    }

    public double getWHeelCurrentAngleDegrees(){
        // Returns offset adjusted angle
        // Also changes scale from 0-360 to (0 to +180) for CCW or (0 to -180) for CW rotation
        double angle = getWHeelCurrentAngleRadians();
        return Math.toDegrees(angle);
    }

    // ------------------ Reset Encoders ---------------
    public void resetEncoders() {
        // Resets Drive Motor and Turning Motor encoder
        driveMotor.setSelectedSensorPosition(0);        // Set Drive Motor Distance to Zero
        turningMotor.setSelectedSensorPosition(0);      // Not really Needed ????
    }

    // ------------------ Get State of Swerve Drive ----------------
    public SwerveModuleState getState() {
        // returns State = Drive Velocity (Meter/sec.) and Wheel Angle (radians -PI to + PI)
        return new SwerveModuleState(getDriveVelocity(), new Rotation2d(getWHeelCurrentAngleRadians()));
    }

    // ---------------- Send Power to Motors ----------------------
    
    public void stop() {
        driveMotor.set(0);
        turningMotor.set(0);
    }

    public void setSingleModule(double speed, double angle) {
        // This is a manual drive of the wheels used only to test and get calibration data  
        // speed in Meters/sec , angle +PI (fully CCW) to -PI (fully CW)
        tgtAngle = angle;      
        tgtSpeed = speed;
        // convert speed in Meters/Sec to Motor Power -1 to +1
        double driveMotorPower = speed / DriveTrainConstants.kPhysicalMaxSpeedMetersPerSecond;
        // send power to Motors
        driveMotor.set(driveMotorPower);
        turningMotor.set(turningPidController.calculate(getWHeelCurrentAngleRadians(), tgtAngle));
    }

    public void setDesiredState(SwerveModuleState state, boolean isOpenLoop) {
        // This is the normal method used to drive the wheels!
        // Passed state provides - Wheel Velocity in Meters/Sec and Wheel Angle in Radians.
        if (Math.abs(state.speedMetersPerSecond) < 0.001) {
            // If the requested speed is too low just stop the motors and get out.
            stop(); 
            return;
        }

        // Compare the current Wheel Angle to Target and determine shortest route (Radians)
        state = SwerveModuleState.optimize(state, getState().angle);

        // Power the Drive Motor (This converts a command in meters/sec into -1.0 to +1.0)
        driveMotor.set(state.speedMetersPerSecond / DriveTrainConstants.kPhysicalMaxSpeedMetersPerSecond);

        // Power the Turning Motor - This uses a PID controller to lock in on Angle (Current Angle , Setpoint)
        turningMotor.set(turningPidController.calculate(getWHeelCurrentAngleRadians(), state.angle.getRadians()));
    }

    public void updateShuffleBoard(){
        SmartDashboard.putNumber( swerveModuleID + " Tgt Speed Meters", tgtSpeed);
        SmartDashboard.putNumber( swerveModuleID + " PID Tgt Angle Degrees", Math.toDegrees(tgtAngle));
        SmartDashboard.putNumber( swerveModuleID + " PID Tgt Angle Radians", tgtAngle);
        SmartDashboard.putNumber( swerveModuleID + " PID Absolute Encoder Radians", getAbsoluteEncoderRadians());
        SmartDashboard.putNumber( swerveModuleID + " PID Absolute Encoder Degrees", getAbsoluteEncoderDegrees());
        SmartDashboard.putNumber( swerveModuleID + " PID Corrected Absolute Encoder Radians", getWHeelCurrentAngleRadians());
        SmartDashboard.putNumber( swerveModuleID + " PID Corrected Absolute Encoder Degrees", getWHeelCurrentAngleDegrees());

        // SmartDashboard.putNumber( swerveModuleID + " Drive Encoder", getDriveWheelEncoderPosition());
        // SmartDashboard.putNumber( swerveModuleID + " Drive Inches", Units.metersToInches(getDrivePosition()));
        // SmartDashboard.putNumber( swerveModuleID + " Drive Meters", getDrivePosition());
    }

}
