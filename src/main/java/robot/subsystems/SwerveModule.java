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

        SmartDashboard.putNumber(swerveModuleID + "Absolute Encoder Radians", getAbsoluteEncoderRad());

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

    public double getAbsoluteEncoderRaw() {
        return absoluteEncoder.getAbsolutePosition();
    }

    public double getAbsoluteEncoderRawDegrees() {
        return Math.toDegrees(getAbsoluteEncoderRaw());
    }
    
    public double getAbsoluteEncoder() {
        return getAbsoluteEncoderRaw() - Math.PI;
    }

    public double getAbsoluteEncoderRad() {
        double angle = getAbsoluteEncoder();                     // Position in Radians
        angle += absoluteEncoderOffsetRad;// Correct for Sensor Misaligned
        angle -= Math.PI;
        //angle -= angle * (absoluteEncoderReversed ? -1.0 : 1.0);  // Change sign as needed
        return angle;
    }

    public double getAbsoluteEncoderDegrees() {
        double angle = absoluteEncoder.getPosition();            // Position in Radians
        angle -= absoluteEncoderOffsetRad;                       // Correct for Sensor Misaligned
        angle = angle * (absoluteEncoderReversed ? -1.0 : 1.0);  // Change sign as needed
        angle = Math.toDegrees(angle);                           // Change to Degrees 
        return angle;
    }

    public double getDriveWheelEncoderPosition(){
        return turningMotor.get();
    }

    public double getAbsoluteEncoderPosition() {
        return absoluteEncoder.getAbsolutePosition();
    }

    public void resetEncoders() {
        driveMotor.setSelectedSensorPosition(0);        // Set Drive Motor Distance to Zero
        turningMotor.setSelectedSensorPosition(0);      // Not really Needed ????
    }

    public SwerveModuleState getState() {
        return new SwerveModuleState(getDriveVelocity(), new Rotation2d(getAbsoluteEncoderRad()));
    }

    public void setSingleModule(double speed, double angle) {  
        // speed in Meters/sec , angle in Degrees -180 Full CW 0 to +180 Full CCW     
        tgtSpeed = speed;
        tgtAngle = angle;
        driveMotor.set(speed / DriveTrainConstants.kPhysicalMaxSpeedMetersPerSecond); // 
        turningMotor.set(turningPidController.calculate(getAbsoluteEncoderRad(), Math.toRadians(angle)));
    }

    public void setDesiredState(SwerveModuleState state) {
        // Passed state provides - Wheel Velocity in Meters/Sec and Wheel Angle in Radians.
        if (Math.abs(state.speedMetersPerSecond) < 0.001) {
            // If the requested speed is too low just stop the motors and get out.
            stop(); 
            return;
        }
        // Compare the current Wheel Angle to Target and determine shortest route
        state = SwerveModuleState.optimize(state, getState().angle);
        // Power the Drive Motor (This converts a command in meters/sec into -1.0 to +1.0)
        driveMotor.set(state.speedMetersPerSecond / DriveTrainConstants.kPhysicalMaxSpeedMetersPerSecond);
        // Power the Turning Motor - This uses a PID controller to lock in on Angle (Current Angle , Setpoint)
        turningMotor.set(turningPidController.calculate(getAbsoluteEncoderRad(), state.angle.getRadians()));
    }

    public void updateShuffleBoard(){
        //SmartDashboard.putNumber( swerveModuleID + "Absolute Encoder Rad", state.angle.getRadians());
        //SmartDashboard.putNumber( swerveModuleID + "Absolute Encoder Rad", getAbsoluteEncoderRad());

        SmartDashboard.putNumber( swerveModuleID + " Tgt Speed", tgtSpeed);
        SmartDashboard.putNumber( swerveModuleID + " PID Tgt Angle Degrees", tgtAngle);
        SmartDashboard.putNumber( swerveModuleID + " PID Tgt Angle Rad", Math.toRadians(tgtAngle));

        SmartDashboard.putNumber( swerveModuleID + " PID Absolute Encoder Raw", getAbsoluteEncoderRaw());
        SmartDashboard.putNumber( swerveModuleID + " PID Corrected Absolute Encoder Rad", getAbsoluteEncoderRad());
        SmartDashboard.putNumber( swerveModuleID + " PID Absolute Encode Raw Degrees", getAbsoluteEncoderRawDegrees());
    }

        // SmartDashboard.putNumber( swerveModuleID + " Turn Encoder", getAbsoluteEncoderPosition());
        // SmartDashboard.putNumber( swerveModuleID + " Raw Encoder Angle", absoluteEncoder.getPosition());
        // SmartDashboard.putNumber( swerveModuleID + " Turn Angle Degrees", getAbsoluteEncoderDegrees());
        // SmartDashboard.putNumber( swerveModuleID + " Turn Angle Radians", getAbsoluteEncoderRad());

        // SmartDashboard.putNumber( swerveModuleID + " Drive Encoder", getDriveWheelEncoderPosition());
        // SmartDashboard.putNumber( swerveModuleID + " Drive Inches", Units.metersToInches(getDrivePosition()));
        // SmartDashboard.putNumber( swerveModuleID + " Drive Meters", getDrivePosition());
    

    public void stop() {
        driveMotor.set(0);
        turningMotor.set(0);
    }
}
