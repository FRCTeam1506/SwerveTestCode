package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.RemoteSensorSource;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import com.ctre.phoenix.sensors.AbsoluteSensorRange;
import com.ctre.phoenix.sensors.CANCoder;
import com.ctre.phoenix.sensors.CANCoderConfiguration;

import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.util.Units;

import frc.robot.Constants.SwerveDrive;

public class SwerveModuleMK3 {

  public String name;

  private TalonFX driveMotor;
  private TalonFX steerMotor;
  private CANCoder encoder;

  public SwerveModuleMK3(TalonFX driveMotor, int steerMotorId, boolean isInverted, int encoderId, Rotation2d offset, String name) {

    this.name = name;
    this.driveMotor = driveMotor;

    this.steerMotor = new TalonFX(steerMotorId);
    this.encoder = new CANCoder(encoderId);


    // * Drive motor config
    TalonFXConfiguration driveTalonFXConfiguration = new TalonFXConfiguration();
    driveTalonFXConfiguration.slot0.kP = SwerveDrive.kDriveP;
    driveTalonFXConfiguration.slot0.kI = SwerveDrive.kDriveI;
    driveTalonFXConfiguration.slot0.kD = SwerveDrive.kDriveD;
    driveTalonFXConfiguration.slot0.kF = SwerveDrive.kDriveF;

    this.driveMotor.configAllSettings(driveTalonFXConfiguration);
    this.driveMotor.setNeutralMode(NeutralMode.Brake);
    

    // * Steer motor config
    TalonFXConfiguration angleTalonFXConfiguration = new TalonFXConfiguration();
    angleTalonFXConfiguration.slot0.kP = SwerveDrive.kAngleP;
    angleTalonFXConfiguration.slot0.kI = SwerveDrive.kAngleI;
    angleTalonFXConfiguration.slot0.kD = SwerveDrive.kAngleD;

    // Use the CANCoder as the remote sensor for the primary TalonFX PID
    angleTalonFXConfiguration.remoteFilter0.remoteSensorDeviceID = this.encoder.getDeviceID();
    angleTalonFXConfiguration.remoteFilter0.remoteSensorSource = RemoteSensorSource.CANCoder;
    
    angleTalonFXConfiguration.primaryPID.selectedFeedbackSensor = FeedbackDevice.RemoteSensor0;
    this.steerMotor.configAllSettings(angleTalonFXConfiguration);
    this.steerMotor.setInverted(isInverted);
    this.steerMotor.setNeutralMode(NeutralMode.Coast); // not needed but nice to keep the robot stopped when you want it stopped

    CANCoderConfiguration canCoderConfiguration = new CANCoderConfiguration();
    canCoderConfiguration.magnetOffsetDegrees = offset.getDegrees();
    this.encoder.setPosition(0.0);
    this.encoder.configAllSettings(canCoderConfiguration);
  }


  /**
   * Gets the relative rotational position of the module
   * @return The relative rotational position of the angle motor in degrees
   */
  public Rotation2d getAngle() {
    return Rotation2d.fromDegrees(this.encoder.getAbsolutePosition()); //include angle offset
  }

  public double getRawAngle() {
    return this.encoder.getAbsolutePosition(); //include angle offset
  }
  //:)
  /**
   * Set the speed + rotation of the swerve module from a SwerveModuleState object
   * @param desiredState - A SwerveModuleState representing the desired new state of the module
   */
  public void setDesiredState(SwerveModuleState desiredState) {

    Rotation2d currentRotation = getAngle();
    SwerveModuleState state = SwerveModuleState.optimize(desiredState, currentRotation);
    
    // Find the difference between our current rotational position + our new rotational position
    Rotation2d rotationDelta = state.angle.minus(currentRotation);
    
    // Find the new absolute position of the module based on the difference in rotation
    double deltaTicks = (rotationDelta.getDegrees() / 360) * SwerveDrive.kEncoderTicksPerRotation;
    // Convert the CANCoder from it's position reading back to ticks
    double currentTicks = this.encoder.getPosition() / this.encoder.configGetFeedbackCoefficient();
    double desiredTicks = currentTicks + deltaTicks;

    //below is a line to comment out from step 5
    this.steerMotor.set(TalonFXControlMode.Position, desiredTicks);

    double feetPerSecond = Units.metersToFeet(state.speedMetersPerSecond);

    //below is a line to comment out from step 5
    this.driveMotor.set(TalonFXControlMode.PercentOutput, feetPerSecond / SwerveDrive.kMaxSpeed);
  }

}
