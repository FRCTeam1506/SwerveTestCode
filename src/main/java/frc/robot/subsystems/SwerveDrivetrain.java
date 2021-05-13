// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.wpilibj.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.kinematics.SwerveDriveKinematics;
import edu.wpi.first.wpilibj.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.geometry.Rotation2d;

import com.kauailabs.navx.frc.AHRS;

import frc.robot.Constants.SwerveDrive;


public class SwerveDrivetrain extends SubsystemBase {

  private TalonFX frontRightMotor;
  private TalonFX frontLeftMotor;
  private TalonFX backRightMotor;
  private TalonFX backLeftMotor;

  private SwerveModuleMK3[] modules;

  private AHRS gyro;

  public SwerveDrivetrain() {
    this.frontLeftMotor     = new TalonFX(SwerveDrive.frontLeftMotorID);
    this.frontRightMotor    = new TalonFX(SwerveDrive.frontRightMotorID);
    this.backLeftMotor      = new TalonFX(SwerveDrive.backLeftMotorID);
    this.backRightMotor     = new TalonFX(SwerveDrive.backRightMotorID);

    // this.frontRightMotor.setInverted(true);
    this.frontLeftMotor.setInverted(true);
    this.backRightMotor.setInverted(true);

    SwerveModuleMK3 frontLeftModule   = new SwerveModuleMK3(this.frontLeftMotor, SwerveDrive.frontLeftTurnTalonID, false, SwerveDrive.frontLeftEncoderID, Rotation2d.fromDegrees(SwerveDrive.frontLeftEncoderOffset), "Front Left");
    SwerveModuleMK3 frontRightModule  = new SwerveModuleMK3(this.frontRightMotor, SwerveDrive.frontRightTurnTalonID, true, SwerveDrive.frontRightEncoderID, Rotation2d.fromDegrees(SwerveDrive.frontRightEncoderOffset), "Front Right");
    SwerveModuleMK3 backLeftModule    = new SwerveModuleMK3(this.backLeftMotor, SwerveDrive.backLeftTurnTalonID, true, SwerveDrive.backLeftEncoderID, Rotation2d.fromDegrees(SwerveDrive.backLeftEncoderOffset), "Back Left");
    SwerveModuleMK3 backRightModule   = new SwerveModuleMK3(this.backRightMotor, SwerveDrive.backRightTurnTalonID, true, SwerveDrive.backRightEncoderID, Rotation2d.fromDegrees(SwerveDrive.backRightEncoderOffset), "Back Right");
    
    this.modules = new SwerveModuleMK3[] {
      frontLeftModule,
      frontRightModule,
      backLeftModule,
      backRightModule
    };

    try {
      this.gyro = new AHRS(SPI.Port.kMXP);
      this.gyro.reset();
    } catch (RuntimeException ex) {
      System.out.println("#######################");
      System.out.println("NavX not plugged in !!!");
      System.out.println("#######################");
    }
  }

  /**
   * Method to drive the robot using joystick info.
   *
   * @param xSpeed Speed of the robot in the x direction (forward).
   * @param ySpeed Speed of the robot in the y direction (sideways).
   * @param rot Angular rate of the robot.
   * @param fieldRelative Whether the provided x and y speeds are relative to the field.
   * @param calibrateGyro button to recalibrate the gyro offset
   */
  public void drive(double xSpeed, double ySpeed, double rot, boolean fieldRelative, boolean calibrateGyro) {
    
    if(calibrateGyro) {
      gyro.reset(); //recalibrates gyro offset
    }

    SwerveModuleState[] states =
      SwerveDrive.normalizedKinematics.toSwerveModuleStates(
        fieldRelative
          ? ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, rot, Rotation2d.fromDegrees(-gyro.getAngle()))
          : new ChassisSpeeds(xSpeed, ySpeed, rot)
      );

    SwerveDriveKinematics.normalizeWheelSpeeds(states, SwerveDrive.kMaxSpeed);
    for (int i = 0; i < states.length; i++) {
      SwerveModuleMK3 module = modules[i];
      SwerveModuleState state = states[i];
      SmartDashboard.putNumber(String.valueOf(i), module.getRawAngle());
      //below is a line to comment out from step 5
      module.setDesiredState(state);
      SmartDashboard.putNumber("gyro angle", gyro.getAngle());
      SmartDashboard.putNumber("Encoder value " + String.valueOf(i), module.getRawAngle());
    }
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
