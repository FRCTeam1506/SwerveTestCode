// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.Joystick;

import frc.robot.Constants.Playstation;
import frc.robot.commands.SwerveDriveCommand;
import frc.robot.subsystems.SwerveDrivetrain;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {

  // * Joysticks
  private static Joystick driver = new Joystick(0);

  // * Subsystems
  private final SwerveDrivetrain drivetrain = new SwerveDrivetrain();

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    configureButtonBindings();
    setDefaultCommands();
  }

  private void configureButtonBindings() {}

  private void setDefaultCommands() {
    drivetrain.setDefaultCommand(
      new SwerveDriveCommand(
        drivetrain,
        () -> driver.getRawAxis(Playstation.LeftYAxis),
        () -> driver.getRawAxis(Playstation.LeftXAxis),
        () -> driver.getRawAxis(Playstation.RightXAxis),
        () -> driver.getRawButton(Playstation.LeftBumper)
      )
    );
  }
}
