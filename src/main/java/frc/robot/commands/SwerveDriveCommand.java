package frc.robot.commands;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj.SlewRateLimiter;
import edu.wpi.first.wpilibj2.command.CommandBase;

import frc.robot.Constants.SwerveDrive;
import frc.robot.subsystems.SwerveDrivetrain;

public class SwerveDriveCommand extends CommandBase {

  private final SwerveDrivetrain drivetrain;
  private DoubleSupplier leftY;
  private DoubleSupplier leftX;
  private DoubleSupplier rightX;
  private BooleanSupplier leftBumper;

  // Slew rate limiters to make joystick inputs more gentle; 1/3 sec from 0 to 1.
  private final SlewRateLimiter xspeedLimiter = new SlewRateLimiter(0.5);
  private final SlewRateLimiter yspeedLimiter = new SlewRateLimiter(0.5);
  private final SlewRateLimiter rotLimiter = new SlewRateLimiter(0.5);

  public SwerveDriveCommand(SwerveDrivetrain drivetrain, DoubleSupplier leftY, DoubleSupplier leftX, DoubleSupplier rightX, BooleanSupplier leftBumper) {
    this.drivetrain = drivetrain;
    addRequirements(drivetrain);

    this.leftY = leftY;
    this.leftX = leftX;
    this.rightX = rightX;
    this.leftBumper = leftBumper;
  }

  @Override
  public void execute() {

    // Get the x speed. We are inverting this because Xbox joysticks return
    // negative values when we push forward.
    final double xSpeed;
    if (Math.abs(this.leftY.getAsDouble()) > 0.1) {
      xSpeed = -xspeedLimiter.calculate(this.leftY.getAsDouble()) * SwerveDrive.kMaxSpeed;
    } else {
      xSpeed = 0.0;
    }

    // Get the y speed or sideways/strafe speed. We are inverting this because
    // we want a positive value when we pull to the left. Xbox joysticks
    // return positive values when you pull to the right by default.
    final double ySpeed;
    if (Math.abs(this.leftX.getAsDouble()) > 0.1) {
      ySpeed = -yspeedLimiter.calculate(this.leftX.getAsDouble()) * SwerveDrive.kMaxSpeed;
    } else {
      ySpeed = 0.0;
    }

    // Get the rate of angular rotation. We are inverting this because we want a
    // positive value when we pull to the left (remember, CCW is positive in
    // mathematics). Xbox joysticks return positive values when you pull to
    // the right by default.
    final double rot;
    if (Math.abs(this.rightX.getAsDouble()) > 0.1) {
      rot = -rotLimiter.calculate(this.rightX.getAsDouble()) * SwerveDrive.kMaxAngularSpeed;
    } else {
      rot = 0.0;
    }

    boolean calibrate = leftBumper.getAsBoolean();

    drivetrain.drive(xSpeed, ySpeed, rot, false, calibrate);
  }

}
