package frc.robot.subsystems.Elevator;

import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.subsystems.drive.ModuleIO.ModuleIOInputs;

import org.littletonrobotics.junction.AutoLog;

public interface ElMotorIO {

  @AutoLog
  public static class ElMotorIOInputs {
    public boolean masterConnected = false;
    public double masterPositionRad = 0.0;
    public double masterVelocityRadPerSec = 0.0;
    public double masterAppliedVolts = 0.0;
    public double masterCurrentAmps = 0.0;

    public boolean followerConnected = false;
    public double followerPositionRad = 0.0;
    public double followerVelocityRadPerSec = 0.0;
    public double followerAppliedVolts = 0.0;
    public double followerCurrentAmps = 0.0;

    public Rotation2d extentionAbsPos = new Rotation2d();
    public Rotation2d extentionPos = new Rotation2d();
  }
    /** Updates the set of loggable inputs. */
  public default void updateInputs(ModuleIOInputs inputs) {}

  /** Run the drive motor at the specified open loop value. */
  public default void setElevatorOpenLoop(double output) {}

  /** Run the drive motor at the specified velocity. */
  public default void setElevatorVelocity(double velocityRadPerSec) {}

  /** Run the turn motor to the specified rotation. */
  public default void setElevatorPosition(Rotation2d rotation) {}
}
