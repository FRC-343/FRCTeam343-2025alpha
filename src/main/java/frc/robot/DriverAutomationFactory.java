package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.commands.PositionWithCoralStation;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.vision.OffsetTags;
import frc.robot.util.CommandCustomController;
import java.util.Set;

public class DriverAutomationFactory {
  private final CommandCustomController driverController;
  // private final CommandCustomController operatorController;

  private final Drive drive;

  public DriverAutomationFactory(
      CommandCustomController driverController,
      CommandCustomController operatorController,
      Drive drive) {
    this.driverController = driverController;
    // this.operatorController = operatorController;
    this.drive = drive;
  }

  public Command LeftCoralPath() {
    return OffsetTags.CORAL_ONE.getDeferredCommand();
  }

  public Command LeftCoralAssist() {
    return Commands.defer(
        () ->
            new PositionWithCoralStation(
                () -> -driverController.getLeftX(), drive, OffsetTags.CORAL_ONE),
        Set.of(drive));
  }
}
