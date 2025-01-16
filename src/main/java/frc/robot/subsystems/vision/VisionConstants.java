package frc.robot.subsystems.vision;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.util.Units;

public class VisionConstants {
  // AprilTag layout
  public static AprilTagFieldLayout aprilTagLayout =
      AprilTagFieldLayout.loadField(AprilTagFields.k2025Reefscape);

  // Camera names, must match names configured on coprocessor
  public static String camera0Name = "FcamLeft";
  public static String camera1Name = "FcamRight";
  public static String camera2Name = "BcamLeft";
  public static String camera3Name = "BcamRight";

  // Robot to camera transforms
  // This is Camera centered IE instead of XYZ its YXZ for the Transform
  public static Transform3d robotToCamera0 =
      new Transform3d( // Front left
          Units.inchesToMeters(10.713),
          Units.inchesToMeters(11.466),
          Units.inchesToMeters(8.055),
          new Rotation3d(0.0, Units.degreesToRadians(-30), Units.degreesToRadians(-45)));
  public static Transform3d robotToCamera1 =
      new Transform3d( // Front Right
          Units.inchesToMeters(10.713),
          Units.inchesToMeters(-11.466),
          Units.inchesToMeters(8.055),
          new Rotation3d(0.0, Units.degreesToRadians(-30), Units.degreesToRadians(45)));
  public static Transform3d robotToCamera2 =
      new Transform3d( // Back left
          Units.inchesToMeters(-10.713),
          Units.inchesToMeters(11.466),
          Units.inchesToMeters(8.055),
          new Rotation3d(0.0, Units.degreesToRadians(-30), Units.degreesToRadians(180 - 45)));
  public static Transform3d robotToCamera3 =
      new Transform3d( // Back Right
          Units.inchesToMeters(-10.713),
          Units.inchesToMeters(-11.466),
          Units.inchesToMeters(8.055),
          new Rotation3d(0.0, Units.degreesToRadians(-30), Units.degreesToRadians(180 + 45)));
  // Basic filtering thresholds
  public static double maxAmbiguity = 0.3;
  public static double maxZError = 0.75;

  // Standard deviation baselines, for 1 meter distance and 1 tag
  // (Adjusted automatically based on distance and # of tags)
  public static double linearStdDevBaseline = 0.02; // Meters
  public static double angularStdDevBaseline = 0.06; // Radians

  // Standard deviation multipliers for each camera
  // (Adjust to trust some cameras more than others)
  public static double[] cameraStdDevFactors =
      new double[] {
        1.0, // Camera 0
        1.0 // Camera 1
      };

  // Multipliers to apply for MegaTag 2 observations
  public static double linearStdDevMegatag2Factor = 0.5; // More stable than full 3D solve
  public static double angularStdDevMegatag2Factor =
      Double.POSITIVE_INFINITY; // No rotation data available
}
