package frc.robot.subsystems.vision;

import java.util.List;
import java.util.Set;
import java.util.stream.Collectors;
import java.util.stream.Stream;

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


/*
 * 
 * 
 *  APRIL TAG ID ASSINGMENT 
 *
 * 
 */
      

 // Coral Sation IDs
      // RED
        public static final Integer RED_CORAL_ONE = 1;
        public static final Integer RED_CORAL_TWO = 2;

        public static final Set<Integer> RED_CORAL_TAGS =
        Set.of(RED_CORAL_ONE, RED_CORAL_TWO);
      // Blue
        public static final Integer BLUE_CORAL_ONE = 13;
        public static final Integer BLUE_CORAL_TWO = 12;

        public static final Set<Integer> BLUE_CORAL_TAGS =
        Set.of(BLUE_CORAL_ONE, BLUE_CORAL_TWO);

 // REEF IDs These go in a clock wise rotation when looking at it from the respective drive station
      // RED
        public static final Integer RED_REEF_ONE = 10;
        public static final Integer RED_REEF_TWO = 9;
        public static final Integer RED_REEF_THREE = 8;
        public static final Integer RED_REEF_FOUR = 7;
        public static final Integer RED_REEF_FIVE = 6;
        public static final Integer RED_REEF_SIX = 11;

        public static final Set<Integer> RED_REEF_TAGS =
        Set.of(RED_REEF_ONE, RED_REEF_TWO, RED_REEF_THREE, RED_REEF_FOUR, RED_REEF_FIVE, RED_REEF_SIX);
     // BLUE
        public static final Integer BLUE_REEF_ONE = 21;
        public static final Integer BLUE_REEF_TWO = 22;
        public static final Integer BLUE_REEF_THREE = 17;
        public static final Integer BLUE_REEF_FOUR = 18;
        public static final Integer BLUE_REEF_FIVE = 19;
        public static final Integer BLUE_REEF_SIX = 20;

        public static final Set<Integer> BLUE_REEF_TAGS =
        Set.of(BLUE_REEF_ONE, BLUE_REEF_TWO, BLUE_REEF_THREE, BLUE_REEF_FOUR, BLUE_REEF_FIVE, BLUE_REEF_SIX);
 // Processor IDs
      // RED
        public static final Integer RED_PROCESSOR = 3;
      // Blue
        public static final Integer BLUE_PROCESSOR = 16;
 // Barge IDs
      // RED
        public static final Integer RED_BARGE_ONE = 5;
        public static final Integer RED_BARGE_TWO = 15;

        public static final Set<Integer> RED_BARGE_TAGS =
        Set.of(RED_BARGE_ONE, RED_BARGE_TWO);
      // BLUE
        public static final Integer BLUE_BARGE_ONE = 14;
        public static final Integer BLUE_BARGE_TWO = 4;

        public static final Set<Integer> BLUE_BARGE_TAGS =
        Set.of(BLUE_BARGE_ONE, BLUE_BARGE_TWO);

// Field IDs

      // RED TAG FIDS
        public static final Set<Integer> RED_TAG_FIDS = 
        Stream.of(RED_BARGE_TAGS, RED_CORAL_TAGS, RED_REEF_TAGS, Set.of(RED_PROCESSOR))
        .flatMap(Set::stream)
        .collect(Collectors.toSet());
      // BLUE TAG FIDS
        public static final Set<Integer> BLUE_TAG_FIDS = 
        Stream.of(BLUE_BARGE_TAGS, BLUE_CORAL_TAGS, BLUE_REEF_TAGS, Set.of(BLUE_PROCESSOR))
        .flatMap(Set::stream)
        .collect(Collectors.toSet());

      // All Tags
        public static final Set<Integer> ALL_TAGS =
        Stream.concat(RED_TAG_FIDS.stream(),BLUE_TAG_FIDS.stream()).collect(Collectors.toSet());

 // List of all possible combinations of april tags
      public static final List<Set<Integer>> POSSIBLE_FRAME_FID_COMBOS =
      List.of(RED_TAG_FIDS, BLUE_TAG_FIDS);
}
