package frc.robot.util;

import static edu.wpi.first.units.Units.Inches;

import com.fasterxml.jackson.databind.ObjectMapper;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import java.io.IOException;
import org.littletonrobotics.junction.Logger;

public class FieldConstants {
  public static final AprilTagLayoutType defaultAprilTagType = AprilTagLayoutType.OFFICIAL;

  public static final double fieldLength = AprilTagLayoutType.OFFICIAL.getLayout().getFieldLength();
  public static final double fieldWidth = AprilTagLayoutType.OFFICIAL.getLayout().getFieldWidth();

  public static class Hub {

    // Dimensions
    public static final double width = Units.inchesToMeters(47.0);
    public static final double height =
        Units.inchesToMeters(72.0); // includes the catcher at the top
    public static final double innerWidth = Units.inchesToMeters(41.7);
    public static final double innerHeight = Units.inchesToMeters(56.5);

    // Relevant reference points on alliance side
    public static final Translation3d topCenterPoint =
        new Translation3d(
            AprilTagLayoutType.OFFICIAL.getLayout().getTagPose(26).get().getX() + width / 2.0,
            fieldWidth / 2.0,
            height);
    public static final Translation3d innerCenterPoint =
        new Translation3d(
            AprilTagLayoutType.OFFICIAL.getLayout().getTagPose(26).get().getX() + width / 2.0,
            fieldWidth / 2.0,
            innerHeight);

    public static final Translation2d goalPoint =
        new Translation2d(Inches.of(182.105 + 5.0), Inches.of(158.845));

    static {
      Logger.recordOutput("FieldConstants/Goal", goalPoint);
    }

    public static final Translation2d nearLeftCorner =
        new Translation2d(topCenterPoint.getX() - width / 2.0, fieldWidth / 2.0 + width / 2.0);
    public static final Translation2d nearRightCorner =
        new Translation2d(topCenterPoint.getX() - width / 2.0, fieldWidth / 2.0 - width / 2.0);
    public static final Translation2d farLeftCorner =
        new Translation2d(topCenterPoint.getX() + width / 2.0, fieldWidth / 2.0 + width / 2.0);
    public static final Translation2d farRightCorner =
        new Translation2d(topCenterPoint.getX() + width / 2.0, fieldWidth / 2.0 - width / 2.0);

    // Relevant reference points on the opposite side
    public static final Translation3d oppTopCenterPoint =
        new Translation3d(
            AprilTagLayoutType.OFFICIAL.getLayout().getTagPose(4).get().getX() + width / 2.0,
            fieldWidth / 2.0,
            height);
    public static final Translation2d oppNearLeftCorner =
        new Translation2d(oppTopCenterPoint.getX() - width / 2.0, fieldWidth / 2.0 + width / 2.0);
    public static final Translation2d oppNearRightCorner =
        new Translation2d(oppTopCenterPoint.getX() - width / 2.0, fieldWidth / 2.0 - width / 2.0);
    public static final Translation2d oppFarLeftCorner =
        new Translation2d(oppTopCenterPoint.getX() + width / 2.0, fieldWidth / 2.0 + width / 2.0);
    public static final Translation2d oppFarRightCorner =
        new Translation2d(oppTopCenterPoint.getX() + width / 2.0, fieldWidth / 2.0 - width / 2.0);

    // Hub faces
    public static final Pose2d nearFace =
        AprilTagLayoutType.OFFICIAL.getLayout().getTagPose(26).get().toPose2d();
    public static final Pose2d farFace =
        AprilTagLayoutType.OFFICIAL.getLayout().getTagPose(20).get().toPose2d();
    public static final Pose2d rightFace =
        AprilTagLayoutType.OFFICIAL.getLayout().getTagPose(18).get().toPose2d();
    public static final Pose2d leftFace =
        AprilTagLayoutType.OFFICIAL.getLayout().getTagPose(21).get().toPose2d();
  }

  public enum AprilTagLayoutType {
    OFFICIAL("2026-official"),
    NONE("2026-none"),
    HUB("2026-hub"),
    OUTPOST("2026-outpost"),
    TOWER("2026-tower");

    private final String name;
    private volatile AprilTagFieldLayout layout;
    private volatile String layoutString;

    AprilTagLayoutType(String name) {
      this.name = name;
    }

    public AprilTagFieldLayout getLayout() {
      if (layout == null) {
        synchronized (this) {
          if (layout == null) {
            try {
              layout = AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField);
              layoutString = new ObjectMapper().writeValueAsString(layout);
            } catch (IOException e) {
              throw new RuntimeException(e);
            }
          }
        }
      }
      return layout;
    }

    public String getLayoutString() {
      if (layoutString == null) {
        getLayout();
      }
      return layoutString;
    }
  }
}
