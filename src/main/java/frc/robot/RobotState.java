package frc.robot;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.interpolation.InterpolatingTreeMap;
import edu.wpi.first.math.interpolation.Interpolator;
import edu.wpi.first.math.interpolation.InverseInterpolator;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.util.AllianceFlipUtil;
import frc.robot.util.FieldConstants;
import java.util.function.Supplier;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.AutoLogOutputManager;
import org.littletonrobotics.junction.Logger;

public class RobotState {

  public static RobotState instance;

  public static RobotState getInstance() {
    if (instance == null) instance = new RobotState();
    return instance;
  }

  private Rotation2d lastRawGyro = new Rotation2d();

  private ChassisSpeeds fieldRelativeVelocity = new ChassisSpeeds();

  public final SwerveDrivePoseEstimator poseEstimator =
      new SwerveDrivePoseEstimator(
          new SwerveDriveKinematics(frc.robot.subsystems.drive.DriveConstants.MODULE_TRANSLATIONS),
          new Rotation2d(),
          new SwerveModulePosition[] {
            new SwerveModulePosition(),
            new SwerveModulePosition(),
            new SwerveModulePosition(),
            new SwerveModulePosition()
          },
          new Pose2d(),
          VecBuilder.fill(0.01, 0.01, 0.02),
          VecBuilder.fill(0.05, 0.05, 0.03));

  private SwerveModulePosition[] lastWheelPositions =
      new SwerveModulePosition[] {
        new SwerveModulePosition(),
        new SwerveModulePosition(),
        new SwerveModulePosition(),
        new SwerveModulePosition()
      };

  private final Field2d robotField = new Field2d();

  public static class Shot {
    public final double shooterRPM;
    public final double hoodPosition;

    public Shot(double shooterRPM, double hoodPosition) {
      this.shooterRPM = shooterRPM;
      this.hoodPosition = hoodPosition;
    }
  }

  private final InterpolatingTreeMap<Distance, Shot> distanceToShotMap =
      new InterpolatingTreeMap<>(
          (startValue, endValue, q) ->
              InverseInterpolator.forDouble()
                  .inverseInterpolate(startValue.in(Meters), endValue.in(Meters), q.in(Meters)),
          (startValue, endValue, t) ->
              new Shot(
                  Interpolator.forDouble()
                      .interpolate(startValue.shooterRPM, endValue.shooterRPM, t),
                  Interpolator.forDouble()
                      .interpolate(startValue.hoodPosition, endValue.hoodPosition, t)));

  private RobotState() {
    AutoLogOutputManager.addObject(this);

    distanceToShotMap.put(Meters.of(1.404), new Shot(3000, 0.1));
    distanceToShotMap.put(Meters.of(3.292), new Shot(3400, 0.4));
    distanceToShotMap.put(Meters.of(2.4853132009026604), new Shot(3200, 0.3));
    distanceToShotMap.put(Meters.of(4.890327530764509), new Shot(4000, 0.55));

    SmartDashboard.putData("RobotState/Field2D", robotField);
  }

  public void updateVelocityPeriodic(ChassisSpeeds speeds) {
    fieldRelativeVelocity = speeds;

    robotField.setRobotPose(getEstimatedPose());
  }

  public void resetPose(Pose2d pose) {
    resetPose(() -> pose);
  }

  public void resetPose(Supplier<Pose2d> pose) {
    poseEstimator.resetPosition(lastRawGyro, lastWheelPositions, pose.get());
  }

  public void addOdometryMeasurement(
      Rotation2d heading, SwerveModulePosition[] modulePositions, double timestamp) {
    lastRawGyro = heading;
    lastWheelPositions = modulePositions;
    poseEstimator.updateWithTime(timestamp, heading, modulePositions);
  }

  public void addVisionMeasurement(VisionObservation update) {
    poseEstimator.addVisionMeasurement(update.visionPose, update.timestamp, update.stdDevs);
  }

  @AutoLogOutput(key = "RobotState/EstimatedPose")
  public Pose2d getEstimatedPose() {
    return poseEstimator.getEstimatedPosition();
  }

  public Rotation2d getRotation() {
    return poseEstimator.getEstimatedPosition().getRotation();
  }

  /** Gets the needed hood angle at the robot's current distance */
  @AutoLogOutput(key = "RobotState/HoodPercent")
  public double getHoodAngle() {
    //    return shooterHoodDistanceMap.get(getDistanceToHubInches());
    return distanceToShotMap.get(Meters.of(getDistanceToHubMeters())).hoodPosition;
  }

  /** Gets the needed hood angle at the robot's current distance */
  public double getHoodAngle(Distance distance) {
    //    return shooterHoodDistanceMap.get(getDistanceToHubInches());
    return distanceToShotMap.get(distance).hoodPosition;
  }

  /** Gets the needed shooter RPM angle at the robot's current distance */
  @AutoLogOutput(key = "RobotState/ShooterRPM")
  public double getShooterRPM() {
    //    return shooterSpeedDistanceMap.get(getDistanceToHubMeters());
    return distanceToShotMap.get(Meters.of(getDistanceToHubMeters())).shooterRPM;
  }

  /** Gets the needed shooter RPM angle for a certain distance */
  public double getShooterRPM(Distance distance) {
    //    return shooterSpeedDistanceMap.get(getDistanceToHubMeters());
    return distanceToShotMap.get(distance).shooterRPM;
  }

  /** Gets the distance to the hub shooter */
  @AutoLogOutput(key = "RobotState/DistanceToGoalMeters")
  public double getDistanceToHubMeters() {
    return AllianceFlipUtil.apply(FieldConstants.Hub.goalPoint)
        .getDistance(getEstimatedPose().getTranslation());
  }

  //  public double getHoodToAngle(double hoodPercent) {
  //    return hoodPercentToAngle.get(hoodPercent);
  //  }

  /** Gets the angle from the robot to a certain point on the field. Used for auto aim */
  public Rotation2d getPointAtAngle(Supplier<Translation2d> point) {
    Translation2d robotToPoint = point.get().minus(getEstimatedPose().getTranslation());
    Rotation2d angleToPoint = new Rotation2d(robotToPoint.getX(), robotToPoint.getY());
    Logger.recordOutput("RobotState/AngleToPoint", angleToPoint);
    return angleToPoint;
  }

  /** Gets the angle from the robot to a certain point on the field. Used for auto aim */
  public Rotation2d getPointAtAngle(Supplier<Pose2d> robotPose, Supplier<Translation2d> point) {
    Translation2d robotToPoint = point.get().minus(robotPose.get().getTranslation());
    Rotation2d angleToPoint = new Rotation2d(robotToPoint.getX(), robotToPoint.getY());
    return angleToPoint;
  }

  public record VisionObservation(Pose2d visionPose, double timestamp, Matrix<N3, N1> stdDevs) {}
}
