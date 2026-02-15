package frc.robot.subsystems.vision;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.FieldConstants;
import frc.robot.commands.AutopilotCommands;
import frc.robot.subsystems.drive.Drive;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

public class VisionLibrary {

  public static final class VisionHelpers {

    public static Pose2d translatePoseRelative() {
      return new Pose2d();
    }

    public static Pose3d getTargetPosition(int targetIndex) {
      AprilTagFieldLayout aprilTagField = FieldConstants.AprilTagLayoutType.OFFICIAL.getLayout();
      Pose3d targetPose = aprilTagField.getTagPose(targetIndex).orElse(Pose3d.kZero);

      return targetPose;
    }
  }

  /**
   * Returns the rotation between the robot and the target.
   *
   * @author Lunaradical
   * @param driveSubsystem The main Drive subsystem used to drive the robot.
   * @param targetIndex The index of the AprilTag you want to focus on.
   * @return The rotation between the target and the robot's current position.
   */
  public static Rotation2d getRotationToTarget(Drive driveSubsystem, int targetIndex) {

    // Get pose of the target
    Pose2d targetPose = VisionHelpers.getTargetPosition(targetIndex).toPose2d();

    // Return no rotation if target is invalid.
    if (targetPose.equals(Pose2d.kZero)) return Rotation2d.kZero;

    // Get the rotation between the poses. (surely subtracting them will get me the rotation,
    // right?)
    Rotation2d rotation = driveSubsystem.getHeading().minus(targetPose.getRotation());

    Logger.recordOutput("VisionLibrary/RotationToTarget", rotation.getRadians());

    // TODO: figure out why drive doesn't accept the rotation value despite returning a real value.
    return rotation;
  }

  /**
   * Returns the rotation value that maps robot orientation to point at the target
   *
   * @author Lunaradical
   * @param driveSubsystem The main Drive subsystem used to drive the robot.
   * @param targetIndex The index of the AprilTag you want to focus on.
   * @return The rotation between the target and the robot's current position.
   */
  public static Rotation2d pointToTarget(Drive driveSubsystem, int targetIndex) {

    // Get pose of the target
    Pose2d targetPose = VisionHelpers.getTargetPosition(targetIndex).toPose2d();

    // Return no rotation if target is invalid.
    if (targetPose.equals(Pose2d.kZero)) return Rotation2d.kZero;

    // Make the pose relative to the Robot (make the Robot the origin)
    Pose2d RelativePose = targetPose.relativeTo(driveSubsystem.getPose());

    // Use trigonometry to get the rotation to the point.
    double Angle = Math.atan(RelativePose.getY() / RelativePose.getX());
    Rotation2d absoluteRotation = Rotation2d.fromRadians(Angle);

    // Get the rotation between the poses. (surely subtracting them will get me the rotation,
    // right?)
    Rotation2d rotation = absoluteRotation.minus(driveSubsystem.getHeading());

    Logger.recordOutput("VisionLibrary/RotationToTarget", rotation.getRadians());

    // TODO: monitor oscillation of function (does the simulation cause it to oscillate or does
    // there need to be rotation damping)?
    return rotation;
  }

  // luna wuz here
  /**
   * Manipulate the drive subsystem using odometry and vision to position yourself perpendicular to
   * a target and face it. Beelines to the target, <b>so use when the passage is clear</b>.
   *
   * @author Lunaradical
   * @param driveSubsystem The main Drive subsystem used to... well, drive the robot.
   * @param targetIndex The index of the AprilTag you want to focus on.
   * @param distanceFromTargetMeters The perpendicular distance in front of the AprilTag in meters.
   * @return The Autopilot Command used to drive the robot to position, or nothing if tag is
   *     invalid.
   */
  public static Supplier<Command> moveToTargetParallel(
      Drive driveSubsystem, int targetIndex, double distanceFromTargetMeters) {
    return () -> {
      // Get the target's pose.
      Pose3d targetPose = VisionHelpers.getTargetPosition(targetIndex);

      // Don't do anything if we don't have a pose (an empty pose is an invalid pose).
      if (targetPose.equals(Pose3d.kZero)) return Commands.none();

      // Map to a 2D plane and transform it by the given distance.
      Pose2d targetPoseFlat = targetPose.toPose2d();
      Pose2d newPose =
          targetPoseFlat.transformBy(
              new Transform2d(distanceFromTargetMeters, 0, Rotation2d.kZero));

      //
      return AutopilotCommands.runAutopilot(driveSubsystem, newPose);
    };
  }
}
