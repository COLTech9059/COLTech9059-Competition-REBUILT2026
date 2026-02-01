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
import frc.robot.subsystems.vision.VisionIO.VisionIOInputs;
import java.util.function.Supplier;

class VisionHelpers {
  static int getBestCamera(Vision vision) {

    VisionIOInputs[] allCameras = vision.getAllCameras();
    int bestCamera = -1;

    for (int i = 0; i < allCameras.length; i++) {}

    return bestCamera;
  }

  static Pose2d translatePoseRelative() {
    return new Pose2d();
  }
}

public class VisionLibrary {
  public static Rotation2d getRotationPowerToTarget(Vision vision) {

    // Get the yaw to target based on the best camera at the moment.
    Rotation2d yawToTarget = vision.getTargetX(VisionHelpers.getBestCamera(vision));

    return new Rotation2d();
  }

  // luna wuz here
  public static Supplier<Command> moveToTargetParallel(
      Drive driveSubsystem, int targetIndex, double distanceFromTarget) {
    return () -> {
      AprilTagFieldLayout aprilTagField = FieldConstants.AprilTagLayoutType.OFFICIAL.getLayout();
      Pose3d targetPose = aprilTagField.getTagPose(targetIndex).orElse(Pose3d.kZero);

      // Don't do anything if we don't have a pose (an empty pose is an invalid pose).
      if (targetPose.equals(Pose3d.kZero)) return Commands.none();

      Pose2d targetPoseFlat = targetPose.toPose2d();
      Pose2d poseDistanceRotated =
          new Pose2d(Meters.of(-distanceFromTarget).magnitude(), 0, new Rotation2d())
              .rotateBy(targetPoseFlat.getRotation());

      Pose2d newPose =
          targetPoseFlat.transformBy(
              new Transform2d(
                  poseDistanceRotated.getX(), poseDistanceRotated.getY(), Rotation2d.kZero));

      // TODO: map new pose relative to target (it's weirdly based on world?)
      // i can't test this without some sort of simulation or real-world test
      // judging from what I understand by this, this should work.
      return AutopilotCommands.runAutopilot(driveSubsystem, newPose);
    };
  }
}
