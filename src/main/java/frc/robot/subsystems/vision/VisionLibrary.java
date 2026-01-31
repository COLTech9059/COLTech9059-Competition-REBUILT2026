package frc.robot.subsystems.vision;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.vision.VisionIO.VisionIOInputs;
import frc.robot.FieldConstants;
import frc.robot.commands.AutopilotCommands;
import java.util.function.Supplier;
import com.therekrab.autopilot.*;

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

  public static int getBestCamera(Vision vision) {

    VisionIOInputs[] allCameras = vision.getAllCameras();
    int bestCamera = -1;

    for (int i = 0; i < allCameras.length; i++) {}

    return bestCamera;
  }

  public static Rotation2d getRotationPowerToTarget(Vision vision) {

    // Get the yaw to target based on the best camera at the moment.
    Rotation2d yawToTarget = vision.getTargetX(VisionHelpers.getBestCamera(vision));

    return new Rotation2d();
  }

  // luna wuz here
  public static Supplier<Command> moveToTargetParallel(Drive driveSubsystem, int targetIndex, double distanceFromTarget) {
    return () -> {
      AprilTagFieldLayout aprilTagField = FieldConstants.AprilTagLayoutType.OFFICIAL.getLayout();
      Pose3d targetPose = aprilTagField.getTagPose(targetIndex).orElse(new Pose3d());

      // Don't do anything if we don't have a pose (an empty pose is an invalid pose).
      if (targetPose.equals(new Pose3d())) return Commands.none();

      // targetPose.

      // TODO: calculate new Pose2d and replace the null value
      return AutopilotCommands.runAutopilot(driveSubsystem, null);
    };
  }

}
