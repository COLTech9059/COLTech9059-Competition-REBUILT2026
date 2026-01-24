package frc.robot.subsystems.vision;

import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.subsystems.vision.VisionIO.VisionIOInputs;

class VisionHelpers {
  static int getBestCamera(Vision vision) {

    VisionIOInputs[] allCameras = vision.getAllCameras();
    int bestCamera = -1;

    for (int i = 0; i < allCameras.length; i++) {
      
    }

    return bestCamera;
  }
}

public class VisionLibrary {
  public static Rotation2d getRotationPowerToTarget(Vision vision) {

    // Get the yaw to target based on the best camera at the moment.
    Rotation2d yawToTarget = vision.getTargetX(VisionHelpers.getBestCamera(vision));



    return new Rotation2d();
  }
}
