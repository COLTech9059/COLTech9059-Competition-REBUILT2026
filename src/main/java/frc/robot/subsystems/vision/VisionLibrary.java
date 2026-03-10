package frc.robot.subsystems.vision;

import static frc.robot.FieldConstants.*;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.commands.AutopilotCommands;
import frc.robot.subsystems.drive.Drive;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

public class VisionLibrary {

  // These PID values yielded alignment times between ~1 & ~ 3.5 seconds in the simulation, though
  // they are wobbly when moving
  private static double POINT_KP = 0.8; // 0.856
  private static double POINT_KI = 0.5; // 0.7
  private static double POINT_KD = 0.07; // 0.328
  private static PIDController pointPID = new PIDController(POINT_KP, POINT_KI, POINT_KD);
  ;

  public static final class VisionHelpers {

    private static final double MIN_AUTO_TURN_SPEED = .25;
    private static final int IDEAL_ANGLE = 45;
    private static final int ANGLE_ERROR_BOUND = 3;

    // Alliance Checking
    public static boolean isAlliance(Alliance alliance) {
      return (DriverStation.getAlliance().isPresent()
          && DriverStation.getAlliance().get() == alliance);
    }

    // Strafing
    private static final double STRAFE_RADIAN_ERROR_BOUND = Math.PI / 180.0;
    private static boolean strafeEnabled = false;

    public static Pose2d translatePoseRelative() {
      return new Pose2d();
    }

    public static Pose3d getTargetPosition(int targetIndex) {
      AprilTagFieldLayout aprilTagField = AprilTagLayoutType.OFFICIAL.getLayout();

      int realIndex = isAlliance(Alliance.Blue) ? targetIndex += 16 : targetIndex;

      Pose3d targetPose = aprilTagField.getTagPose(realIndex).orElse(Pose3d.kZero);

      return targetPose;
    }

    public static Pose3d getTrueTargetPosition(int targetIndex) {
      AprilTagFieldLayout aprilTagField = AprilTagLayoutType.OFFICIAL.getLayout();
      Pose3d targetPose = aprilTagField.getTagPose(targetIndex).orElse(Pose3d.kZero);

      return targetPose;
    }

    public static void toggleStrafing() {
      strafeEnabled = !strafeEnabled;
    }

    /**
     * Overrides basic driving power to automatically turn the robot to the closest given degree
     * angle in a quadrant on the unit circle.
     *
     * @author Lunaradical
     * @param driveSubsystem The main Drive subsystem used to drive the robot.
     * @param stickPower Joystick turning power to return if the robot isn't in a valid zone.
     * @return Rotation power for turning the robot.
     */
    public static double getRotationPowerWithRampConsideration(
        Drive DriveSubsystem, double stickPower) {

      Logger.recordOutput(
          "VisionLibrary/inLeftBounds", (isRobotInZone(DriveSubsystem, ZoneName.RAMP_LEFT)));
      Logger.recordOutput(
          "VisionLibrary/inRightBounds", (isRobotInZone(DriveSubsystem, ZoneName.RAMP_RIGHT)));
      if (!(isRobotInZone(DriveSubsystem, ZoneName.RAMP_LEFT)
          || isRobotInZone(DriveSubsystem, ZoneName.RAMP_RIGHT))) return stickPower;

      Rotation2d robotRotation = DriveSubsystem.getHeading();

      int robotRotationInDegrees = (int) robotRotation.getDegrees();

      // Add negative rotation to simulate full circle with range [0,360].
      if (Math.signum(robotRotationInDegrees) == -1)
        robotRotationInDegrees = 360 + robotRotationInDegrees;

      // Determine closest direction to 45 degree angle.
      int degreeDifference = robotRotationInDegrees % (90);

      // Determine turning direction
      int sign = 1;
      if (degreeDifference >= IDEAL_ANGLE) sign = -1;

      Logger.recordOutput("VisionLibrary/DegreeDifference", degreeDifference);

      if ((degreeDifference < (IDEAL_ANGLE + ANGLE_ERROR_BOUND))
          && (degreeDifference > (IDEAL_ANGLE - ANGLE_ERROR_BOUND))) {
        Logger.recordOutput("VisionLibrary/TurnPower", 0.0);
        return 0.0;
      }

      Logger.recordOutput(
          "VisionLibrary/TurnPower",
          Math.max(MIN_AUTO_TURN_SPEED, (Math.abs(degreeDifference - IDEAL_ANGLE) / IDEAL_ANGLE))
              * sign);

      return Math.max(MIN_AUTO_TURN_SPEED, (Math.abs(degreeDifference - IDEAL_ANGLE) / IDEAL_ANGLE))
          * sign;
    }

    public static double getRotationPowerWithStrafeConsideration(
        Drive DriveSubsystem, double stickPower) {
      Logger.recordOutput(
          "VisionLibrary/Strafing/inAllianceZone",
          (isRobotInZone(DriveSubsystem, ZoneName.ALLIANCE)));
      if (!isRobotInZone(DriveSubsystem, ZoneName.ALLIANCE)) return stickPower;
      return getRotationPowerToTarget(DriveSubsystem, getPointPose(PointName.HUB));
    }

    public static double getRotationPower(Drive DriveSubsystem, double stickPower) {

      Logger.recordOutput("VisionLibrary/Strafing/strafeEnabled", (strafeEnabled));

      double turnPower =
          strafeEnabled
              ? getRotationPowerWithStrafeConsideration(DriveSubsystem, stickPower)
              : stickPower;

      // Ramps automatically override turn power input.
      turnPower = getRotationPowerWithRampConsideration(DriveSubsystem, turnPower);

      return turnPower;
    }

    // Distance Calculations
    public static double getDistanceFromPoint(Drive driveSubsystem, Pose2d pointPose) {
      Pose2d drivePose = driveSubsystem.getPose();
      
      double x = Math.abs(pointPose.getX() - drivePose.getX());
      double y = Math.abs(pointPose.getY() - drivePose.getY());

      return Math.sqrt((x*x) + (y*y));
    }

  }

  /**
   * Returns whether or not the robot exists in a given zone name.
   *
   * @author Lunaradical
   * @param driveSubsystem The main Drive subsystem used to drive the robot.
   * @param zone The ZoneName enum you want to check the robot exists in.
   * @return Whether or not the robot is in the box.
   */
  public static boolean isRobotInZone(Drive driveSubsystem, ZoneName zone) {
    // Get the bounds.
    Pose2d[] zoneBounds = getOdometryZone(zone);

    // Break if it's a point at the origin.
    if (zoneBounds.equals(new Pose2d[] {Pose2d.kZero, Pose2d.kZero})) return false;

    // Separate the points
    Pose2d pointLeft = zoneBounds[0];
    Pose2d pointRight = zoneBounds[1];

    // Robot Pose
    Pose2d robotPose = driveSubsystem.getPose();

    // Determine position
    boolean isLeftOfBounds = (Math.signum((robotPose.getX() - pointLeft.getX())) == -1.0);
    boolean isRightOfBounds = (Math.signum((robotPose.getX() - pointRight.getX())) == 1.0);
    boolean isAboveBounds = (Math.signum((robotPose.getY() - pointRight.getY())) == 1.0);
    boolean isBelowBounds = (Math.signum((robotPose.getY() - pointLeft.getY())) == -1.0);

    Logger.recordOutput(
        "VisionLibrary/notInBounds",
        (isLeftOfBounds || isRightOfBounds || isAboveBounds || isBelowBounds));

    Logger.recordOutput(
        "VisionLibrary/" + zone.name() + "/leftBoundDistance",
        (robotPose.getX() - pointLeft.getX()));
    Logger.recordOutput(
        "VisionLibrary/" + zone.name() + "/rightBoundDistance",
        (robotPose.getX() - pointRight.getX()));
    Logger.recordOutput(
        "VisionLibrary/" + zone.name() + "/topBoundDistance",
        (robotPose.getY() - pointLeft.getY()));
    Logger.recordOutput(
        "VisionLibrary/" + zone.name() + "/bottomBoundDistance",
        (robotPose.getY() - pointLeft.getY()));

    if (isLeftOfBounds || isRightOfBounds || isAboveBounds || isBelowBounds) return false;

    return true;
  }

  /**
   * Returns the rotation between the robot and the target.
   *
   * @author Lunaradical
   * @param driveSubsystem The main Drive subsystem used to drive the robot.
   * @param targetPose The pose you want the robot to look at.
   * @return The rotation to point at the target.
   */
  public static double getRotationPowerToTarget(Drive driveSubsystem, Pose2d targetPose) {
    // Return no rotation if target is invalid.
    if (targetPose.equals(Pose2d.kZero)) return 0.0;

    // Make the pose relative to the Robot (make the Robot the origin)
    Pose2d robotPose = driveSubsystem.getPose();

    // Use trigonometry to get the rotation to the point.
    double desiredAngle =
        Math.atan((targetPose.getY() - robotPose.getY()) / (targetPose.getX() - robotPose.getX()));

    double currentRobotRotation = driveSubsystem.getHeading().getRadians();

    // This math is true.
    if (VisionHelpers.isAlliance(Alliance.Red)) desiredAngle = (Math.PI + desiredAngle);
    if (desiredAngle > Math.PI) desiredAngle = (-Math.PI + (desiredAngle - Math.PI));

    // Angle from setpoint, however does not give power with respect to PI in the red alliance.
    double angleFromSetpoint = desiredAngle - currentRobotRotation;

    if (VisionHelpers.isAlliance(Alliance.Red)
        && Math.signum(currentRobotRotation) != Math.signum(desiredAngle))
      angleFromSetpoint = currentRobotRotation - desiredAngle;

    Logger.recordOutput("VisionLibrary/Strafing/AngleToTarget", desiredAngle);
    Logger.recordOutput("VisionLibrary/Strafing/AngleFromSetpoint", angleFromSetpoint);
    Logger.recordOutput(
        "VisionLibrary/Strafing/TurnPowerFromTarget",
        Math.min(Math.sqrt(Math.abs(angleFromSetpoint / (Math.PI / 4))), 1));

    if (Math.abs(angleFromSetpoint) < VisionHelpers.STRAFE_RADIAN_ERROR_BOUND) return 0.0;

    // Proportion angle to power
    double basePower = Math.min(Math.sqrt(Math.abs(angleFromSetpoint / (Math.PI / 4))), 1);
    double sign = 1;

    if (Math.signum(angleFromSetpoint) == -1.0) sign = -1;

    return Math.max(.125, basePower) * sign;
  }

  public static double getRotationPowerToTarget(Drive driveSubsystem, int targetIndex) {

    // Get pose of the target
    Pose2d targetPose = VisionHelpers.getTargetPosition(targetIndex).toPose2d();

    return getRotationPowerToTarget(driveSubsystem, targetPose);
  }

  /**
   * Returns the rotation value that maps robot orientation to point at the target
   *
   * @author DevAspen
   * @param driveSubsystem The main Drive subsystem used to drive the robot.
   * @param targetIndex The index of the AprilTag you want to focus on.
   * @return The rotation between the target and the robot's current position.
   */
  public static double pointToTargetWithPID(Drive driveSubsystem, int targetIndex) {

    // Get pose of the target
    Pose2d targetPose = VisionHelpers.getTargetPosition(targetIndex).toPose2d();

    // Return no rotation if target is invalid.
    if (targetPose.equals(Pose2d.kZero)) return Rotation2d.kZero.getRadians();

    // Make the pose relative to the Robot (make the Robot the origin)
    Pose2d RelativePose = targetPose.relativeTo(driveSubsystem.getPose());

    // Use trigonometry to get the rotation to the point.
    double Angle = Math.atan(RelativePose.getY() / RelativePose.getX());
    Rotation2d absoluteRotation = Rotation2d.fromRadians(Angle);

    // Get the rotation between the poses. (surely subtracting them will get me the rotation,
    // right?)
    // double currentRotation = driveSubsystem.getHeading().getRadians();
    // double angleDifference = (Angle - currentRotation);
    Rotation2d rotation = absoluteRotation.minus(driveSubsystem.getHeading());

    Logger.recordOutput("VisionLibrary/RotationToTarget", rotation.getRadians());
    Logger.recordOutput("VisionLibrary/PointKP", pointPID.getP());
    Logger.recordOutput("VisionLibrary/PointKI", pointPID.getI());
    Logger.recordOutput("VisionLibrary/PointKD", pointPID.getD());

    // TODO: monitor oscillation of function (does the simulation cause it to oscillate or does
    // there need to be rotation damping)?
    return pointPID.calculate(driveSubsystem.getHeading().getRadians(), Angle);
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
    // double currentRotation = driveSubsystem.getHeading().getRadians();
    // double angleDifference = (Angle - currentRotation);
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

  public static void setPointKP(double kP) {
    pointPID.setP(kP);
  }

  public static double getPointKP() {
    return pointPID.getP();
  }

  public static void setPointKI(double kI) {
    pointPID.setI(kI);
  }

  public static double getPointKI() {
    return pointPID.getI();
  }

  public static void setPointKD(double kD) {
    pointPID.setD(kD);
  }

  public static double getPointKD() {
    return pointPID.getD();
  }
}
