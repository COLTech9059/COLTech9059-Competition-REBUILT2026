// Copyright (c) 2024-2026 Az-FIRST
// http://github.com/AZ-First
// Copyright (c) 2025-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// version 3 as published by the Free Software Foundation or
// available in the root directory of this project.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.
//
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import com.fasterxml.jackson.core.JsonProcessingException;
import com.fasterxml.jackson.databind.ObjectMapper;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Filesystem;
import java.io.IOException;
import java.nio.file.Path;
import lombok.Getter;

/**
 * Contains various field dimensions and useful reference points. All units are in meters and poses
 * have a blue alliance origin.
 */
public class FieldConstants {

  /** AprilTag Field Layout ************************************************ */
  public static final double aprilTagWidth = Inches.of(6.50).in(Meters);

  public static final String aprilTagFamily = "36h11";

  public static final double fieldLength = AprilTagLayoutType.OFFICIAL.getLayout().getFieldLength();
  public static final double fieldWidth = AprilTagLayoutType.OFFICIAL.getLayout().getFieldWidth();

  public static final int aprilTagCount = AprilTagLayoutType.OFFICIAL.getLayout().getTags().size();
  public static final AprilTagLayoutType defaultAprilTagType = AprilTagLayoutType.OFFICIAL;

  public static final AprilTagFieldLayout aprilTagLayout =
      AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField);

  @Getter
  public enum AprilTagLayoutType {
    OFFICIAL("2026-official"),
    NONE("2026-none"),
    REEFSCAPE("2025-official");

    AprilTagLayoutType(String name) {
      try {
        layout =
            new AprilTagFieldLayout(
                Constants.disableHAL
                    ? Path.of("src", "main", "deploy", "apriltags", name + ".json")
                    : Path.of(
                        Filesystem.getDeployDirectory().getPath(), "apriltags", name + ".json"));
      } catch (IOException e) {
        throw new RuntimeException(e);
      }

      try {
        layoutString = new ObjectMapper().writeValueAsString(layout);
      } catch (JsonProcessingException e) {
        throw new RuntimeException(
            "Failed to serialize AprilTag layout JSON " + toString() + "for PhotonVision");
      }
    }

    private final AprilTagFieldLayout layout;
    private final String layoutString;
  }

  /** Odometry Zones ************************************************ */
  public enum ZoneName {
    ALLIANCE,
    RAMP_RIGHT,
    RAMP_LEFT
  }

  public enum PointName {
    HUB
  }

  private static final Pose2d FIELD_SIZE = new Pose2d(fieldLength, fieldWidth, Rotation2d.kZero);

  private static final Pose2d[][] odometryZones = {
    {new Pose2d(0, 0, Rotation2d.kZero), new Pose2d(3.6, fieldWidth, Rotation2d.kZero)},
    {new Pose2d(3.3, 1.2, Rotation2d.kZero), new Pose2d(6.0, 3.9, Rotation2d.kZero)},
    {new Pose2d(3.3, 4.2, Rotation2d.kZero), new Pose2d(6.0, 7.2, Rotation2d.kZero)},
  };

  private static final Pose2d[] pointPoses = {new Pose2d(4.626, 4.035, Rotation2d.kZero)};

  private static Pose2d[] convertPoseToRedAllianceSide(Pose2d[] boxBounds) {
    return new Pose2d[] {
      new Pose2d(
          (FIELD_SIZE.getX() - boxBounds[1].getX()),
          (FIELD_SIZE.getY() - boxBounds[1].getY()),
          Rotation2d.kZero),
      new Pose2d(
          (FIELD_SIZE.getX() - boxBounds[0].getX()),
          (FIELD_SIZE.getY() - boxBounds[0].getY()),
          Rotation2d.kZero)
    };
  }

  private static Pose2d convertPoseToRedAllianceSide(Pose2d pointPose) {
    return new Pose2d(
        (FIELD_SIZE.getX() - pointPose.getX()),
        (FIELD_SIZE.getY() - pointPose.getY()),
        Rotation2d.kZero);
  }

  public static Pose2d[] getOdometryZone(ZoneName zone) {

    boolean isFlipped =
        (DriverStation.getAlliance().isPresent()
            && (DriverStation.getAlliance().get() == Alliance.Red));

    switch (zone) {
      case ALLIANCE:
        if (isFlipped) return convertPoseToRedAllianceSide(odometryZones[0]);
        return odometryZones[0];
      case RAMP_RIGHT:
        if (isFlipped) return convertPoseToRedAllianceSide(odometryZones[1]);
        return odometryZones[1];
      case RAMP_LEFT:
        if (isFlipped) return convertPoseToRedAllianceSide(odometryZones[2]);
        return odometryZones[2];
      default:
        return new Pose2d[] {Pose2d.kZero, Pose2d.kZero};
    }
  }

  public static Pose2d getPointPose(PointName point) {
    boolean isFlipped =
        (DriverStation.getAlliance().isPresent()
            && (DriverStation.getAlliance().get() == Alliance.Red));

    switch (point) {
      case HUB:
        if (isFlipped) return convertPoseToRedAllianceSide(pointPoses[0]);
        return pointPoses[0];
      default:
        return Pose2d.kZero;
    }
  }
}
