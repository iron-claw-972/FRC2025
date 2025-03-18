// Copyright 2021-2025 FRC 6328
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

package frc.robot.util.Vision;

import java.util.ArrayList;
import java.util.List;

import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.inputs.LoggableInputs;
import org.photonvision.targeting.PhotonPipelineResult;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;

public interface VisionIO {
  public static class VisionIOInputs implements LoggableInputs {
    public boolean connected = false;
    public List<PhotonPipelineResult> results = new ArrayList<>();
    
    // PhotonVision should never return more than 5 results, except possibly for very long loop overruns
    private static final int maxLength = 5;

    private boolean intitalized = false;

    @Override
    public void toLog(LogTable table) {
      // LogTable does not easily allow removal of logs, especially ProtobufSerializables, so extra values will need to be ignored
      // This is not very efficient, since unused values are still taking up memory, but there is no easy way to remove them
      if (!intitalized){
        for(int i = 0; i < maxLength; i++){
          table.put("Results"+i, new PhotonPipelineResult());
          intitalized = true;
        }
      }
      table.put("Connected", connected);
      double length = Math.min(results.size(), maxLength);
      table.put("Length", length);
      for(int i = 0; i < length; i++){
        table.put("Results"+i, results.get(i));
      }
    }

    @Override
    public void fromLog(LogTable table) {
      connected = table.get("Connected", false);
      int length = table.get("Length", 0);
      results = new ArrayList<>(length);
      // Java gets confused when null is used for a generic type argument
      PhotonPipelineResult nullResult = null;
      for(int i = 0; i < length; i++){
        PhotonPipelineResult result = table.get("Results"+i, nullResult);
        if(result != null){
          results.add(result);
        }
      }
    }
  }

  /** Represents the angle to a simple target, not used for pose estimation. */
  public static record TargetObservation(Rotation2d tx, Rotation2d ty) {}

  /** Represents a robot pose sample used for pose estimation. */
  public static record PoseObservation(
      double timestamp,
      Pose3d pose,
      double ambiguity,
      int tagCount,
      double averageTagDistance) {}

  public default void updateInputs() {}
}