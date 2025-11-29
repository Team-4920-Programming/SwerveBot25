/*
 * MIT License
 *
 * Copyright (c) PhotonVision
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

 package frc.robot.subsystems.swervedrive;

 import static frc.robot.Constants.Vision4920.*;
//import static frc.robot.Constants.Vision4920.kTagLayout;

import edu.wpi.first.math.Matrix;
 import edu.wpi.first.math.VecBuilder;
 import edu.wpi.first.math.geometry.Pose2d;
 import edu.wpi.first.math.geometry.Transform3d;
 import edu.wpi.first.math.numbers.N1;
 import edu.wpi.first.math.numbers.N3;
import frc.robot.Constants;

import java.util.List;
import java.util.Optional;
 import org.photonvision.EstimatedRobotPose;
 import org.photonvision.PhotonCamera;
 import org.photonvision.PhotonPoseEstimator;
 import org.photonvision.PhotonPoseEstimator.PoseStrategy;
 import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;
 
 public class Vision4920 {
     private final PhotonCamera camera;
     private final PhotonPoseEstimator photonEstimator;
     private Matrix<N3, N1> curStdDevs;
     private double lastEstTimestamp = 0;
 
  
    
     public Vision4920(String CameraName, Transform3d RobotToCam) {
         camera = new PhotonCamera(CameraName);
         photonEstimator =
                new PhotonPoseEstimator(kTagLayout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, RobotToCam);


         photonEstimator.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);
     }
 
     public PhotonPipelineResult getLatestResult() {
         return camera.getLatestResult();
     }
 
     public void changePipeline(int NewPipeline)
     {
         camera.setPipelineIndex(NewPipeline);
         
     }
 
     public Optional<EstimatedRobotPose> getEstimatedGlobalPose() {
        Optional<EstimatedRobotPose> visionEst = Optional.empty();
        for (var change : camera.getAllUnreadResults()) {
            visionEst = photonEstimator.update(change);
            updateEstimationStdDevs(visionEst, change.getTargets());
        }
        return visionEst;
    }
 /**
     * Calculates new standard deviations This algorithm is a heuristic that creates dynamic standard
     * deviations based on number of tags, estimation strategy, and distance from the tags.
     *
     * @param estimatedPose The estimated pose to guess standard deviations for.
     * @param targets All targets in this camera frame
     */
    private void updateEstimationStdDevs(
            Optional<EstimatedRobotPose> estimatedPose, List<PhotonTrackedTarget> targets) {
        if (estimatedPose.isEmpty()) {
            // No pose input. Default to single-tag std devs
            curStdDevs = kSingleTagStdDevs;

        } else {
            // Pose present. Start running Heuristic
            var estStdDevs = kSingleTagStdDevs;
            int numTags = 0;
            double avgDist = 0;

            // Precalculation - see how many tags we found, and calculate an average-distance metric
            for (var tgt : targets) {
                var tagPose = photonEstimator.getFieldTags().getTagPose(tgt.getFiducialId());
                if (tagPose.isEmpty()) continue;
                numTags++;
                avgDist +=
                        tagPose
                                .get()
                                .toPose2d()
                                .getTranslation()
                                .getDistance(estimatedPose.get().estimatedPose.toPose2d().getTranslation());
            }

            if (numTags == 0) {
                // No tags visible. Default to single-tag std devs
                curStdDevs = kSingleTagStdDevs;
            } else {
                // One or more tags visible, run the full heuristic.
                avgDist /= numTags;
                // Decrease std devs if multiple targets are visible
                if (numTags > 1) estStdDevs = kMultiTagStdDevs;
                // Increase std devs based on (average) distance
                if (numTags == 1 && avgDist > 4)
                    estStdDevs = VecBuilder.fill(Double.MAX_VALUE, Double.MAX_VALUE, Double.MAX_VALUE);
                else estStdDevs = estStdDevs.times(1 + (avgDist * avgDist / 30));
                curStdDevs = estStdDevs;
            }
        }
    }
     /**
      * The standard deviations of the estimated pose from {@link #getEstimatedGlobalPose()}, for use
      * with {@link edu.wpi.first.math.estimator.SwerveDrivePoseEstimator SwerveDrivePoseEstimator}.
      * This should only be used when there are targets visible.
      *
      * @param estimatedPose The estimated pose to guess standard deviations for.
      */
     public Matrix<N3, N1> getEstimationStdDevs(Pose2d estimatedPose) {
         var estStdDevs = kSingleTagStdDevs ;
         var targets = getLatestResult().getTargets();
         int numTags = 0;
         double avgDist = 0;
         for (var tgt : targets) {
             var tagPose = photonEstimator.getFieldTags().getTagPose(tgt.getFiducialId());
             if (tagPose.isEmpty()) continue;
             numTags++;
             avgDist +=
                     tagPose.get().toPose2d().getTranslation().getDistance(estimatedPose.getTranslation());
         }
         if (numTags == 0) return estStdDevs;
         avgDist /= numTags;
         // Decrease std devs if multiple targets are visible
         if (numTags > 1) estStdDevs = kMultiTagStdDevs;
         // Increase std devs based on (average) distance
         if (numTags == 1 && avgDist > 4)
             estStdDevs = VecBuilder.fill(Double.MAX_VALUE, Double.MAX_VALUE, Double.MAX_VALUE);
         else estStdDevs = estStdDevs.times(1 + (avgDist * avgDist / 30));
 
         return estStdDevs;
     }
     public Matrix<N3, N1> confidenceCalculator(EstimatedRobotPose estimation) {
         double smallestDistance = Double.POSITIVE_INFINITY;
         for (var target : estimation.targetsUsed) {
           var t3d = target.getBestCameraToTarget();
           var distance = Math.sqrt(Math.pow(t3d.getX(), 2) + Math.pow(t3d.getY(), 2) + Math.pow(t3d.getZ(), 2));
           if (distance < smallestDistance)
             smallestDistance = distance;
         }
         
         double poseAmbiguityFactor = estimation.targetsUsed.size() != 1
             ? 1
             : Math.max(
                 1,
                 (estimation.targetsUsed.get(0).getPoseAmbiguity()
                     + Constants.Vision4920.POSE_AMBIGUITY_SHIFTER)
                     * Constants.Vision4920.POSE_AMBIGUITY_MULTIPLIER);
         double confidenceMultiplier = Math.max(
             1,
             (Math.max(
                 1,
                 Math.max(0, smallestDistance - Constants.Vision4920.NOISY_DISTANCE_METERS)
                     * Constants.Vision4920.DISTANCE_WEIGHT)
                 * poseAmbiguityFactor)
                 / (1
                     + ((estimation.targetsUsed.size() - 1) * Constants.Vision4920.TAG_PRESENCE_WEIGHT)));
     
         return Constants.Vision4920.VISION_MEASUREMENT_STANDARD_DEVIATIONS.times(confidenceMultiplier);
       }
 
       public boolean isConnected(){
         return camera.isConnected();
       }
     
  
 }
 