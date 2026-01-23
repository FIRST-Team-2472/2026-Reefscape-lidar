package frc.robot.extras;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.LidarConstants;
import frc.robot.extras.LidarMapComponents.LidarSimulator;
import frc.robot.extras.LidarMapComponents.MapPoint;
import frc.robot.extras.LidarMapComponents.Polygon;

//this class is designed to take in a point cloud and a rough pose estimate and return a refined pose estimate
//it does so by comparing the measured point cloud to simulated point clouds at various nearby positions and selecting the best match
//it does not however improve rotational accuracy yet

public class PointCloudPositionEstimator {
    public static FieldPoint estimatePose(FieldPose2d roughPose, double[] measuredPointCloud) {
        // Default to global map for backwards compatibility
        return estimatePose(roughPose, measuredPointCloud, frc.robot.extras.LidarMap.getMap());
    }

    public static FieldPoint estimatePose(FieldPose2d roughPose, double[] measuredPointCloud, Polygon[] map) {
        if (areAllZeros(measuredPointCloud)) {
            return new FieldPoint(roughPose.getX(), roughPose.getY());
        }
        MapPoint robotPose = new MapPoint(roughPose.getX(), roughPose.getY());
        MapPoint[] roughParticles = generateParticles(robotPose, 60, 0.05, 0.006);
        double[] particleScores = scoreParticles(roughParticles, measuredPointCloud,
                roughPose.getRotation().getDegrees(), map);
        MapPoint bestParticle = getBestParticle(roughParticles, particleScores);

        // we have the 1st estimate now we need to refine this
        MapPoint[] moderateParticles = generateParticles(bestParticle, 20, 0.02, 0.002);
        double[] moderateParticleScores = scoreParticles(moderateParticles, measuredPointCloud,
                roughPose.getRotation().getDegrees(), map);
        MapPoint bestModerateParticle = getBestParticle(moderateParticles, moderateParticleScores);

        // we found the best probable location using particles, now we estimate confidence by seeing how the map looks from that point
        // and if it has a lot of variance in x or y
        MapPoint[] idealPointsSeen = LidarSimulator.getPerfectPointCloud(bestModerateParticle, roughPose.getRotation().getDegrees(), map);

        double xConfidence = getXConfidence(idealPointsSeen);
        double yConfidence = getYConfidence(idealPointsSeen);

        double finalX = interpolate(roughPose.getX(), 1.0/4.0, bestModerateParticle.x, 1.0/(xConfidence+0.5));
        double finalY = interpolate(roughPose.getY(), 1.0/4.0, bestModerateParticle.y, 1.0/(yConfidence+0.5));

        System.out.println("X Confidence: " + xConfidence + " Y Confidence: " + yConfidence);

        return new FieldPoint(finalX, finalY);
    }
    public static FieldPoint estimatePose(FieldPose2d roughPose, double[] measuredPointCloud, MapPoint predeterminedParticle) {
        if (areAllZeros(measuredPointCloud)) {
            return new FieldPoint(roughPose.getX(), roughPose.getY());
        }
        
        MapPoint robotPose = new MapPoint(roughPose.getX(), roughPose.getY());
        MapPoint[] roughParticles = generateParticles(robotPose, 20, 0.05, 0.01);
        
        // Add predetermined particle to the generated set
        MapPoint[] allParticles = new MapPoint[roughParticles.length + 1];
        System.arraycopy(roughParticles, 0, allParticles, 0, roughParticles.length);
        allParticles[allParticles.length - 1] = predeterminedParticle;

        double[] particleScores = scoreParticles(allParticles, measuredPointCloud,
                roughPose.getRotation().getDegrees());// lower is better
        MapPoint bestParticle = getBestParticle(allParticles, particleScores);

        // we have the 1st estimate now we need to refine this
        MapPoint[] moderateParticles = generateParticles(bestParticle, 20, 0.02, 0.002);
        double[] moderateParticleScores = scoreParticles(moderateParticles, measuredPointCloud,
                roughPose.getRotation().getDegrees());
        MapPoint bestModerateParticle = getBestParticle(moderateParticles, moderateParticleScores);

        MapPoint[] idealPointsSeen = LidarSimulator.getPerfectPointCloud(bestModerateParticle, roughPose.getRotation().getDegrees());

        double xConfidence = getXConfidence(idealPointsSeen);
        double yConfidence = getYConfidence(idealPointsSeen);

        double finalX = interpolate(roughPose.getX(), 1.0/4.0, bestModerateParticle.x, 1.0/(xConfidence+0.5));
        double finalY = interpolate(roughPose.getY(), 1.0/4.0, bestModerateParticle.y, 1.0/(yConfidence+0.5));

        System.out.println("X Confidence: " + xConfidence + " Y Confidence: " + yConfidence);

        return new FieldPoint(finalX, finalY);
    }

    public static double interpolate(double a, double aConfidence, double b, double bConfidence){
        return (a * aConfidence + b * bConfidence)/(aConfidence + bConfidence);
    }

    public static boolean areAllZeros(double[] measuredPointCloud) {
        for (int i = 0; i < measuredPointCloud.length; i++) {
            if (measuredPointCloud[i] != 0)
                return false;
        }
        return true;
    }
    public static boolean areAllZeros(double[][] measuredPointCloud) {
        for (int i = 0; i < measuredPointCloud.length; i++) {
            for(int j = 0; j < measuredPointCloud[i].length; j++){
                if (measuredPointCloud[i][j] != 0)
                    return false;
            }
        }
        return true;
    }
    public static boolean tooLittleUsefulData(double[][] measuredPointCloud) {
        int usefulPoints = 0;
        for (int i = 0; i < measuredPointCloud.length; i++) {
            for(int j = 0; j < measuredPointCloud[i].length; j++){
                if (measuredPointCloud[i][j] != 0)
                    usefulPoints++;
            }
        }
        return usefulPoints < 20;
    }
    // lower number is higher confidence
    public static double getXConfidence(MapPoint[] idealPointsSeen){
        double xVariance = 0;
        double maxX = idealPointsSeen[0].x;
        double minX = idealPointsSeen[0].x;
        for(int i = 1; i < idealPointsSeen.length; i++){
            if (idealPointsSeen[i].x > maxX){
                maxX = idealPointsSeen[i].x;
            }
            if (idealPointsSeen[i].x < minX){
                minX = idealPointsSeen[i].x;
            }
        }
        xVariance = maxX - minX;
        xVariance *= 10; // scale to be bigger number easy to read
        return xVariance * xVariance;
    }
    public static double getYConfidence(MapPoint[] idealPointsSeen){
        double yVariance = 0;
        double maxY = idealPointsSeen[0].y;
        double minY = idealPointsSeen[0].y;
        for(int i = 1; i < idealPointsSeen.length; i++){
            if (idealPointsSeen[i].y > maxY){
                maxY = idealPointsSeen[i].y;
            }
            if (idealPointsSeen[i].y < minY){
                minY = idealPointsSeen[i].y;
            }
        }
        yVariance = maxY - minY;
        yVariance *= 10; // scale to be bigger number easy to read
        return yVariance * yVariance;
    }

    public static MapPoint getBestParticle(MapPoint[] particles, double[] particleScores) {
        double bestScore = Double.MAX_VALUE;
        int bestIndex = -1;
        for (int i = 0; i < particleScores.length; i++) {
            if (particleScores[i] < bestScore) {
                bestScore = particleScores[i];
                bestIndex = i;
            }
        }
        SmartDashboard.putNumber("BestScore", bestScore);
        return particles[bestIndex];
    }
    public static MapPoint averageAmongBestParticles(MapPoint[] particles, double[] particleScores) {
        Integer[] indices = new Integer[particleScores.length];
        for (int i = 0; i < particleScores.length; i++) {
            indices[i] = i;
        }

        java.util.Arrays.sort(indices, (a, b) -> Double.compare(particleScores[a], particleScores[b]));

        int count = Math.min(4, particles.length);
        double weightedSumX = 0;
        double weightedSumY = 0;
        double totalWeight = 0;

        for (int i = 0; i < count; i++) {
            double score = particleScores[indices[i]];
            // Use inverse of score as weight. Small epsilon prevents division by zero
            // for perfect matches.
            double weight = 1.0 / (score + 1e-5); 

            weightedSumX += particles[indices[i]].x * weight;
            weightedSumY += particles[indices[i]].y * weight;
            totalWeight += weight;
        }
        
        if (count > 0) {
            SmartDashboard.putNumber("BestScore", particleScores[indices[0]]);
        } else {
             // Fallback if no particles provided
            return new MapPoint(0, 0);
        }

        return new MapPoint(weightedSumX / totalWeight, weightedSumY / totalWeight);
    }

    public static double[] scoreParticles(MapPoint[] roughParticles, double[] measuredPointCloud,
            double robotAngleDegrees) {
        // lower is better
        double maxLidarRange = 0.3; // 30cm limit from simulator
        double minLidarRange = 0.025; // 25mm limit

        return java.util.stream.IntStream.range(0, roughParticles.length).parallel().mapToDouble(i -> {
            // we just assume rotation is perfect for now
            double[] expectedMeasurementAtParticle = LidarSimulator.getPerfectSimData(roughParticles[i],
                    robotAngleDegrees);
            double score = 0;
            for (int j = 0; j < measuredPointCloud.length; j++) {
                double m = measuredPointCloud[j];
                double e = expectedMeasurementAtParticle[j];

                if (m == 0 && e == 0) {
                    continue; // Both agree on "out of range"
                } else if (m != 0 && e != 0) {
                     // Both valid, standard error
                    score += (m - e) * (m - e);
                } else {
                    // One is valid, one is 0 (out of range/invalid)
                    double v = (m != 0) ? m : e;
                    
                    double distToLow = Math.abs(v - minLidarRange);
                    double distToHigh = Math.abs(maxLidarRange - v);
                    
                    double error = Math.min(distToLow, distToHigh);
                    score += error * error;
                }
            }
            return score;
        }).toArray();
    }
    public static double[] scoreParticles(MapPoint[] roughParticles, double[][] measuredPointClouds,
            double robotAngleDegrees) {
        // lower is better
        double maxLidarRange = 0.3; // 30cm limit from simulator
        double minLidarRange = 0.025; // 25mm limit

        return java.util.stream.IntStream.range(0, roughParticles.length).parallel().mapToDouble(i -> {
            // we just assume rotation is perfect for now
            double[][] expectedMeasurementsAtParticle = LidarSimulator.getDualLidarSimData(roughParticles[i],
                    robotAngleDegrees, 0, LidarConstants.kLidarHorizontalOffsetFromCenterMeters, LidarConstants.kLidarAngleOffsetFromForwardDegrees);
            double score = 0;
            for (int j = 0; j < measuredPointClouds.length; j++) {
                for(int l = 0; l < measuredPointClouds[j].length; l++){
                    if (l >= expectedMeasurementsAtParticle[j].length) break; // Check bounds

                    double m = measuredPointClouds[j][l];
                    double e = expectedMeasurementsAtParticle[j][l];

                    if (m == 0 && e == 0) {
                        continue; // Both agree on "out of range"
                    } else if (m != 0 && e != 0) {
                        // Both valid, standard error
                        score += (m - e) * (m - e);
                    } else {
                        // One is valid, one is 0 (out of range/invalid)
                        // The valid one 'v' is in [0.025, 0.3]
                        // The invalid one 'inv' implies value in [0, 0.025) U (0.3, inf)
                        // We calculate distance from 'v' to the nearest valid region of 'inv'
                        
                        double v = (m != 0) ? m : e;
                        
                        double distToLow = Math.abs(v - minLidarRange);
                        double distToHigh = Math.abs(maxLidarRange - v);
                        
                        // We prefer the explanation that minimizes the error (benefit of the doubt)
                        double error = Math.min(distToLow, distToHigh);
                        score += error * error;
                    }
                }
                
            }
            return score;
        }).toArray();
    }





    public static MapPoint[] generateParticles(MapPoint estimatedPose, int count, double maxSpread, double minSpread) {
        MapPoint[] roughParticles = new MapPoint[count];
        roughParticles[roughParticles.length - 1] = estimatedPose; // add the rough pose as a particle so we dont
                                                                   // decrease in accuracy
        for (int i = 0; i < roughParticles.length - 1; i++) {
            roughParticles[i] = new MapPoint(
                    estimatedPose.x + Math.random() * maxSpread - maxSpread / 2,
                    estimatedPose.y + Math.random() * maxSpread - maxSpread / 2);
            int tooCloseCount = 0;
            while (isTooClose(roughParticles[i], roughParticles, (short) i, minSpread) && tooCloseCount < 4) {// regenerarte
                                                                                                              // if too
                                                                                                              // close
                                                                                                              // to
                                                                                                              // previous
                                                                                                              // points
                tooCloseCount++;
                roughParticles[i] = new MapPoint(
                        estimatedPose.x + Math.random() * maxSpread - maxSpread / 2,
                        estimatedPose.y + Math.random() * maxSpread - maxSpread / 2);
            }
        }
        return roughParticles;
    }

    public static boolean isTooClose(MapPoint pointToCheck, MapPoint[] PreviousPoints, short arrayPosition,
            double minDistance) {
        double minDistSq = minDistance * minDistance;
        for (int i = 0; i < arrayPosition; i++) {
            double distSq = (pointToCheck.x - PreviousPoints[i].x) * (pointToCheck.x - PreviousPoints[i].x) +
                            (pointToCheck.y - PreviousPoints[i].y) * (pointToCheck.y - PreviousPoints[i].y);
            if (distSq < minDistSq) {
                return true;
            }
        }
        return false;
    }





    public static FieldPoint estimatePoseDualLidar(FieldPose2d roughPose, double[][] measuredPointClouds, Polygon[] map){
        if (tooLittleUsefulData(measuredPointClouds)) {
            return new FieldPoint(roughPose.getX(), roughPose.getY());
        }
        MapPoint robotPose = new MapPoint(roughPose.getX(), roughPose.getY());
        MapPoint[] roughParticles = generateParticles(robotPose, 45, 0.05, 0.006);
        double[] particleScores = scoreParticlesNoTwist(roughParticles, measuredPointClouds,
                roughPose.getRotation().getDegrees(), map);
        MapPoint bestParticle = averageAmongBestParticles(roughParticles, particleScores);

        // we have the 1st estimate now we need to refine this
        MapPoint[] moderateParticles = generateParticles(bestParticle, 20, 0.02, 0.001);
        double[] moderateParticleScores = scoreParticlesNoTwist(moderateParticles, measuredPointClouds,
                roughPose.getRotation().getDegrees(), map);
        MapPoint bestModerateParticle = averageAmongBestParticles(moderateParticles, moderateParticleScores);

        // Skip 3rd pass for speed
        return new FieldPoint(bestModerateParticle.x, bestModerateParticle.y);
    }

    public static double[] scoreParticlesNoTwist(MapPoint[] roughParticles, double[][] measuredPointClouds,
            double robotAngleDegrees, frc.robot.extras.LidarMapComponents.Polygon[] map) {
        // lower is better
        double maxLidarRange = 0.3; // 30cm limit from simulator
        double minLidarRange = 0.025; // 25mm limit
        
        // Single angle check
        return java.util.stream.IntStream.range(0, roughParticles.length).parallel().mapToDouble(i -> {
            double testingAngle = robotAngleDegrees;
            
            double[][] expectedMeasurementsAtParticle = LidarSimulator.getDualLidarSimData(roughParticles[i],   
                    testingAngle, 0, LidarConstants.kLidarHorizontalOffsetFromCenterMeters, LidarConstants.kLidarAngleOffsetFromForwardDegrees, map);
            double score = 0;
            for (int j = 0; j < measuredPointClouds.length; j++) {
                for(int l = 0; l < measuredPointClouds[j].length; l++){
                    if (l >= expectedMeasurementsAtParticle[j].length) break; // Check bounds

                    double m = measuredPointClouds[j][l];
                    double e = expectedMeasurementsAtParticle[j][l];

                    if (m == 0 && e == 0) {
                        continue; // Both agree on "out of range"
                    } else if (m != 0 && e != 0) {
                        // Both valid, standard error
                        score += (m - e) * (m - e);
                    } else {
                        // One is valid, one is 0 (out of range/invalid)
                        double v = (m != 0) ? m : e;
                        
                        double distToLow = Math.abs(v - minLidarRange);
                        double distToHigh = Math.abs(maxLidarRange - v);
                        
                        double error = Math.min(distToLow, distToHigh);
                        score += error * error;
                    }
                }
            }
            return score;
        }).toArray();
    }
    
    // Overloads with Map support
    public static double[] scoreParticles(MapPoint[] roughParticles, double[] measuredPointCloud,
            double robotAngleDegrees, frc.robot.extras.LidarMapComponents.Polygon[] map) {
         // lower is better
        double maxLidarRange = 0.3; // 30cm limit from simulator
        double minLidarRange = 0.025; // 25mm limit

        return java.util.stream.IntStream.range(0, roughParticles.length).parallel().mapToDouble(i -> {
            double[] expectedMeasurementAtParticle = LidarSimulator.getPerfectSimData(roughParticles[i],
                    robotAngleDegrees, map);
            double score = 0;
            for (int j = 0; j < measuredPointCloud.length; j++) {
                double m = measuredPointCloud[j];
                double e = expectedMeasurementAtParticle[j];

                if (m == 0 && e == 0) {
                    continue; // Both agree on "out of range"
                } else if (m != 0 && e != 0) {
                    score += (m - e) * (m - e);
                } else {
                    double v = (m != 0) ? m : e;
                    double distToLow = Math.abs(v - minLidarRange);
                    double distToHigh = Math.abs(maxLidarRange - v);
                    double error = Math.min(distToLow, distToHigh);
                    score += error * error;
                }
            }
            return score;
        }).toArray();
    }




}