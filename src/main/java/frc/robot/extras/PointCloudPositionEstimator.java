package frc.robot.extras;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.LidarConstants;
import frc.robot.extras.LidarMapComponents.LidarSimulator;
import frc.robot.extras.LidarMapComponents.MapPoint;

//this class is designed to take in a point cloud and a rough pose estimate and return a refined pose estimate
//it does so by comparing the measured point cloud to simulated point clouds at various nearby positions and selecting the best match
//it does not however improve rotational accuracy yet

public class PointCloudPositionEstimator {
    public static FieldPoint estimatePose(FieldPose2d roughPose, double[] measuredPointCloud) {
        // Default to global map for backwards compatibility
        return estimatePose(roughPose, measuredPointCloud, frc.robot.extras.LidarMap.getMap());
    }

    public static FieldPoint estimatePose(FieldPose2d roughPose, double[] measuredPointCloud, frc.robot.extras.LidarMapComponents.Polygon[] map) {
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
    public static FieldPoint estimatePose(FieldPose2d roughPose, double[][] measuredPointCloud) {
        return estimatePose(roughPose, measuredPointCloud, frc.robot.extras.LidarMap.getMap());
    }
    public static FieldPoint estimatePoseWithAngleSearch(FieldPose2d roughPose, double[][] measuredPointCloud) {
        if (areAllZeros(measuredPointCloud)) {
            return new FieldPoint(roughPose.getX(), roughPose.getY());
        }
        MapPoint robotPose = new MapPoint(roughPose.getX(), roughPose.getY());
        MapPoint[] roughParticles = generateParticles(robotPose, 45, 0.05, 0.006);
        double[] particleScores = scoreParticlesWithTwist(roughParticles, measuredPointCloud,
                roughPose.getRotation().getDegrees());// lower is better
        MapPoint bestParticle = averageAmongBestParticles(roughParticles, particleScores);

        // we have the 1st estimate now we need to refine this
        MapPoint[] moderateParticles = generateParticles(bestParticle, 20, 0.02, 0.001);
        double[] moderateParticleScores = scoreParticlesWithTwist(moderateParticles, measuredPointCloud,
                roughPose.getRotation().getDegrees());
        MapPoint bestModerateParticle = averageAmongBestParticles(moderateParticles, moderateParticleScores);

        // 3rd pass for fine tuning
        MapPoint[] fineParticles = generateParticles(bestModerateParticle, 20, 0.006, 0.001);
        double[] fineParticleScores = scoreParticlesWithTwist(fineParticles, measuredPointCloud,
                roughPose.getRotation().getDegrees());
        MapPoint bestFineParticle = averageAmongBestParticles(fineParticles, fineParticleScores);

        return new FieldPoint(bestFineParticle.x, bestFineParticle.y);
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
    public static double[] scoreParticlesWithTwist(MapPoint[] roughParticles, double[][] measuredPointClouds,
            double robotAngleDegrees) {
        // lower is better
        double maxLidarRange = 0.3; // 30cm limit from simulator
        double minLidarRange = 0.025; // 25mm limit
        
        // Define angles to test relative to robotAngleDegrees
        double[] angleOffsets = {-.5, 0.0, .5}; 

        return java.util.stream.IntStream.range(0, roughParticles.length).parallel().mapToDouble(i -> {
            double bestParticleScore = Double.MAX_VALUE;

            for (double angleOffset : angleOffsets) {
                double testingAngle = robotAngleDegrees + angleOffset;
                
                double[][] expectedMeasurementsAtParticle = LidarSimulator.getDualLidarSimData(roughParticles[i],   
                        testingAngle, 0, LidarConstants.kLidarHorizontalOffsetFromCenterMeters, LidarConstants.kLidarAngleOffsetFromForwardDegrees);
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
                
                if (score < bestParticleScore) {
                    bestParticleScore = score;
                }
            }
            return bestParticleScore;
        }).toArray();
    }

    public static FieldPose2d refinePoseICP(FieldPose2d roughPose, double[][] measuredPointCloud) {
        int maxIterations = 40;
        double convergenceThreshold = 1e-6; 

        double x = roughPose.getX();
        double y = roughPose.getY();
        double theta = roughPose.getRotation().getRadians();

        double lidarOffsetH = LidarConstants.kLidarHorizontalOffsetFromCenterMeters; 
        double lidarAngleOffset = Math.toRadians(LidarConstants.kLidarAngleOffsetFromForwardDegrees);

        frc.robot.extras.LidarMapComponents.Polygon[] map = frc.robot.extras.LidarMap.getMap(); 
        
        java.util.ArrayList<frc.robot.extras.LidarMapComponents.LineSegment> segments = new java.util.ArrayList<>();
        for (frc.robot.extras.LidarMapComponents.Polygon p : map) {
            for (frc.robot.extras.LidarMapComponents.LineSegment s : p.LineSegmentArray) {
                segments.add(s);
            }
        }

        for (int iter = 0; iter < maxIterations; iter++) {
            double[][] A = new double[3][3];
            double[] B = new double[3];
            int pointsUsed = 0;

            for (int lidarIdx = 0; lidarIdx < 2; lidarIdx++) {
                double lidarBaseAngle = (lidarIdx == 0) ? -lidarAngleOffset : lidarAngleOffset;
                double lidarLocalX = 0; 
                double lidarLocalY = (lidarIdx == 0) ? lidarOffsetH : -lidarOffsetH;
                
                double[] cloud = measuredPointCloud[lidarIdx];

                for (int i = 0; i < cloud.length; i++) {
                    double range = cloud[i];
                    if (range < 0.01 || range > 10.0) continue; 

                    double rayAngle = lidarBaseAngle + Math.toRadians(i * 0.6 - 50);
                    double pLocalX = lidarLocalX + range * Math.cos(rayAngle);
                    double pLocalY = lidarLocalY + range * Math.sin(rayAngle);

                    double cosTheta = Math.cos(theta);
                    double sinTheta = Math.sin(theta);
                    double pGlobalX = x + pLocalX * cosTheta - pLocalY * sinTheta;
                    double pGlobalY = y + pLocalX * sinTheta + pLocalY * cosTheta;

                    double minDistSq = Double.MAX_VALUE;
                    frc.robot.extras.LidarMapComponents.LineSegment bestSeg = null;

                    for (frc.robot.extras.LidarMapComponents.LineSegment seg : segments) {
                        double distSq = getDistanceSqToSegment(pGlobalX, pGlobalY, seg);
                        if (distSq < minDistSq) {
                            minDistSq = distSq;
                            bestSeg = seg;
                        }
                    }

                    if (bestSeg != null && minDistSq < 0.2) { 
                         double segDx = bestSeg.endPoint.x - bestSeg.startPoint.x;
                         double segDy = bestSeg.endPoint.y - bestSeg.startPoint.y;
                         
                         // Tighter bounds check (approx 1m extension allowed for 10m wall)
                         double t = ((pGlobalX - bestSeg.startPoint.x) * segDx + (pGlobalY - bestSeg.startPoint.y) * segDy) / (segDx * segDx + segDy * segDy);
                         if (t < -0.1 || t > 1.1) continue;

                         double segLen = Math.hypot(segDx, segDy);
                         double nx = -segDy / segLen;
                         double ny = segDx / segLen;
                         
                         double C = (pGlobalX - bestSeg.startPoint.x) * nx + (pGlobalY - bestSeg.startPoint.y) * ny;
                         
                         double vx = pGlobalX - x;
                         double vy = pGlobalY - y;
                         
                         double J_rot = -nx * vy + ny * vx; 
                         
                         // Improved Weighting:
                         // 1. Clamp range to avoid singularity
                         // 2. Add robust loss (Huber-like): downweight large residuals
                         double safeRange = Math.max(range, 0.1); 
                         double rangeWeight = 1.0 / (safeRange * safeRange);
                         
                         double residual = Math.abs(C);
                         double robustWeight = 1.0;
                         if (residual > 0.05) { // If error > 5cm, reduce influence
                             robustWeight = 0.05 / residual;
                         }

                         double w = rangeWeight * robustWeight;

                         A[0][0] += w * nx*nx;
                         A[0][1] += w * nx*ny;
                         A[0][2] += w * nx*J_rot;
                         
                         A[1][0] += w * nx*ny; 
                         A[1][1] += w * ny*ny;
                         A[1][2] += w * ny*J_rot;
                         
                         A[2][0] += w * nx*J_rot; 
                         A[2][1] += w * ny*J_rot;
                         A[2][2] += w * J_rot*J_rot;
                         
                         B[0] += w * nx * (-C);
                         B[1] += w * ny * (-C);
                         B[2] += w * J_rot * (-C);
                         
                         pointsUsed++;
                    }
                }
            }
            
            if (pointsUsed < 10) break; 
            
            double[] sol = solve3x3(A, B);
            if (sol == null) break; 
            
            x += sol[0];
            y += sol[1];
            theta += sol[2];
            
            if (Math.abs(sol[0]) < convergenceThreshold && Math.abs(sol[1]) < convergenceThreshold && Math.abs(sol[2]) < Math.toRadians(0.01)) {
                break;
            }
        }
        
        return new FieldPose2d(x, y, new edu.wpi.first.math.geometry.Rotation2d(theta));
    }

    private static double getDistanceSqToSegment(double px, double py, frc.robot.extras.LidarMapComponents.LineSegment seg) {
        double x1 = seg.startPoint.x;
        double y1 = seg.startPoint.y;
        double x2 = seg.endPoint.x;
        double y2 = seg.endPoint.y;
        double dx = x2 - x1;
        double dy = y2 - y1;
        if (dx == 0 && dy == 0) return (px - x1)*(px - x1) + (py - y1)*(py - y1);
        
        double t = ((px - x1) * dx + (py - y1) * dy) / (dx * dx + dy * dy);
        t = Math.max(0, Math.min(1, t));
        double closestX = x1 + t * dx;
        double closestY = y1 + t * dy;
        return (px - closestX)*(px - closestX) + (py - closestY)*(py - closestY);
    }

    private static double[] solve3x3(double[][] A, double[] B) {
        double det = A[0][0] * (A[1][1] * A[2][2] - A[1][2] * A[2][1]) -
                     A[0][1] * (A[1][0] * A[2][2] - A[1][2] * A[2][0]) +
                     A[0][2] * (A[1][0] * A[2][1] - A[1][1] * A[2][0]);

        if (Math.abs(det) < 1e-9) return null;

        double invDet = 1.0 / det;
        double[] X = new double[3];

        X[0] = (B[0] * (A[1][1] * A[2][2] - A[1][2] * A[2][1]) -
                A[0][1] * (B[1] * A[2][2] - A[1][2] * B[2]) +
                A[0][2] * (B[1] * A[2][1] - A[1][1] * B[2])) * invDet;

        X[1] = (A[0][0] * (B[1] * A[2][2] - A[1][2] * B[2]) -
                B[0] * (A[1][0] * A[2][2] - A[1][2] * A[2][0]) +
                A[0][2] * (A[1][0] * B[2] - B[1] * A[2][0])) * invDet;

        X[2] = (A[0][0] * (A[1][1] * B[2] - B[1] * A[2][1]) -
                A[0][1] * (A[1][0] * B[2] - B[1] * A[2][0]) +
                B[0] * (A[1][0] * A[2][1] - A[1][1] * A[2][0])) * invDet;
        
        return X;
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

    public static FieldPoint estimatePose(FieldPose2d roughPose, double[][] measuredPointCloud, frc.robot.extras.LidarMapComponents.Polygon[] map) {
        if (tooLittleUsefulData(measuredPointCloud)) {
            return new FieldPoint(roughPose.getX(), roughPose.getY());
        }
        MapPoint robotPose = new MapPoint(roughPose.getX(), roughPose.getY());
        MapPoint[] roughParticles = generateParticles(robotPose, 45, 0.05, 0.006);
        double[] particleScores = scoreParticlesWithTwist(roughParticles, measuredPointCloud,
                roughPose.getRotation().getDegrees(), map);
        MapPoint bestParticle = averageAmongBestParticles(roughParticles, particleScores);

        // we have the 1st estimate now we need to refine this
        MapPoint[] moderateParticles = generateParticles(bestParticle, 20, 0.02, 0.001);
        double[] moderateParticleScores = scoreParticlesWithTwist(moderateParticles, measuredPointCloud,
                roughPose.getRotation().getDegrees(), map);
        MapPoint bestModerateParticle = averageAmongBestParticles(moderateParticles, moderateParticleScores);

        // 3rd pass for fine tuning
        MapPoint[] fineParticles = generateParticles(bestModerateParticle, 20, 0.006, 0.001);
        double[] fineParticleScores = scoreParticlesWithTwist(fineParticles, measuredPointCloud,
                roughPose.getRotation().getDegrees(), map);
        MapPoint bestFineParticle = averageAmongBestParticles(fineParticles, fineParticleScores);

        FieldPose2d icpStart = new FieldPose2d(bestFineParticle.x, bestFineParticle.y, roughPose.getRotation());
        FieldPose2d refinedPose = refinePoseICP(icpStart, measuredPointCloud, map);

        return new FieldPoint(refinedPose.getX(), refinedPose.getY());
    }

    public static double[] scoreParticles(MapPoint[] roughParticles, double[] measuredPointCloud,
            double robotAngleDegrees, frc.robot.extras.LidarMapComponents.Polygon[] map) {
        // lower is better
        double maxLidarRange = 0.3; // 30cm limit from simulator
        double minLidarRange = 0.025; // 25mm limit

        return java.util.stream.IntStream.range(0, roughParticles.length).parallel().mapToDouble(i -> {
            // we just assume rotation is perfect for now
            double[] expectedMeasurementAtParticle = LidarSimulator.getPerfectSimData(roughParticles[i],
                    robotAngleDegrees, map);
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
    public static double[] scoreParticlesWithTwist(MapPoint[] roughParticles, double[][] measuredPointClouds,
            double robotAngleDegrees, frc.robot.extras.LidarMapComponents.Polygon[] map) {
        // lower is better
        double maxLidarRange = 0.3; // 30cm limit from simulator
        double minLidarRange = 0.025; // 25mm limit
        
        // Define angles to test relative to robotAngleDegrees
        double[] angleOffsets = {-.5, 0.0, .5}; 

        return java.util.stream.IntStream.range(0, roughParticles.length).parallel().mapToDouble(i -> {
            double bestParticleScore = Double.MAX_VALUE;

            for (double angleOffset : angleOffsets) {
                double testingAngle = robotAngleDegrees + angleOffset;
                
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
                
                if (score < bestParticleScore) {
                    bestParticleScore = score;
                }
            }
            return bestParticleScore;
        }).toArray();
    }
    public static FieldPose2d refinePoseICP(FieldPose2d roughPose, double[][] measuredPointCloud, frc.robot.extras.LidarMapComponents.Polygon[] map) {
        int maxIterations = 40;
        double convergenceThreshold = 1e-6; 

        double x = roughPose.getX();
        double y = roughPose.getY();
        double theta = roughPose.getRotation().getRadians();

        double lidarOffsetH = LidarConstants.kLidarHorizontalOffsetFromCenterMeters; 
        double lidarAngleOffset = Math.toRadians(LidarConstants.kLidarAngleOffsetFromForwardDegrees);

        java.util.ArrayList<frc.robot.extras.LidarMapComponents.LineSegment> segments = new java.util.ArrayList<>();
        for (frc.robot.extras.LidarMapComponents.Polygon p : map) {
            for (frc.robot.extras.LidarMapComponents.LineSegment s : p.LineSegmentArray) {
                segments.add(s);
            }
        }

        for (int iter = 0; iter < maxIterations; iter++) {
            double[][] A = new double[3][3];
            double[] B = new double[3];
            int pointsUsed = 0;

            for (int lidarIdx = 0; lidarIdx < 2; lidarIdx++) {
                double lidarBaseAngle = (lidarIdx == 0) ? -lidarAngleOffset : lidarAngleOffset;
                double lidarLocalX = 0; 
                double lidarLocalY = (lidarIdx == 0) ? lidarOffsetH : -lidarOffsetH;
                
                double[] cloud = measuredPointCloud[lidarIdx];

                for (int i = 0; i < cloud.length; i += 2) {
                    double range = cloud[i];
                    if (range < 0.01 || range > 10.0) continue; 

                    double rayAngle = lidarBaseAngle + Math.toRadians(i * 0.6 - 50);
                    double pLocalX = lidarLocalX + range * Math.cos(rayAngle);
                    double pLocalY = lidarLocalY + range * Math.sin(rayAngle);

                    double cosTheta = Math.cos(theta);
                    double sinTheta = Math.sin(theta);
                    double pGlobalX = x + pLocalX * cosTheta - pLocalY * sinTheta;
                    double pGlobalY = y + pLocalX * sinTheta + pLocalY * cosTheta;

                    double minDistSq = Double.MAX_VALUE;
                    frc.robot.extras.LidarMapComponents.LineSegment bestSeg = null;

                    for (frc.robot.extras.LidarMapComponents.LineSegment seg : segments) {
                        double distSq = getDistanceSqToSegment(pGlobalX, pGlobalY, seg);
                        if (distSq < minDistSq) {
                            minDistSq = distSq;
                            bestSeg = seg;
                        }
                    }

                    if (bestSeg != null && minDistSq < 0.01) { 
                         double segDx = bestSeg.endPoint.x - bestSeg.startPoint.x;
                         double segDy = bestSeg.endPoint.y - bestSeg.startPoint.y;
                         
                         double t = ((pGlobalX - bestSeg.startPoint.x) * segDx + (pGlobalY - bestSeg.startPoint.y) * segDy) / (segDx * segDx + segDy * segDy);
                         if (t < -0.1 || t > 1.1) continue;

                         double segLen = Math.hypot(segDx, segDy);
                         double nx = -segDy / segLen;
                         double ny = segDx / segLen;
                         
                         double dxToRobot = x - pGlobalX;
                         double dyToRobot = y - pGlobalY;
                         if (nx * dxToRobot + ny * dyToRobot < 0) {
                             nx = -nx; 
                             ny = -ny;
                         }
                         
                         double C = (pGlobalX - bestSeg.startPoint.x) * nx + (pGlobalY - bestSeg.startPoint.y) * ny;
                         
                         double vx = pGlobalX - x;
                         double vy = pGlobalY - y;
                         
                         double J_rot = -nx * vy + ny * vx; 
                         
                         double safeRange = Math.max(range, 0.1); 
                         double w = 1.0 / (safeRange * safeRange);

                         A[0][0] += w * nx*nx;
                         A[0][1] += w * nx*ny;
                         A[0][2] += w * nx*J_rot;
                         
                         A[1][0] += w * nx*ny; 
                         A[1][1] += w * ny*ny;
                         A[1][2] += w * ny*J_rot;
                         
                         A[2][0] += w * nx*J_rot; 
                         A[2][1] += w * ny*J_rot;
                         A[2][2] += w * J_rot*J_rot;
                         
                         B[0] += w * nx * (-C);
                         B[1] += w * ny * (-C);
                         B[2] += w * J_rot * (-C);
                         
                         pointsUsed++;
                    }
                }
            }
            
            if (pointsUsed < 10) break; 
            
            double damping = 0.5; 
            A[0][0] += damping;
            A[1][1] += damping;
            A[2][2] += damping;
            
            double[] sol = solve3x3(A, B);
            if (sol == null) break; 
            
            x += sol[0];
            y += sol[1];
            theta += sol[2];
            
            theta = edu.wpi.first.math.MathUtil.angleModulus(theta);
            
            if (Math.abs(sol[0]) < convergenceThreshold && Math.abs(sol[1]) < convergenceThreshold && Math.abs(sol[2]) < Math.toRadians(0.01)) {
                break;
            }
        }
        
        return new FieldPose2d(x, y, new edu.wpi.first.math.geometry.Rotation2d(theta));
    }
}