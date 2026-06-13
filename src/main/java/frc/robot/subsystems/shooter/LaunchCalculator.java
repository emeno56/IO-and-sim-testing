package frc.robot.subsystems.shooter;

import java.security.Timestamp;
import java.util.ArrayList;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.util.Units;

public class LaunchCalculator {
    //Constants
    private static final double kGravity = 9.81;

    private static final double kTargetHeight = Units.inchesToMeters(72);
    private static final double kTargetRadius = Units.inchesToMeters(41.7 / 2.0);

    private static final double kShooterOffsetX = 0.2; //??? in meters
    private static final double kShooterOffsetZ = 0.6; //??? meters
    private static final double kShooterOffsetY = 0;

    private static final double kMaxAngle = 45;

    private static final double kTimeStepSeconds = 0.01;

    private static final double kFuelMaxHeightM = 3.5;

    public static Pose3d[] simulatePoses(Pose3d robotPose, LaunchParameters parameters) {
        //mutable array because I don't know how long the shot is
        ArrayList<Pose3d> poses = new ArrayList<>();
        double height = kShooterOffsetZ; //starting height
        double time = 0; 

        double angle = 90 - parameters.angle;

        //calculate the x and z exit velocities
        double vx = parameters.mps * Math.cos(Math.toRadians(angle));
        double vz = parameters.mps * Math.sin(Math.toRadians(angle));

        //calculate shots until it hits the ground
        while(height > 0) {
            //calculate the new x by adding the robot offset to the velocity * time to get distance
            double x = kShooterOffsetX + vx * time;
            double z = getZComponentFromVelocityAndTime(vz, time);

            //add the new pose to the fuel array
            poses.add(new Pose3d(x, 0, z, Rotation3d.kZero));

            //update the boolean height condition
            height = z;
            //increase time
            time += kTimeStepSeconds;
        }

        //get the array from the list
        Pose3d[] fuelPoses = listToArray(poses);

        //put the pose3d map in front of the robot.
        for(int i = 0; i < fuelPoses.length; ++i) {
            fuelPoses[i] = robotPose.plus(
                new Transform3d(fuelPoses[i].getX(), fuelPoses[i].getY(), fuelPoses[i].getZ(), Rotation3d.kZero)
            );
        }

        return fuelPoses;
    }

    public static LaunchParameters calculateBestParameters(double distanceToTarget) {
        LaunchParameters parameters = null;
        distanceToTarget -= kShooterOffsetX;

        //check each angle with each speed to find the best shot
        for(double ang = 0; ang < kMaxAngle; ang += 1) {
            for(double mps = 1; mps < 20; mps += 1) {
                double vx = mps * Math.cos(Math.toRadians(90 - ang));
                double vz = mps * Math.sin(Math.toRadians(90 - ang));
                //if the fuel maximum height is greater than the provided limit, skip
                if(kShooterOffsetZ + Math.pow(vz, 2) / (2 * kGravity) > kFuelMaxHeightM) continue;

                //if the fuel height when the fuel is at the closest point of the target is less than the target height (doesn't reach target), bad parameters
                double timeToCloseTarget = (distanceToTarget - kTargetRadius) / vx;
                double heightAtCloseTargetTime = getZComponentFromVelocityAndTime(vz, timeToCloseTarget);
                if(heightAtCloseTargetTime < kTargetHeight) continue;

                //if the fuel height when the fuel is at the closes point of the target is greater than the target height (goes over target), no good
                double timeToFarTarget = (distanceToTarget + kTargetRadius) / vx;
                double heightAtFarTargetTime = getZComponentFromVelocityAndTime(vz, timeToFarTarget);
                if(heightAtFarTargetTime > kTargetHeight) continue;

                if(parameters == null) {
                    parameters = new LaunchParameters(mps, ang);
                    continue;
                }
                //if the vertical velocity is >= the current parameters vz (with a given toleracne of 10 cm/s
                    //if this is the first good solution, we found a launch solution
                    //else if thise solution is closer to the center of the target, use it instead
                if(vz >= parameters.getVz() - 0.1) {
                    double timeToTarget = distanceToTarget / vx;
                    double heightAtTargetTime = getZComponentFromVelocityAndTime(vz, timeToTarget);
                    double error = Math.abs(heightAtTargetTime - kTargetHeight);
                    if(parameters.error < 0) {
                        parameters = new LaunchParameters(mps, ang);
                        parameters.error = error;
                        continue;
                    }
                    if(error < parameters.error) {
                        parameters = new LaunchParameters(mps, ang);
                        parameters.error = error;
                    }
                }
            }
        }

        return parameters == null ? new LaunchParameters(0, 0) : parameters;
    }

    /** a little helper method to compose an array from ArrayList */
    public static Pose3d[] listToArray(ArrayList<Pose3d> list) {
        Pose3d[] ret = new Pose3d[list.size()];

        for(int i = 0; i < list.size(); i++) {
            ret[i] = list.get(i);
        }

        return ret;
    }

    
    private static double getZComponentFromVelocityAndTime(double vz, double time) {
        return kShooterOffsetZ + vz * time - (kGravity * time * time) / 2.0;
    }
    
    public static class LaunchParameters {
        public final double mps;
        public final double angle;
        public double error;
        private final double angleRad;
    
        public LaunchParameters(double mps, double angle) {
            this.mps = mps;
            this.angle = angle;
            error = -1;
            angleRad = Math.toRadians(90 - angle);
        }
    
        public double getVx() {
            return mps * Math.cos(angleRad);
        }
    
        public double getVz() {
            return mps * Math.sin(angleRad);
        }
    }
}
