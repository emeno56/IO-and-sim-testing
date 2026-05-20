package frc.robot.util;

import java.util.function.Supplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.util.Units;
import frc.robot.subsystems.customDrive.Drive;

import static frc.robot.util.Bump.*;

public class BumpUtil {
    private static double kminBumpV = 1;

    boolean isOnBump = false;
    Supplier<Pose2d> robotPose;
    Supplier<Pose2d> checkPose;

    public BumpUtil(Supplier<Pose2d> checkPose, Supplier<Pose2d> robotPose) {
        this.robotPose = robotPose;
        this.checkPose = checkPose;
    }

    /**
     * to be called periodically to keep checking whether the robot is in the bump;
     */

    public void updateChecks() {
        for(Bump bump : BUMPS) {
            if(bump.isInBump(checkPose.get())) {
                isOnBump = true;
                return;
            }
        }
        isOnBump = false;
    }

    public boolean isOnBump() {
        return isOnBump;
    }

    public Pose3d bumpVisualizePose() {
        double x = robotPose.get().getX();
        double y = robotPose.get().getY();
        double roll = calculateRoll();
        double pitch = calculatePitch();
        double yaw = robotPose.get().getRotation().getRadians();
        double z = calculateZ();
        return new Pose3d(x,y,z,new Rotation3d(roll, pitch, yaw));
    }

   public double calculatePitch() {
        double worldSlopeAngle = getWorldSlopeAngle();
        if (worldSlopeAngle == 0.0) return 0.0;

        // Project world X-slope onto robot's forward axis
        // Robot heading: angle of robot's +X (forward) in world frame
        double heading = robotPose.get().getRotation().getRadians();

        // The bump slopes in the world X direction.
        // The component of robot's forward axis along world X is cos(heading).
        return -worldSlopeAngle * Math.cos(heading);
    }

    public double calculateRoll() {
        double worldSlopeAngle = getWorldSlopeAngle();
        if (worldSlopeAngle == 0.0) return 0.0;

        double heading = robotPose.get().getRotation().getRadians();

        // The component of robot's LEFT axis along world X is -sin(heading).
        // A slope pushing the left side up = positive roll in WPILib convention.
        return worldSlopeAngle * -Math.sin(heading);
    }

    private double getWorldSlopeAngle() {
        for (Bump bump : BUMPS) {
            if (!bump.isInBump(robotPose.get())) continue;

            double physicalStartX = bump.getPhysicalStartX();
            double physicalEndX   = bump.getPhysicalEndX();
            double physicalMidX   = (physicalStartX + physicalEndX) / 2.0;

            double robotCenterX = robotPose.get().getX();

            // Only apply slope if robot center is within physical bump bounds
            if (robotCenterX < physicalStartX || robotCenterX > physicalEndX) {
                return 0.0;
            }

            return (robotCenterX < physicalMidX)
                ? Units.degreesToRadians(15.0)
                : Units.degreesToRadians(-15.0);
        }
        return 0.0;
    }

    public double calculateZ() {
        double height = Units.inchesToMeters(6.513);

        for (Bump bump : BUMPS) {
            if (!bump.isInBump(robotPose.get())) continue;

            double bumpStartX = bump.getPhysicalStartX();
            double bumpEndX   = bump.getPhysicalEndX();
            double bumpDepth  = bumpEndX - bumpStartX;
            double rampDepth  = bumpDepth / 2.0;

            double robotCenterX = robotPose.get().getX();
            double posInBump = MathUtil.clamp(robotCenterX - bumpStartX, 0.0, bumpDepth);

            double surfaceHeight;
            if (posInBump <= rampDepth) {
                surfaceHeight = height * (posInBump / rampDepth);
            } else {
                surfaceHeight = height * (1.0 - (posInBump - rampDepth) / rampDepth);
            }

            return surfaceHeight;
        }
        return 0.0;
    }
}
