package frc.robot.util.bump;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import frc.robot.subsystems.customDrive.Drive;

public class Bump {
    private static final double xToleranceM = 0.1;

    Translation2d corner1;
    Translation2d corner2;
    Translation2d physical1;
    Translation2d physical2;

    private Bump(Translation2d corner1, Translation2d corner2, Translation2d physical1, Translation2d physical2) {
        this.corner1 = corner1;
        this.corner2 = corner2;
        this.physical1 = physical1;
        this.physical2 = physical2;
    }

    private Bump(Translation2d corner1, Translation2d corner2) {
        this.corner1 = corner1;
        this.corner2 = corner2;
        this.physical1 = corner1;
        this.physical2 = corner2;
    }

    public boolean isInBump(Pose2d pose) {
        //if in the x limit of the corners of the box
        if((pose.getX() > corner1.getX() && pose.getX() < corner2.getX()) || (pose.getX() < corner1.getX() && pose.getX() > corner2.getX())) {
            //if in the y limit of the corners of the box
            if((pose.getY() > corner1.getY() && pose.getY() < corner2.getY()) || (pose.getY() < corner1.getY() && pose.getY() > corner2.getY())) {
                return true;
            }
        }
        return false;
    }

    public boolean isInPhysicalBump(Pose2d pose) {
        double startX = getPhysicalStartX(Drive.DRIVE_BASE_RADIUS);
        double endX   = getPhysicalEndX(Drive.DRIVE_BASE_RADIUS);
        double startY = Math.min(physical1.getY(), physical2.getY()) - Drive.DRIVE_BASE_RADIUS;
        double endY   = Math.max(physical1.getY(), physical2.getY()) + Drive.DRIVE_BASE_RADIUS;

        if (pose.getX() > startX && pose.getX() < endX) {
            if (pose.getY() > startY && pose.getY() < endY) {
                return true;
            }
        }
        return false;
    }
    
    public Translation2d getCorner1() {
        return corner1;
    }

    public Translation2d getCorner2() {
        return corner2;
    }

    public static Bump flipBumpToOtherSide(Bump fromBump) {
        double fieldWidth = 8.07;
        Translation2d newC1 = new Translation2d(fromBump.getCorner1().getX(), fieldWidth - fromBump.getCorner1().getY());
        Translation2d newC2 = new Translation2d(fromBump.getCorner2().getX(), fieldWidth - fromBump.getCorner2().getY());
        Translation2d newP1 = new Translation2d(fromBump.getPhysical1().getX(), fieldWidth - fromBump.getPhysical1().getY());
        Translation2d newP2 = new Translation2d(fromBump.getPhysical2().getX(), fieldWidth - fromBump.getPhysical2().getY());
        return new Bump(newC1, newC2, newP1, newP2);
    }

    public static Bump flipBumpToOtherAlliance(Bump fromBump) {
        double fieldLength = 16.54;
        Translation2d newC1 = new Translation2d(fieldLength - fromBump.getCorner1().getX(), fromBump.getCorner1().getY());
        Translation2d newC2 = new Translation2d(fieldLength - fromBump.getCorner2().getX(), fromBump.getCorner2().getY());
        Translation2d newP1 = new Translation2d(fieldLength - fromBump.getPhysical1().getX(), fromBump.getPhysical1().getY());
        Translation2d newP2 = new Translation2d(fieldLength - fromBump.getPhysical2().getX(), fromBump.getPhysical2().getY());
        return new Bump(newC1, newC2, newP1, newP2);
    }

    public Bump addRobotRadius(double robotRadiusM, double robotRadiusLimit, double idkyButItWorks) {
        Translation2d newC1 = new Translation2d(corner1.getX() - robotRadiusM, corner1.getY() + robotRadiusLimit);
        Translation2d newC2 = new Translation2d(corner2.getX() + robotRadiusM, corner2.getY() - idkyButItWorks);
        return new Bump(newC1, newC2, physical1, physical2);
    }

    public Bump addXTolerance(double toleranceM) {
        Translation2d newC1 = new Translation2d(corner1.getX() - toleranceM, corner1.getY());
        Translation2d newC2 = new Translation2d(corner2.getX() + toleranceM, corner2.getY());
        return new Bump(newC1, newC2, physical1, physical2);
    }

    public double getPhysicalStartX() {
        return Math.min(physical1.getX(), physical2.getX());
    }

    public double getPhysicalEndX() {
        return Math.max(physical1.getX(), physical2.getX());
    }

    public double getPhysicalStartX(double robotRadius) {
        return Math.min(physical1.getX(), physical2.getX()) - robotRadius;
    }

    public double getPhysicalEndX(double robotRadius) {
        return Math.max(physical1.getX(), physical2.getX()) + robotRadius;
    }

    public Translation2d getPhysical1() {
        return physical1;
    }

    public Translation2d getPhysical2() {
        return physical2;
    }

    //Bump Bumps
    public static final Bump BLUE_RIGHT_BUMP = new Bump(
        new Translation2d(
            Units.inchesToMeters(158.6), 
            1.668), 
        new Translation2d(
            Units.inchesToMeters(158.6 + 44.4), 
            1.668 + 1.854))
            .addRobotRadius(Drive.DRIVE_BASE_HYPOTONUSE, Drive.DRIVE_BASE_HALF_WIDTH, Drive.DRIVE_BASE_RADIUS)
            .addXTolerance(xToleranceM);
    public static final Bump BLUE_LEFT_BUMP = flipBumpToOtherSide(BLUE_RIGHT_BUMP);
    public static final Bump RED_LEFT_BUMP = flipBumpToOtherAlliance(BLUE_RIGHT_BUMP);
    public static final Bump RED_RIGHT_BUMP = flipBumpToOtherSide(flipBumpToOtherAlliance(BLUE_RIGHT_BUMP));
    public static final Bump[] BUMPS = {BLUE_LEFT_BUMP, BLUE_RIGHT_BUMP, RED_LEFT_BUMP, RED_RIGHT_BUMP};

}
