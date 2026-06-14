package frc.robot.util;

import static frc.robot.Constants.FIELD_LENGTH_M;
import static frc.robot.Constants.FIELD_WIDTH_M;
import static frc.robot.Constants.getHubLocation;

import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.drivesims.AbstractDriveTrainSimulation;
import org.ironmaple.simulation.seasonspecific.rebuilt2026.RebuiltFuelOnField;
import org.ironmaple.simulation.seasonspecific.rebuilt2026.RebuiltHub;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;

public class FuelUtil {
    private static final double fuelDiameter = 0.15; //15 cm
    private static final double fuelSpacing = 0.0025; //2.5 mm
    private static final double centerLine = Units.inchesToMeters(2); //the 2 inch gap between the fuel at the center of the field
    private static final double hopperX = 0.3;
    private static final double hopperY = 0.0;
    private static final double BALL_HEIGHT_METERS = Units.inchesToMeters(3.0);

    public static void fullFieldReset() {
        //the corner fuel in the neutral zone
        Translation2d redRightCornerFuel = new Translation2d(FIELD_LENGTH_M / 2 + Units.inchesToMeters(36) - fuelDiameter / 2, FIELD_WIDTH_M / 2 + Units.inchesToMeters(103) - fuelDiameter / 2 - fuelDiameter * 2.0);
        Translation2d redLeftCornerFuel = new Translation2d(redRightCornerFuel.getX(), FIELD_WIDTH_M - redRightCornerFuel.getY());
        Translation2d blueRightCornerFuel = new Translation2d(FIELD_LENGTH_M - redLeftCornerFuel.getX(), redLeftCornerFuel.getY());
        Translation2d blueLeftCornerFuel = new Translation2d(blueRightCornerFuel.getX(), FIELD_WIDTH_M - blueRightCornerFuel.getY());
        //clear the gamepieces
        SimulatedArena.getInstance().clearGamePieces();
        //add all of the starting gamepieces, 504 in total
        makeRedRightCornerFuel(redRightCornerFuel);
        makeRedLeftCornerFuel(redLeftCornerFuel);
        makeBlueRightCornerFuel(blueRightCornerFuel);
        makeBlueLeftCornerFuel(blueLeftCornerFuel);
        makeRedDepot();
        makeBlueDepot();
    }

    private static void makeRedRightCornerFuel(Translation2d outsideCornerFuel) {
        //actual center of the field
        Translation2d center = new Translation2d(FIELD_LENGTH_M / 2, FIELD_WIDTH_M / 2);
        //adjust the center x and y for the center of the first fuel
        center = new Translation2d(center.getX() + fuelDiameter / 2, center.getY());
        center = new Translation2d(center.getX(), center.getY() + centerLine / 2 + fuelDiameter / 2);
        //go in order of a grid to place all of the fuel in a quadrant
        for(double x = center.getX(); x <= outsideCornerFuel.getX(); x += fuelDiameter + fuelSpacing) {
            for(double y = center.getY(); y <= outsideCornerFuel.getY(); y += fuelDiameter + fuelSpacing) {
                SimulatedArena.getInstance().addGamePiece(new RebuiltFuelOnField(new Translation2d(x, y)));
            }
        }
    }

    private static void makeRedLeftCornerFuel(Translation2d outsideCornerFuel) {
        //create center
        Translation2d center = new Translation2d(FIELD_LENGTH_M / 2, FIELD_WIDTH_M / 2);
        //change center based on which fuel quadrant we are in
        center = new Translation2d(center.getX() + fuelDiameter / 2, center.getY() - centerLine / 2 - fuelDiameter / 2);
        //apply fuel
        for(double x = center.getX(); x <= outsideCornerFuel.getX(); x += fuelDiameter + fuelSpacing) {
            for(double y = outsideCornerFuel.getY(); y <= center.getY(); y += fuelDiameter + fuelSpacing) {
                SimulatedArena.getInstance().addGamePiece(new RebuiltFuelOnField(new Translation2d(x, y)));
            }
        }
    }

    private static void makeBlueRightCornerFuel(Translation2d outsideCornerFuel) {
        //create center
        Translation2d center = new Translation2d(FIELD_LENGTH_M / 2, FIELD_WIDTH_M / 2);
        //change center based on which fuel quadrant we are in
        center = new Translation2d(center.getX() - fuelDiameter / 2, center.getY() - centerLine / 2 - fuelDiameter / 2);
        //apply fuel
        for(double x = outsideCornerFuel.getX(); x <= center.getX(); x += fuelDiameter + fuelSpacing) {
            for(double y = outsideCornerFuel.getY(); y <= center.getY(); y += fuelDiameter + fuelSpacing) {
                SimulatedArena.getInstance().addGamePiece(new RebuiltFuelOnField(new Translation2d(x, y)));
            }
        }
    }

    private static void makeBlueLeftCornerFuel(Translation2d outsideCornerFuel) {
        //create center
        Translation2d center = new Translation2d(FIELD_LENGTH_M / 2, FIELD_WIDTH_M / 2);
        //change center based on which fuel quadrant we are in
        center = new Translation2d(center.getX() - fuelDiameter / 2, center.getY() + centerLine / 2 + fuelDiameter / 2);
        //apply fuel
        for(double x = outsideCornerFuel.getX(); x <= center.getX(); x += fuelDiameter + fuelSpacing) {
            for(double y = center.getY(); y <= outsideCornerFuel.getY(); y += fuelDiameter + fuelSpacing) {
                SimulatedArena.getInstance().addGamePiece(new RebuiltFuelOnField(new Translation2d(x, y)));
            }
        }
    }

    private static void makeRedDepot() {
        // start and end x and y coordinates for depots
        double startX = FIELD_LENGTH_M - fuelDiameter * 4 - fuelSpacing * 3 + fuelDiameter / 2;
        double endX = FIELD_LENGTH_M - fuelDiameter / 2;
        double startY = Units.inchesToMeters(82.84) - fuelDiameter * 3 - fuelSpacing * 2 + fuelDiameter / 2;
        double endY = startY + fuelDiameter * 6 + fuelSpacing * 5;
        //add fuel
        for(double x = startX; x <= endX; x += fuelDiameter + fuelSpacing) {
            for(double y = startY; y <= endY; y += fuelDiameter + fuelSpacing) {
                SimulatedArena.getInstance().addGamePiece(new RebuiltFuelOnField(new Translation2d(x, y)));
            }
        }
    }

    private static void makeBlueDepot() {
        // start and end x and y coordinates for depots
        double startX = fuelDiameter / 2;
        double endX = fuelDiameter * 4 + fuelSpacing * 3 -fuelDiameter / 2;
        double startY = FIELD_WIDTH_M - Units.inchesToMeters(82.84) - fuelDiameter * 3 - fuelSpacing * 2 + fuelDiameter / 2;
        double endY = startY + fuelDiameter * 6 + fuelSpacing * 5;
        //add fuel
        for(double x = startX; x <= endX; x += fuelDiameter + fuelSpacing) {
            for(double y = startY; y <= endY; y += fuelDiameter + fuelSpacing) {
                SimulatedArena.getInstance().addGamePiece(new RebuiltFuelOnField(new Translation2d(x, y)));
            }
        }
    }

    public static void addHubExitFuel(boolean redHub) {
        Translation2d exitCenter = getHubLocation().getTranslation().plus(new Translation2d(redHub ? -1 * (Units.inchesToMeters(47) / 2.0) : (Units.inchesToMeters(47) / 2.0), 0));
        Translation2d exit = exitCenter.plus(new Translation2d(0, Math.random() * Units.inchesToMeters(47) - Units.inchesToMeters(47) / 2.0));
        RebuiltFuelOnField exitFuel = new RebuiltFuelOnField(exit);
        exitFuel.setLinearVelocity(2.5, 0);
        SimulatedArena.getInstance().addGamePiece(exitFuel);
    }

    public static Pose3d[] getFuelPosesFromRobot(Pose3d robotPose3d, int count, double extendDistanceInches) {
        if (count == 0) return new Pose3d[0];

        Pose3d[] poses = new Pose3d[count];

        int ballsPerRow = 6;
        int ballsPerCol = 4;
        int ballsPerLayer = ballsPerRow * ballsPerCol;

        int placed = 0;
        int slot = 0;

        while (placed < count) {
            int layer     = slot / ballsPerLayer;
            int remainder = slot % ballsPerLayer;
            int row       = remainder / ballsPerCol;
            int col       = remainder % ballsPerCol;

            slot++;

            // Skip front 2 rows (row <= 1) top 2 layers (layer >= 2)
            if (row <= 1 && layer >= 2) continue;

            // Safety — grid is full
            if (layer >= 4) break;

            double x = hopperX - Units.inchesToMeters(extendDistanceInches)
                        - (row * fuelDiameter)
                        - fuelDiameter / 2.0
                        + (1.5 * fuelDiameter)
                        + Units.inchesToMeters(1.0);

            double y = hopperY + ((col - 1.5) * fuelDiameter);

            double z = BALL_HEIGHT_METERS + Units.inchesToMeters(3.0) + (layer * fuelDiameter);

            poses[placed] = robotPose3d.plus(
                new Transform3d(x, y, z, new Rotation3d())
            );

            placed++;
        }

        return poses;
    }
}
