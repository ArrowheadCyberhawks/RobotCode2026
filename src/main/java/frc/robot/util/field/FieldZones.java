package frc.robot.util.field;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import frc.robot.util.geometry.AllianceFlipUtil;

public final class FieldZones {

        public static double trenchXMargin = Units.inchesToMeters(25.0);
        private static Zone trenchZone = buildTrenchZone();

        private static Zone buildTrenchZone() {
                double blueXNear = FieldConstants.LinesVertical.hubCenter - trenchXMargin;
                double blueXFar = FieldConstants.LinesVertical.hubCenter + trenchXMargin;
                double redXNear = FieldConstants.LinesVertical.oppHubCenter - trenchXMargin;
                double redXFar = FieldConstants.LinesVertical.oppHubCenter + trenchXMargin;
                Zone blueLeft = new Zone.RectangleZone(blueXNear, blueXFar,
                                FieldConstants.LinesHorizontal.leftTrenchOpenEnd, FieldConstants.fieldWidth);
                Zone blueRight = new Zone.RectangleZone(blueXNear, blueXFar, 0.0,
                                FieldConstants.LinesHorizontal.rightTrenchOpenStart);
                Zone redLeft = new Zone.RectangleZone(redXNear, redXFar,
                                FieldConstants.LinesHorizontal.leftTrenchOpenEnd, FieldConstants.fieldWidth);
                Zone redRight = new Zone.RectangleZone(redXNear, redXFar, 0.0,
                                FieldConstants.LinesHorizontal.rightTrenchOpenStart);
                return redLeft.union(redRight).union(blueLeft.union(blueRight));
        }

        public static final Zone blueAlliance = new Zone.RectangleZone(
                        0.0,
                        FieldConstants.LinesVertical.allianceZone,
                        0.0,
                        FieldConstants.fieldWidth);

        public static final Zone redAlliance = new Zone.RectangleZone(
                        FieldConstants.LinesVertical.oppAllianceZone,
                        FieldConstants.fieldLength,
                        0.0,
                        FieldConstants.fieldWidth);

        public static final Zone NEUTRAL = new Zone.RectangleZone(
                        FieldConstants.LinesVertical.neutralZoneNear,
                        FieldConstants.LinesVertical.neutralZoneFar,
                        0.0,
                        FieldConstants.fieldWidth);

        public static final Zone BLUETOWER = new Zone.RectangleZone(
                        0.0,
                        FieldConstants.Tower.leftUpright.getX(),
                        FieldConstants.Tower.rightUpright.getY(),
                        FieldConstants.Tower.leftUpright.getY());

        public static final Zone REDTOWER = new Zone.RectangleZone(
                        FieldConstants.Tower.oppLeftUpright.getX(),
                        FieldConstants.fieldLength,
                        FieldConstants.Tower.oppRightUpright.getY(),
                        FieldConstants.Tower.oppLeftUpright.getY());

        public static Zone nearAlliance() {
                return AllianceFlipUtil.shouldFlip() ? redAlliance : blueAlliance;
        }

        public static Zone opposingAlliance() {
                return AllianceFlipUtil.shouldFlip() ? blueAlliance : redAlliance;
        }

        public static Zone TRENCH() {
                return trenchZone;
        }

        // Maybe move these to robotcontainer to avoid needing to cache the trench zone
        // and have it recalc each time

        public static Zone TOWER() {
                return BLUETOWER.union(REDTOWER);
        } 

        public static Zone AIM() {
                return nearAlliance().difference(TRENCH()).difference(TOWER());
        }

        public static Zone PASS() {
                return NEUTRAL
                        .union(opposingAlliance())
                        .difference(TRENCH())
                        .difference(TOWER());
        }

        public static Zone LEFTPASS() {
                double splitY = FieldConstants.LinesHorizontal.center;
                if (AllianceFlipUtil.shouldFlip()) {
                        return PASS().intersection(new Zone.RectangleZone(
                                0.0, FieldConstants.fieldLength,
                                0.0, splitY));  // Red: low Y half
                } else {
                        return PASS().intersection(new Zone.RectangleZone(
                                0.0, FieldConstants.fieldLength,
                                splitY, FieldConstants.fieldWidth));  // Blue: high Y half
                }
        }

        public static Zone RIGHTPASS() {
                double splitY = FieldConstants.LinesHorizontal.center;
                if (AllianceFlipUtil.shouldFlip()) {
                        return PASS().intersection(new Zone.RectangleZone(
                                0.0, FieldConstants.fieldLength,
                                splitY, FieldConstants.fieldWidth));  // Red: high Y half
                } else {
                        return PASS().intersection(new Zone.RectangleZone(
                                0.0, FieldConstants.fieldLength,
                                0.0, splitY));  // Blue: low Y half
                }
        }

        public static void updateTrenchMargin(double newMargin) {
                trenchXMargin = newMargin;
                trenchZone = buildTrenchZone();
        }

        public static boolean inAIM(Translation2d position) {
                return AIM().contains(position);
        }

        public static boolean inPASS(Translation2d position) {
                return PASS().contains(position);
        }

        public static boolean inLEFTPASS(Translation2d position) {
                return LEFTPASS().contains(position);
        }

        public static boolean inRIGHTPASS(Translation2d position) {
                return RIGHTPASS().contains(position);
        }

        public static boolean inTRENCH(Translation2d position) {
                return TRENCH().contains(position);
        }

        public static boolean inNearAlliance(Translation2d position) {
                return nearAlliance().contains(position);
        }

        public static boolean inOpposingAlliance(Translation2d position) {
                return opposingAlliance().contains(position);
        }

        public static boolean inNeutral(Translation2d position) {
                return NEUTRAL.contains(position);
        }

        public static boolean inTower(Translation2d position) {
                return TOWER().contains(position);
        }
}
