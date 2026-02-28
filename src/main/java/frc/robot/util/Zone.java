//Based off of FRC 4481's explanation of zones: 
package frc.robot.util;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.util.geometry.AllianceFlipUtil;

import java.util.function.Supplier;

/**
 * A Zone represents an area on the field that determines if a Translation2d lies of inside it.
 */
public interface Zone {

    boolean contains(Translation2d point);

    /**
     * Convenience wrapper for WPILib Triggers.
     */
    default Trigger contains(Supplier<Translation2d> supplier) {
        return new Trigger(() -> contains(supplier.get()));
    }

    // Set operations
    default Zone union(Zone other) {
        return new CompositeZone(this, other, SetOperation.UNION);
    }

    default Zone intersection(Zone other) {
        return new CompositeZone(this, other, SetOperation.INTERSECTION);
    }

    default Zone difference(Zone other) {
        return new CompositeZone(this, other, SetOperation.DIFFERENCE);
    }

    default Zone complement() {
        return new CompositeZone(this, null, SetOperation.COMPLEMENT);
    }

    // Primitive Implementations

    /**
     * Circular zone defined by center and radius.
     */
    public class CircleZone implements Zone {
        private final Translation2d center;
        private final double radius;

        public CircleZone(Translation2d center, double radius) {
            this.center = center;
            this.radius = radius;
        }

        @Override
        public boolean contains(Translation2d point) {
            return point.getDistance(center) <= radius;
        }
    }

    /**
     * Axis-aligned rectangle zone.
     */
    public class RectangleZone implements Zone {
        private final double minX, maxX;
        private final double minY, maxY;

        public RectangleZone(double minX, double maxX, double minY, double maxY) {
            this.minX = minX;
            this.maxX = maxX;
            this.minY = minY;
            this.maxY = maxY;
        }

        @Override
        public boolean contains(Translation2d point) {
            double x = point.getX();
            double y = point.getY();
            return x >= minX && x <= maxX && y >= minY && y <= maxY;
        }
    }

    public enum SetOperation {
        UNION,
        INTERSECTION,
        DIFFERENCE,
        COMPLEMENT
    }

    public class CompositeZone implements Zone {
        private final Zone a;
        private final Zone b;
        private final SetOperation operation;

        public CompositeZone(Zone a, Zone b, SetOperation operation) {
            this.a = a;
            this.b = b;
            this.operation = operation;
        }

        @Override
        public boolean contains(Translation2d point) {
            switch (operation) {
                case UNION:
                    return a.contains(point) || b.contains(point);
                case INTERSECTION:
                    return a.contains(point) && b.contains(point);
                case DIFFERENCE:
                    return a.contains(point) && !b.contains(point);
                case COMPLEMENT:
                    return !a.contains(point);
                default:
                    return false;
            }
        }
    }
}