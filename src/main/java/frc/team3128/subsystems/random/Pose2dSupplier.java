package frc.team3128.subsystems.random;

import edu.wpi.first.math.geometry.Pose2d;

/**
 * Supplier for Pose2d
 *
 * @see Supplier
 * @since 2024 Crescendo
 */
@FunctionalInterface
public interface Pose2dSupplier {

    /**
     * Gets a result.
     *
     * @return a result
     */
    Pose2d getAsPose2d();
}

