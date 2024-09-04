package frc.robot.util;

import org.junit.jupiter.api.Test;

/**
 * Simple check on PathPlanner path
 */
public class PathCheck {

    /**
     * Load the path groups.
     * <p>
     * We have had problems with syntax errors in a path.
     */
    @Test
    public void pathGroupLoaderTest() {
        // load the paths
        //   may throw a ParseException; that error will fail this test
        PathGroupLoader.loadPathGroups();
    }
}
