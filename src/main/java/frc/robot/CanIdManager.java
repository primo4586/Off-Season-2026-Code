package frc.robot;

public class CanIdManager {
    public static final int IDS_PER_ROBOT = 10; // must be >= motors per robot

    // 6 robots * 10 = IDs 1..60, which stays within 0-62
    public static int get(int robotId, int localIndex) {
        if (localIndex >= IDS_PER_ROBOT)
            throw new IllegalArgumentException("localIndex too large");
        return 1 + robotId * IDS_PER_ROBOT + localIndex;
    }

}
