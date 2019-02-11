package frc.team3407.vision;

import org.opencv.core.RotatedRect;

public class HatchTarget {
    private RotatedRect left;
    private RotatedRect right;

    public HatchTarget(RotatedRect rr1, RotatedRect rr2) {
        if (rr1.center.x < rr2.center.x) {
            left = rr1;
            right = rr2;
        } else {
            left = rr2;
            right = rr1;
        }
    }

    public RotatedRect getLeft() {
        return left;
    }

    public RotatedRect getRight() {
        return right;
    }

    public double getMidPoint() {
        return (left.center.x + right.center.x) / 2;
    }

    public double getOffset(double range) {
        double midpoint = range / 2;
        return getMidPoint() - midpoint;
    }
}
