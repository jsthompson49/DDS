package frc.team3407.vision;

import edu.wpi.cscore.VideoSource;
import edu.wpi.first.vision.VisionPipeline;
import edu.wpi.first.vision.VisionThread;
import org.opencv.core.Mat;

import java.util.Comparator;
import java.util.List;

public class HatchTargetPipeline implements VisionPipeline {

    private final HatchTargetRecognizer targetRecognizer = new HatchTargetRecognizer();
    private final NetworkTableTargetData networkTableTargetData = new NetworkTableTargetData();
    private final int width;

    private double offset;

    private int hits = 0;
    private long count = 0;
    private long duration = 0;
    private int staleCount = 0;
    private long lastSnapshotTime = 0;

    private static final int MAX_STALE_COUNT = 5;
    private static final long SNAPSHOT_INTERVAL_MILLISECONDS = 250;

    public static void startVisionThread(VideoSource videoSource) {
        System.out.println("Starting vision thread");
        new VisionThread(videoSource, new HatchTargetPipeline(videoSource.getVideoMode().width), pipeline -> pipeline.setTargetData()).start();
    }

    private HatchTargetPipeline(int width) {
        this.width = width;
    }

    @Override
    public void process(Mat image) {
        long start = System.currentTimeMillis();
        if ((start - lastSnapshotTime) < SNAPSHOT_INTERVAL_MILLISECONDS) {
            return;
        }

        List<HatchTarget> hatchTargets = targetRecognizer.find(image, (rr,idx) -> {});

        int hitCount = (hatchTargets == null) ? 0 : hatchTargets.size();
        int hit = 0;
        HatchTarget hatchTarget = null;
        if (hitCount == 1) {
            hatchTarget = hatchTargets.get(0);
        } else if (hitCount > 1) {
            hatchTarget = hatchTargets.stream().min(new HatchTargetComparator()).get();
        }

        double targetOffset = width;
        if (hatchTarget != null) {
            targetOffset = hatchTarget.getOffset(width);
            hit = 1;
        }

        System.out.println("Processing image: hits=" + hitCount);
        long end = System.currentTimeMillis();
        synchronized (this) {
            count++;
            duration += (end - start);
            hits += hit;
            if (hit == 0) {
                staleCount++;
                if (staleCount == MAX_STALE_COUNT) {
                    offset = width;
                    staleCount = 0;
                }
            } else {
                staleCount = 0;
                offset = targetOffset;
            }
        }
    }

    public synchronized void setTargetData() {
        networkTableTargetData.update(offset, staleCount, hits, count, duration);
    }

    private class HatchTargetComparator implements Comparator<HatchTarget> {
        @Override
        public int compare(HatchTarget o1, HatchTarget o2) {
            double offset1 = o1.getOffset(width);
            double offset2 = o2.getOffset(width);

            return (offset1 == offset2) ? 0 : ((offset1 < offset2) ? -1 : 1);
        }
    }
}
