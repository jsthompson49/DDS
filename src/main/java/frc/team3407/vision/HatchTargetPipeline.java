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

    public static void startVisionThread(VideoSource videoSource) {
        new VisionThread(videoSource, new HatchTargetPipeline(videoSource.getVideoMode().width), pipeline -> pipeline.setTargetData()).start();
    }

    private HatchTargetPipeline(int width) {
        this.width = width;
    }

    @Override
    public void process(Mat image) {
        long start = System.currentTimeMillis();
        List<HatchTarget> hatchTargets = targetRecognizer.find(image, (rr,idx) -> {});

        double targetOffset = -9999.1111;
        int hit = 0;
        if (hatchTargets != null) {
            // Get the closest one
            HatchTarget hatchTarget = hatchTargets.stream().min(new HatchTargetComparator()).get();
            targetOffset = hatchTarget.getOffset(width);
            hit = 1;
        }

        long end = System.currentTimeMillis();
        synchronized (this) {
            offset = targetOffset;
            count++;
            duration += (end - start);
            hits += hit;
        }
    }

    public synchronized void setTargetData() {
        networkTableTargetData.update(offset, hits, count, duration);
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
