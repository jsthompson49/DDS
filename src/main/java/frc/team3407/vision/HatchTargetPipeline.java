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
        List<HatchTargetRecognizer.HatchTarget> hatchTargets = targetRecognizer.find(image,
                (rr,idx) -> {});

        double targetOffset = -9999.1111;
        if (hatchTargets != null) {
            // Get the closest one
            HatchTargetRecognizer.HatchTarget hatchTarget = hatchTargets.stream().min(new HatchTargetComparator()).get();
            targetOffset = hatchTarget.getOffset(width);
        }

        long end = System.currentTimeMillis();
        synchronized (this) {
            offset = targetOffset;
            count++;
            duration += (end - start);
        }
    }

    public synchronized void setTargetData() {
        networkTableTargetData.update(offset, count, duration);
    }

    private class HatchTargetComparator implements Comparator<HatchTargetRecognizer.HatchTarget> {
        @Override
        public int compare(HatchTargetRecognizer.HatchTarget o1, HatchTargetRecognizer.HatchTarget o2) {
            double offset1 = o1.getOffset(width);
            double offset2 = o2.getOffset(width);

            return (offset1 == offset2) ? 0 : ((offset1 < offset2) ? -1 : 1);
        }
    }
}
