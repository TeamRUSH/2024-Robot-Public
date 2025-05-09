package net.teamrush27.frc2024.util;

import com.fasterxml.jackson.core.JsonProcessingException;
import com.fasterxml.jackson.databind.ObjectMapper;
import com.fasterxml.jackson.databind.ObjectReader;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.Pose3d;

public class AprilTag2024Field {

    static AprilTag2024Field instance;

    private AprilTagFieldLayout layout;
    private static final String TAGS_JSON = "{\"tags\":[{\"ID\":1,\"pose\":{\"translation\":{\"x\":15.079471999999997,\"y\":0.24587199999999998,\"z\":1.355852},\"rotation\":{\"quaternion\":{\"W\":0.5000000000000001,\"X\":0.0,\"Y\":0.0,\"Z\":0.8660254037844386}}}},{\"ID\":2,\"pose\":{\"translation\":{\"x\":16.185134,\"y\":0.883666,\"z\":1.355852},\"rotation\":{\"quaternion\":{\"W\":0.5000000000000001,\"X\":0.0,\"Y\":0.0,\"Z\":0.8660254037844386}}}},{\"ID\":3,\"pose\":{\"translation\":{\"x\":16.579342,\"y\":4.982717999999999,\"z\":1.4511020000000001},\"rotation\":{\"quaternion\":{\"W\":6.123233995736766e-17,\"X\":0.0,\"Y\":0.0,\"Z\":1.0}}}},{\"ID\":4,\"pose\":{\"translation\":{\"x\":16.579342,\"y\":5.547867999999999,\"z\":1.4511020000000001},\"rotation\":{\"quaternion\":{\"W\":6.123233995736766e-17,\"X\":0.0,\"Y\":0.0,\"Z\":1.0}}}},{\"ID\":5,\"pose\":{\"translation\":{\"x\":14.700757999999999,\"y\":8.2042,\"z\":1.355852},\"rotation\":{\"quaternion\":{\"W\":-0.7071067811865475,\"X\":-0.0,\"Y\":0.0,\"Z\":0.7071067811865476}}}},{\"ID\":6,\"pose\":{\"translation\":{\"x\":1.8415,\"y\":8.2042,\"z\":1.355852},\"rotation\":{\"quaternion\":{\"W\":-0.7071067811865475,\"X\":-0.0,\"Y\":0.0,\"Z\":0.7071067811865476}}}},{\"ID\":7,\"pose\":{\"translation\":{\"x\":-0.038099999999999995,\"y\":5.547867999999999,\"z\":1.4511020000000001},\"rotation\":{\"quaternion\":{\"W\":1.0,\"X\":0.0,\"Y\":0.0,\"Z\":0.0}}}},{\"ID\":8,\"pose\":{\"translation\":{\"x\":-0.038099999999999995,\"y\":4.982717999999999,\"z\":1.4511020000000001},\"rotation\":{\"quaternion\":{\"W\":1.0,\"X\":0.0,\"Y\":0.0,\"Z\":0.0}}}},{\"ID\":9,\"pose\":{\"translation\":{\"x\":0.356108,\"y\":0.883666,\"z\":1.355852},\"rotation\":{\"quaternion\":{\"W\":0.8660254037844387,\"X\":0.0,\"Y\":0.0,\"Z\":0.49999999999999994}}}},{\"ID\":10,\"pose\":{\"translation\":{\"x\":1.4615159999999998,\"y\":0.24587199999999998,\"z\":1.355852},\"rotation\":{\"quaternion\":{\"W\":0.8660254037844387,\"X\":0.0,\"Y\":0.0,\"Z\":0.49999999999999994}}}},{\"ID\":11,\"pose\":{\"translation\":{\"x\":11.904726,\"y\":3.7132259999999997,\"z\":1.3208},\"rotation\":{\"quaternion\":{\"W\":-0.8660254037844387,\"X\":-0.0,\"Y\":0.0,\"Z\":0.49999999999999994}}}},{\"ID\":12,\"pose\":{\"translation\":{\"x\":11.904726,\"y\":4.49834,\"z\":1.3208},\"rotation\":{\"quaternion\":{\"W\":0.8660254037844387,\"X\":0.0,\"Y\":0.0,\"Z\":0.49999999999999994}}}},{\"ID\":13,\"pose\":{\"translation\":{\"x\":11.220196,\"y\":4.105148,\"z\":1.3208},\"rotation\":{\"quaternion\":{\"W\":6.123233995736766e-17,\"X\":0.0,\"Y\":0.0,\"Z\":1.0}}}},{\"ID\":14,\"pose\":{\"translation\":{\"x\":5.320792,\"y\":4.105148,\"z\":1.3208},\"rotation\":{\"quaternion\":{\"W\":1.0,\"X\":0.0,\"Y\":0.0,\"Z\":0.0}}}},{\"ID\":15,\"pose\":{\"translation\":{\"x\":4.641342,\"y\":4.49834,\"z\":1.3208},\"rotation\":{\"quaternion\":{\"W\":0.5000000000000001,\"X\":0.0,\"Y\":0.0,\"Z\":0.8660254037844386}}}},{\"ID\":16,\"pose\":{\"translation\":{\"x\":4.641342,\"y\":3.7132259999999997,\"z\":1.3208},\"rotation\":{\"quaternion\":{\"W\":-0.4999999999999998,\"X\":-0.0,\"Y\":0.0,\"Z\":0.8660254037844387}}}}],\"field\":{\"length\":16.541,\"width\":8.211}}";
    private static final ObjectReader reader = new ObjectMapper().readerFor(AprilTagFieldLayout.class);

    public static AprilTag2024Field getInstance() {
        if (instance == null) {
            try {
                instance = new AprilTag2024Field(reader.readValue(TAGS_JSON));
            } catch (JsonProcessingException e) {
                e.printStackTrace();
            }
        }
        return instance;
    }

    private AprilTag2024Field(AprilTagFieldLayout layout) {
        this.layout = layout;
    }

    public AprilTagFieldLayout getLayout() {
        return layout;
    }

    public Pose3d getTagPose3d(int id) {
        return layout.getTagPose(id).orElseGet(() -> new Pose3d());
    }
    public double getFieldLength() {
        return layout.getFieldLength();
    }
    public double getFieldWidth() {
        return layout.getFieldWidth();
    }
}
