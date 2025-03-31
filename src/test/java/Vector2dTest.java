import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import net.teamrush27.frc2024.util.Vector2d;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;
import static org.junit.jupiter.api.Assertions.assertEquals;

class Vector2dTest {
    private Pose2d orgin;
    private Pose2d end;

    @BeforeEach
    void setup() {
    }

    @Test
    void Test1(){
        orgin = new Pose2d(new Translation2d(5, 5), new Rotation2d(0));
        end = new Pose2d(new Translation2d(10, 10), new Rotation2d(0));
        Vector2d originToEnd = new Vector2d(orgin, end);

        System.out.println("Translation2d object " + originToEnd.getTranslation2d().toString());
        System.out.println("Distance " + originToEnd.getDistance());
        System.out.println("Angle " + originToEnd.getRotation().getDegrees());
        assertEquals(originToEnd.getRotation().getDegrees(), 45, 0.01);
    }

    @Test
    void Test2(){
        orgin = new Pose2d(new Translation2d(10, 10), new Rotation2d(0));
        end = new Pose2d(new Translation2d(13, 14), new Rotation2d(0));
        Vector2d originToEnd = new Vector2d(orgin, end);

        System.out.println("Translation2d object " + originToEnd.getTranslation2d().toString());
        System.out.println("Distance " + originToEnd.getDistance());
        System.out.println("Angle " + originToEnd.getRotation().getDegrees());
        assertEquals(originToEnd.getDistance(), 5, 0.01);
    }

    @Test
    void Test3(){
        orgin = new Pose2d(new Translation2d(10, 10), new Rotation2d(Math.PI));
        end = new Pose2d(new Translation2d(13, 14), new Rotation2d(0));
        Vector2d originToEnd = new Vector2d(orgin, end);

        System.out.println("Translation2d object " + originToEnd.getTranslation2d().toString());
        System.out.println("Distance " + originToEnd.getDistance());
        System.out.println("Angle " + originToEnd.getRotation().getDegrees());
        assertEquals(originToEnd.getDistance(), 5, 0.01);
        assertEquals(originToEnd.getRotation().getDegrees(), 53.13010235415599, 0.01);
    }

}