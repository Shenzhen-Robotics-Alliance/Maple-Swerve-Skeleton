package frc.robot.subsystems.led;

import edu.wpi.first.wpilibj.util.Color;
import java.util.Arrays;

public sealed interface LEDAnimation {
    /**
     * Plays one frame the animation
     *
     * @param colors an array containing a series of {@link Color} that this function will modify
     * @param t the time of the frame. 0~1 represents a full animation cycle
     */
    void play(Color[] colors, double t);

    static double oscillate(double t) {
        return 0.5 + 0.5 * Math.sin(2 * Math.PI * t);
    }

    final class Breathe implements LEDAnimation {
        private final Color color;

        public Breathe(Color color) {
            this.color = color;
        }

        @Override
        public void play(Color[] colors, double t) {
            final double brightness = oscillate(t + 0.25);
            for (int i = 0; i < colors.length; i++)
                colors[i] = new Color(color.red * brightness, color.green * brightness, color.blue * brightness);
        }
    }

    final class ShowColor implements LEDAnimation {
        private final Color color;

        public ShowColor(Color color) {
            this.color = color;
        }

        @Override
        public void play(Color[] colors, double t) {
            Arrays.fill(colors, color);
        }
    }

    final class SlideBackAndForth extends Slide {
        public SlideBackAndForth(Color color) {
            super(color);
        }

        @Override
        public void play(Color[] colors, double t) {
            super.play(colors, oscillate(t));
        }
    }

    sealed class Slide implements LEDAnimation {
        private final Color color;

        public Slide(Color color) {
            this.color = color;
        }

        @Override
        public void play(Color[] colors, double t) {
            if (t < 0.5) playSlideForward(colors, t * 2);
            else playSlideBackwards(colors, (t - 0.5) * 2);
        }

        private void playSlideForward(Color[] colors, double t) {
            for (int i = 0; i < colors.length; i++) {
                colors[i] = i < t * colors.length ? color : new Color();
            }
        }

        private void playSlideBackwards(Color[] colors, double t) {
            for (int i = 0; i < colors.length; i++) {
                colors[i] = i > t * colors.length ? color : new Color();
            }
        }
    }

    final class Charging implements LEDAnimation {
        private final Color color;

        public Charging(Color color) {
            this.color = color;
        }

        @Override
        public void play(Color[] colors, double t) {
            for (int i = 0; i < colors.length; i++) {
                colors[i] = i < t * colors.length
                        ? new Color(color.red * t + 0.2, color.green * t + 0.2, color.blue * t + 0.2)
                        : new Color();
            }
        }
    }

    final class Rainbow implements LEDAnimation {
        @Override
        public void play(Color[] colors, double t) {
            final int firstPixelHue = (int) (t * 180), v = 128;
            for (var i = 0; i < colors.length; i++) {
                int colorH = (firstPixelHue + (i * 180 / colors.length)) % 180;
                colors[i] = Color.fromHSV(colorH, 255, 128);
            }
        }
    }
}
