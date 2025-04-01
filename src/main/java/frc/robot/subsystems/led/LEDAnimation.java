package frc.robot.subsystems.led;

import edu.wpi.first.wpilibj.util.Color;
import java.util.Arrays;
import java.util.function.Supplier;

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
        private final Supplier<Color> color;

        public Breathe(Supplier<Color> color) {
            this.color = color;
        }

        @Override
        public void play(Color[] colors, double t) {
            final double brightness = oscillate(t + 0.25);
            for (int i = 0; i < colors.length; i++)
                colors[i] = new Color(
                        color.get().red * brightness, color.get().green * brightness, color.get().blue * brightness);
        }
    }

    final class ShowColor implements LEDAnimation {
        private final Supplier<Color> color;

        public ShowColor(Supplier<Color> color) {
            this.color = color;
        }

        @Override
        public void play(Color[] colors, double t) {
            Arrays.fill(colors, color.get());
        }
    }

    final class SlideBackAndForth extends Slide {
        public SlideBackAndForth(Supplier<Color> color) {
            super(color);
        }

        @Override
        public void play(Color[] colors, double t) {
            super.play(colors, oscillate(t));
        }
    }

    sealed class Slide implements LEDAnimation {
        private final Supplier<Color> color;

        public Slide(Supplier<Color> color) {
            this.color = color;
        }

        @Override
        public void play(Color[] colors, double t) {
            if (t < 0.5) playSlideForward(colors, t * 2);
            else playSlideBackwards(colors, (t - 0.5) * 2);
        }

        private void playSlideForward(Color[] colors, double t) {
            for (int i = 0; i < colors.length; i++) {
                colors[i] = i < t * colors.length ? color.get() : new Color();
            }
        }

        private void playSlideBackwards(Color[] colors, double t) {
            for (int i = 0; i < colors.length; i++) {
                colors[i] = i > t * colors.length ? color.get() : new Color();
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
                double brightness = 0.5 + 0.5 * t;
                colors[i] = i < t * colors.length
                        ? new Color(color.red * brightness, color.green * brightness, color.blue * brightness)
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
