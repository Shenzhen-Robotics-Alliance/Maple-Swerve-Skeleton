package frc.robot.subsystems.led;

import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import org.ironmaple.utils.mathutils.MapleCommonMath;

public sealed interface LEDAnimation {
    /**
     * Plays one frame the animation
     *
     * @param buffer the {@link AddressableLEDBuffer} to play on
     * @param t the time of the frame. 0~1 represents a full animation cycle
     */
    void play(AddressableLEDBuffer buffer, double t);

    final class Breathe implements LEDAnimation {
        private final int colorR, colorG, colorB;

        public Breathe(int colorR, int colorG, int colorB) {
            this.colorR = colorR;
            this.colorG = colorG;
            this.colorB = colorB;
        }

        @Override
        public void play(AddressableLEDBuffer buffer, double t) {
            final double brightness = 0.5 + 0.5 * Math.sin(t * Math.PI);
            for (int i = 0; i < buffer.getLength(); i++)
                buffer.setRGB(i, (int) (colorR * brightness), (int) (colorG * brightness), (int) (colorB * brightness));
        }
    }

    final class ShowColor implements LEDAnimation {
        private final int colorR, colorG, colorB;

        public ShowColor(int colorR, int colorG, int colorB) {
            this.colorR = colorR;
            this.colorG = colorG;
            this.colorB = colorB;
        }

        @Override
        public void play(AddressableLEDBuffer buffer, double t) {
            for (int i = 0; i < buffer.getLength(); i++) buffer.setRGB(i, colorR, colorG, colorB);
        }
    }

    final class SlideBackAndForth extends Slide {
        public SlideBackAndForth(int colorR, int colorG, int colorB, double slideLength) {
            super(colorR, colorG, colorB, slideLength);
        }

        @Override
        public void play(AddressableLEDBuffer buffer, double t) {
            super.play(buffer, 0.5 + 0.5 * Math.sin(t * Math.PI));
        }
    }

    sealed class Slide implements LEDAnimation {
        private final int colorR, colorG, colorB;
        private final double slideLength;

        public Slide(int colorR, int colorG, int colorB, double slideLength) {
            this.colorR = colorR;
            this.colorG = colorG;
            this.colorB = colorB;
            this.slideLength = slideLength;
        }

        @Override
        public void play(AddressableLEDBuffer buffer, double t) {
            int halfLength = buffer.getLength() / 2;
            double lowerEdge = MapleCommonMath.linearInterpretation(0, -slideLength, 1, 1, t);
            double upperEdge = lowerEdge + slideLength;
            /* strip is half the entire led */
            for (int i = 0; i < halfLength; i++) {
                int r = colorR, g = colorG, b = colorB;
                if (i >= (int) (upperEdge * halfLength) || i <= (int) (lowerEdge * halfLength)) r = g = b = 0;
                buffer.setRGB(halfLength + i, r, g, b);
                buffer.setRGB(halfLength - i - 1, r, g, b);
            }
        }
    }

    final class Charging implements LEDAnimation {
        private final int colorR, colorG, colorB;

        public Charging(int colorR, int colorG, int colorB) {
            this.colorR = colorR;
            this.colorG = colorG;
            this.colorB = colorB;
        }

        @Override
        public void play(AddressableLEDBuffer buffer, double t) {
            final int edge = (int) (t * buffer.getLength() / 2);
            final double coolDownTime = 0.2;

            t *= 1 + coolDownTime;
            for (int i = 0; i < buffer.getLength() / 2; i++) {
                int r = colorR, g = colorG, b = colorB;
                if (t > 1) {
                    double brightness = (1 + coolDownTime - t) / coolDownTime;
                    r = (int) (r * brightness);
                    g = (int) (g * brightness);
                    b = (int) (b * brightness);
                } else if (i > edge) r = g = b = 0;
                buffer.setRGB(buffer.getLength() / 2 + 1 - i, r, g, b);
                buffer.setRGB(buffer.getLength() / 2 - 1 + i, r, g, b);
            }
        }
    }

    final class Rainbow implements LEDAnimation {
        @Override
        public void play(AddressableLEDBuffer buffer, double t) {
            final int firstPixelHue = (int) (t * 180), v = 128;
            for (var i = 0; i < buffer.getLength() / 2; i++) {
                final int colorH = (firstPixelHue + (i * 180 / buffer.getLength())) % 180;
                buffer.setHSV(buffer.getLength() / 2 + i, colorH, 255, v);
                buffer.setHSV(buffer.getLength() / 2 - i, colorH, 255, v);
            }
        }
    }
}
