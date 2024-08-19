package frc.robot.utils;

import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import frc.robot.utils.CustomMaths.MapleCommonMath;

/**
 * interface for an LED animation that can be displayed on the dashboard or by an addressable led
 * */
public interface LEDAnimation {
    void play(AddressableLEDBuffer buffer, double t);


    final class Breathe implements LEDAnimation {
        private final int colorR, colorG, colorB;
        private final double hz;
        public Breathe(int colorR, int colorG, int colorB, double hz) {
            this.colorR = colorR;
            this.colorG = colorG;
            this.colorB = colorB;
            this.hz = hz;
        }

        @Override
        public void play(AddressableLEDBuffer buffer, double t) {
            t *= hz;
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
            for (int i = 0; i < buffer.getLength(); i++)
                buffer.setRGB(i, colorR, colorG, colorB);
        }
    }

    final class SlideBackAndForth extends Slide {
        private final double hz1;
        public SlideBackAndForth(int colorR, int colorG, int colorB, double hz, double slideLength) {
            super(colorR, colorG, colorB, 1, slideLength);
            this.hz1 = hz;
        }

        @Override
        public void play(AddressableLEDBuffer buffer, double t) {
            super.play(buffer, 0.5 + 0.5 * Math.sin(t * hz1));
        }
    }

    class Slide implements LEDAnimation {
        private final int colorR, colorG, colorB;
        private final double hz, slideLength;
        public Slide(int colorR, int colorG, int colorB, double hz, double slideLength) {
            this.colorR = colorR;
            this.colorG = colorG;
            this.colorB = colorB;
            this.hz = hz;
            this.slideLength = slideLength;
        }

        @Override
        public void play(AddressableLEDBuffer buffer, double t) {
            t *= hz;
            t %= 1;
            final double
                    lowerEdge = MapleCommonMath.linearInterpretation(0, -slideLength, 1, 1, t),
                    upperEdge = lowerEdge + slideLength;
            /* strip is half the entire led */
            final int stripLength = buffer.getLength() / 2;
            for (int i = 0; i < stripLength; i++) {
                int r = colorR, g = colorG, b = colorB;
                if (i >= (int)(upperEdge * stripLength) || i <= (int)(lowerEdge * stripLength)) r = g = b = 0;
                buffer.setRGB(stripLength + i, r, g, b);
                buffer.setRGB(stripLength - i-1, r, g, b);
            }
        }
    }

    final class Charging implements LEDAnimation {
        private final int colorR, colorG, colorB;
        private final double hz;
        public Charging(int colorR, int colorG, int colorB, double hz) {
            this.colorR = colorR;
            this.colorG = colorG;
            this.colorB = colorB;
            this.hz = hz;
        }

        @Override
        public void play(AddressableLEDBuffer buffer, double t) {
            t *= hz;
            t %= 1;
            final int edge = (int) (t * buffer.getLength() / 2);
            final double coolDownTime = 0.5 * hz;

            t *= 1+coolDownTime;
            for (int i = 0; i < buffer.getLength() / 2; i++) {
                int r = colorR, g = colorG, b = colorB;
                if (t > 1) {
                    double brightness = (1+coolDownTime - t) / coolDownTime;
                    r = (int) (r*brightness);
                    g = (int) (g*brightness);
                    b = (int) (b*brightness);
                } else if (i > edge)
                    r = g = b = 0;
                buffer.setRGB(buffer.getLength() /2 + 1 - i, r, g, b);
                buffer.setRGB(buffer.getLength() /2 - 1 + i, r, g, b);
            }
        }
    }

    class ChargingDualColor implements LEDAnimation {
        private final int fromColorR, fromColorG, fromColorB, toColorR, toColorG, toColorB;
        private final double duration;

        public ChargingDualColor(int fromColorR, int fromColorG, int fromColorB, int toColorR, int toColorG, int toColorB, double duration) {
            this.fromColorR = fromColorR;
            this.fromColorG = fromColorG;
            this.fromColorB = fromColorB;
            this.toColorR = toColorR;
            this.toColorG = toColorG;
            this.toColorB = toColorB;
            this.duration = duration;
        }

        @Override
        public void play(AddressableLEDBuffer buffer, double t) {
            t = Math.min(Math.max(t/duration, 0), 1);

            // TODO charging animation
        }
    }

    final class Rainbow implements LEDAnimation {
        private final double hz;
        public Rainbow(double hz) {
            this.hz = hz;
        }

        @Override
        public void play(AddressableLEDBuffer buffer, double t) {
            t*= hz;
            final int firstPixelHue = (int) (t * 180),
                    v = 128;
            for(var i = 0; i < buffer.getLength()/2; i++) {
                final int colorH = (firstPixelHue + (i * 180 / buffer.getLength())) % 180;
                buffer.setHSV(buffer.getLength()/2 + i, colorH, 255, v);
                buffer.setHSV(buffer.getLength()/2 - i, colorH, 255, v);
            }
        }
    }

    final class PoliceLight implements LEDAnimation {
        private final double hz;
        public PoliceLight(double hz) {
            this.hz = hz;
        }

        @Override
        public void play(AddressableLEDBuffer buffer, double t) {
            t*= hz;
            for (int i = 0; i < buffer.getLength() / 2; i++) {
                final int blink = t > 0.5? 255 : 0;
                buffer.setRGB(i, t > 0.5? 255 : 0, 0, 0);
                buffer.setRGB(buffer.getLength() - i-1, 0, 0,  t < 0.5? 255 : 0);
            }
        }
    }
}