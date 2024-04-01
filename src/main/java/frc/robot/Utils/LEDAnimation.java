package frc.robot.Utils;

import edu.wpi.first.wpilibj.AddressableLEDBuffer;

public interface LEDAnimation {
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
            final double brightness = Math.sin(t * Math.PI);
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

    final class Slide implements LEDAnimation {
        private final int colorR, colorG, colorB;
        public Slide(int colorR, int colorG, int colorB) {
            this.colorR = colorR;
            this.colorG = colorG;
            this.colorB = colorB;
        }

        @Override
        public void play(AddressableLEDBuffer buffer, double t) {
            final double slideLength = 0.8;

            t *= 2;
            final int
                    lowerEdge = t < 1 ?
                    (int) ((2*t-1) * buffer.getLength() / 2)
                    :(int) ((1-2*(t-1)) * buffer.getLength() / 2),
                    upperEdge = lowerEdge + (int)(slideLength * buffer.getLength() / 2);
            for (int i = 0; i < buffer.getLength() / 2; i++) {
                int r = colorR, g = colorG, b = colorB;
                if (i > upperEdge || i < lowerEdge) r = g = b = 30;
                buffer.setRGB(buffer.getLength() / 2 + i, r, g, b);
                buffer.setRGB(buffer.getLength() / 2 - i-1, r, g, b);
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
            final double coolDownTime = 0.3;

            t *= 1+coolDownTime;
            for (int i = 0; i < buffer.getLength() / 2; i++) {
                int r = colorR, g = colorG, b = colorB;
                if (t > 1) {
                    double brightness = (1+coolDownTime - t) / coolDownTime;
                    r = (int) (r*brightness);
                    g = (int) (g*brightness);
                    b = (int) (b*brightness);
                } else if (i > edge) r = g = b = 0;
                buffer.setRGB(i, r, g, b);
                buffer.setRGB(buffer.getLength() - i-1, r, g, b);
            }
        }
    }

    final class Rainbow implements LEDAnimation {
        @Override
        public void play(AddressableLEDBuffer buffer, double t) {
            final int firstPixelHue = (int) (t * 180),
                    v = (int) (32 + 96 * Math.sin(Math.PI*t));
            for (var i = 0; i < buffer.getLength(); i++)
                buffer.setHSV(i, (firstPixelHue + (i * 180 / buffer.getLength())) % 180, 255, v);
        }
    }

    final class PoliceLight implements LEDAnimation {

        @Override
        public void play(AddressableLEDBuffer buffer, double t) {
            for (int i = 0; i < buffer.getLength() / 2; i++) {
                final int blink = t > 0.5? 255 : 0;
                buffer.setRGB(i, t > 0.5? 255 : 0, 0, 0);
                buffer.setRGB(buffer.getLength() - i-1, 0, 0,  t < 0.5? 255 : 0);
            }
        }
    }
}
