import java.io.IOException;
import java.util.concurrent.TimeUnit;

import com.pi4j.io.i2c.I2CBus;
import com.pi4j.io.i2c.I2CDevice;
import com.pi4j.io.i2c.I2CFactory;

public class AdafruitPWMServoDriver {

	// Registers/etc.
	private final int SUBADR1 = 0x02;
	private final int SUBADR2 = 0x03;
	private final int SUBADR3 = 0x04;
	private final int MODE1 = 0x00;
	private final int PRESCALE = 0xFE;
	private final int LED0_ON_L = 0x06;
	private final int LED0_ON_H = 0x07;
	private final int LED0_OFF_L = 0x08;
	private final int LED0_OFF_H = 0x09;
	private final int ALLLED_ON_L = 0xFA;
	private final int ALLLED_ON_H = 0xFB;
	private final int ALLLED_OFF_L = 0xFC;
	private final int ALLLED_OFF_H = 0xFD;

	private I2CDevice pwmServoDriver;

	public AdafruitPWMServoDriver(int address) {
		try {
			final I2CBus bus = I2CFactory.getInstance(I2CBus.BUS_1);
			pwmServoDriver = bus.getDevice(address);
			pwmServoDriver.write(MODE1, (byte) 0x00);
		} catch (Exception e) {
			e.printStackTrace();
		}
	}

	public void setPWMFreq(int freq) throws IOException, InterruptedException {
		// "Sets the PWM frequency"
		double prescaleval = 25000000.0; // 25MHz
		prescaleval /= 4096.0; // 12-bit
		prescaleval /= freq;
		prescaleval -= 1.0;
		double prescale = Math.floor(prescaleval + 0.5);

		int oldmode = pwmServoDriver.read(MODE1);
		int newmode = (oldmode & 0x7F) | 0x10; // sleep
		pwmServoDriver.write(MODE1, (byte) newmode); // go to sleep
		pwmServoDriver.write(PRESCALE, (byte) (Math.floor(prescale)));
		pwmServoDriver.write(MODE1, (byte) oldmode);
		TimeUnit.MILLISECONDS.sleep(5);
		pwmServoDriver.write(MODE1, (byte) (oldmode | 0x80));
	}

	private synchronized void setPWM(int channel, int on, int off) throws IOException {
		// "Sets a single PWM channel"
		pwmServoDriver.write(LED0_ON_L + 4 * channel, (byte) (on & 0xFF));
		pwmServoDriver.write(LED0_ON_H + 4 * channel, (byte) (on >> 8));
		pwmServoDriver.write(LED0_OFF_L + 4 * channel, (byte) (off & 0xFF));
		pwmServoDriver.write(LED0_OFF_H + 4 * channel, (byte) (off >> 8));
	}

	public void setServoPulse(int channel, int pulse) throws IOException {
		double pulseLength = 1000000.0; // 1,000,000 us per second
		pulseLength /= 60; // 60 Hz
		// print "%d us per period" % pulseLength
		pulseLength /= 4096; // 12 bits of resolution
		// print "%d us per bit" % pulseLength
		// pulse *= 1000;
		pulse /= pulseLength;
		setPWM(channel, 0, pulse);
	}
}
