import java.io.IOException;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.concurrent.TimeUnit;

import com.pi4j.io.i2c.I2CBus;
import com.pi4j.io.i2c.I2CDevice;
import com.pi4j.io.i2c.I2CFactory;

public class AdafruitL3GD20 {

	private I2CDevice device;
	private final BitOps bitOps;

	private final int REG_R_WHO_AM_I = 0x0f; // Device identification register
	private final int REG_RW_CTRL_REG1 = 0x20; // Control register 1
	private final int REG_RW_CTRL_REG2 = 0x21; // Control register 2
	private final int REG_RW_CTRL_REG3 = 0x22; // Control register 3
	private final int REG_RW_CTRL_REG4 = 0x23; // Control register 4
	private final int REG_RW_CTRL_REG5 = 0x24; // Control register 5
	private final int REG_RW_REFERENCE = 0x25; // Reference value for interrupt
												// generation
	private final int REG_R_OUT_TEMP = 0x26; // Output temperature
	private final int REG_R_STATUS_REG = 0x27; // Status register
	private final int REG_R_OUT_X_L = 0x28; // X-axis angular data rate LSB
	private final int REG_R_OUT_X_H = 0x29; // X-axis angular data rate MSB
	private final int REG_R_OUT_Y_L = 0x2a; // Y-axis angular data rate LSB
	private final int REG_R_OUT_Y_H = 0x2b; // Y-axis angular data rate MSB
	private final int REG_R_OUT_Z_L = 0x2c; // Z-axis angular data rate LSB
	private final int REG_R_OUT_Z_H = 0x2d; // Z-axis angular data rate MSB
	private final int REG_RW_FIFO_CTRL_REG = 0x2e; // Fifo control register
	private final int REG_R_FIFO_SRC_REG = 0x2f; // Fifo src register
	private final int REG_RW_INT1_CFG_REG = 0x30; // Interrupt 1 configuration
	// register
	private final int REG_R_INT1_SRC_REG = 0x31; // Interrupt source register
	private final int REG_RW_INT1_THS_XH = 0x32; // Interrupt 1 threshold level
													// X MSB
	// register
	private final int REG_RW_INT1_THS_XL = 0x33; // Interrupt 1 threshold level
													// X LSB
	// register
	private final int REG_RW_INT1_THS_YH = 0x34; // Interrupt 1 threshold level
													// Y MSB
	// register
	private final int REG_RW_INT1_THS_YL = 0x35; // Interrupt 1 threshold level
													// Y LSB
	// register
	private final int REG_RW_INT1_THS_ZH = 0x36; // Interrupt 1 threshold level
													// Z MSB
	// register
	private final int REG_RW_INT1_THS_ZL = 0x37; // Interrupt 1 threshold level
													// Z LSB
	// register
	private final int REG_RW_INT1_DURATION = 0x38; // Interrupt 1 duration
													// register

	private final int MASK_CTRL_REG1_Xen = 0x01; // X enable
	private final int MASK_CTRL_REG1_Yen = 0x02; // Y enable
	private final int MASK_CTRL_REG1_Zen = 0x04; // Z enable
	private final int MASK_CTRL_REG1_PD = 0x08; // Power-down
	private final int MASK_CTRL_REG1_BW = 0x30; // Bandwidth
	private final int MASK_CTRL_REG1_DR = 0xc0; // Output data rate
	private final int MASK_CTRL_REG2_HPCF = 0x0f; // High pass filter cutoff
													// frequency
	private final int MASK_CTRL_REG2_HPM = 0x30; // High pass filter mode
													// selection
	private final int MASK_CTRL_REG3_I2_EMPTY = 0x01; // FIFO empty interrupt on
	// DRDY/INT2
	private final int MASK_CTRL_REG3_I2_ORUN = 0x02; // FIFO overrun interrupt
														// on
	// DRDY/INT2
	private final int MASK_CTRL_REG3_I2_WTM = 0x04; // FIFO watermark interrupt
													// on
	// DRDY/INT2
	private final int MASK_CTRL_REG3_I2_DRDY = 0x08; // Date-ready on DRDY/INT2
	private final int MASK_CTRL_REG3_PP_OD = 0x10; // Push-pull / Open-drain
	private final int MASK_CTRL_REG3_H_LACTIVE = 0x20; // Interrupt active
														// configuration on INT1
	private final int MASK_CTRL_REG3_I1_BOOT = 0x40; // Boot status available on
														// INT1
	private final int MASK_CTRL_REG3_I1_Int1 = 0x80; // Interrupt enabled on
														// INT1
	private final int MASK_CTRL_REG4_SIM = 0x01; // SPI Serial interface
													// selection
	private final int MASK_CTRL_REG4_FS = 0x30; // Full scale selection
	private final int MASK_CTRL_REG4_BLE = 0x40; // Big/little endian selection
	private final int MASK_CTRL_REG4_BDU = 0x80; // Block data update
	private final int MASK_CTRL_REG5_OUT_SEL = 0x03; // Out selection
														// configuration
	private final int MASK_CTRL_REG5_INT_SEL = 0xc0; // INT1 selection
														// configuration
	private final int MASK_CTRL_REG5_HPEN = 0x10; // High-pass filter enable
	private final int MASK_CTRL_REG5_FIFO_EN = 0x40; // Fifo enable
	private final int MASK_CTRL_REG5_BOOT = 0x80; // Reboot memory content
	private final int MASK_STATUS_REG_ZYXOR = 0x80; // Z, Y, X axis overrun
	private final int MASK_STATUS_REG_ZOR = 0x40; // Z axis overrun
	private final int MASK_STATUS_REG_YOR = 0x20; // Y axis overrun
	private final int MASK_STATUS_REG_XOR = 0x10; // X axis overrun
	private final int MASK_STATUS_REG_ZYXDA = 0x08; // Z, Y, X data available
	private final int MASK_STATUS_REG_ZDA = 0x04; // Z data available
	private final int MASK_STATUS_REG_YDA = 0x02; // Y data available
	private final int MASK_STATUS_REG_XDA = 0x01; // X data available
	private final int MASK_FIFO_CTRL_REG_FM = 0xe0; // Fifo mode selection
	private final int MASK_FIFO_CTRL_REG_WTM = 0x1f; // Fifo treshold -
														// watermark
	// level
	private final int MASK_FIFO_SRC_REG_FSS = 0x1f; // Fifo stored data level
	private final int MASK_FIFO_SRC_REG_EMPTY = 0x20; // Fifo empty bit
	private final int MASK_FIFO_SRC_REG_OVRN = 0x40; // Overrun status
	private final int MASK_FIFO_SRC_REG_WTM = 0x80; // Watermark status
	private final int MASK_INT1_CFG_ANDOR = 0x80; // And/Or configuration of
													// interrupt
	// events
	private final int MASK_INT1_CFG_LIR = 0x40; // Latch interrupt request
	private final int MASK_INT1_CFG_ZHIE = 0x20; // Enable interrupt generation
													// on Z
	// high
	private final int MASK_INT1_CFG_ZLIE = 0x10; // Enable interrupt generation
													// on Z
	// low
	private final int MASK_INT1_CFG_YHIE = 0x08; // Enable interrupt generation
													// on Y
	// high
	private final int MASK_INT1_CFG_YLIE = 0x04; // Enable interrupt generation
													// on Y
	// low
	private final int MASK_INT1_CFG_XHIE = 0x02; // Enable interrupt generation
													// on X
	// high
	private final int MASK_INT1_CFG_XLIE = 0x01; // Enable interrupt generation
													// on X
	// low
	private final int MASK_INT1_SRC_IA = 0x40; // Int1 active
	private final int MASK_INT1_SRC_ZH = 0x20; // Int1 source Z high
	private final int MASK_INT1_SRC_ZL = 0x10; // Int1 source Z low
	private final int MASK_INT1_SRC_YH = 0x08; // Int1 source Y high
	private final int MASK_INT1_SRC_YL = 0x04; // Int1 source Y low
	private final int MASK_INT1_SRC_XH = 0x02; // Int1 source X high
	private final int MASK_INT1_SRC_XL = 0x01; // Int1 source X low
	private final int MASK_INT1_THS_H = 0x7f; // MSB
	private final int MASK_INT1_THS_L = 0xff; // LSB
	private final int MASK_INT1_DURATION_WAIT = 0x80; // Wait number of samples
														// or not
	private final int MASK_INT1_DURATION_D = 0x7f; // Duration of int1 to be
													// recognized

	private final ArrayList<String> powerModeEnum = new ArrayList<String>() {

		private static final long serialVersionUID = 1L;

		{
			add("Power-down");
			add("Sleep");
			add("Normal");
		}
	};
	private final HashMap<String, Integer> powerModeDict = new HashMap<String, Integer>() {

		private static final long serialVersionUID = 1L;

		{
			put(powerModeEnum.get(0), 0);
			put(powerModeEnum.get(1), 1);
			put(powerModeEnum.get(2), 2);
		}
	};

	private final ArrayList<Object> enabledEnum = new ArrayList<Object>() {

		private static final long serialVersionUID = 1L;

		{
			add(false);
			add(true);
		}
	};
	private final HashMap<Object, Integer> enabledDict = new HashMap<Object, Integer>() {

		private static final long serialVersionUID = 1L;

		{
			put(enabledEnum.get(0), 0);
			put(enabledEnum.get(1), 1);
		}
	};

	private final ArrayList<String> levelEnum = new ArrayList<String>() {

		private static final long serialVersionUID = 1L;

		{
			add("High");
			add("Low");
		}
	};

	private final HashMap<Object, Integer> levelDict = new HashMap<Object, Integer>() {

		private static final long serialVersionUID = 1L;

		{
			put(levelEnum.get(0), 0);
			put(levelEnum.get(1), 1);
		}
	};

	private final ArrayList<String> outputEnum = new ArrayList<String>() {

		private static final long serialVersionUID = 1L;

		{
			add("Push-pull");
			add("Open drain");
		}
	};

	private final HashMap<Object, Integer> outputDict = new HashMap<Object, Integer>() {
		private static final long serialVersionUID = 1L;

		{
			put(outputEnum.get(0), 0);
			put(outputEnum.get(1), 1);
		}
	};

	private final ArrayList<String> simModeEnum = new ArrayList<String>() {

		private static final long serialVersionUID = 1L;

		{
			add("4-wire");
			add("3-wire");
		}
	};
	private final HashMap<Object, Integer> simModeDict = new HashMap<Object, Integer>() {

		private static final long serialVersionUID = 1L;

		{
			put(simModeEnum.get(0), 0);
			put(simModeEnum.get(1), 1);
		}
	};

	private final ArrayList<String> fullScaleEnum = new ArrayList<String>() {

		private static final long serialVersionUID = 1L;

		{
			add("250dps");
			add("500dps");
			add("2000dps");
		}
	};

	private final HashMap<Object, Integer> fullScaleDict = new HashMap<Object, Integer>() {

		private static final long serialVersionUID = 1L;

		{
			put(fullScaleEnum.get(0), 0x00);
			put(fullScaleEnum.get(1), 0x01);
			put(fullScaleEnum.get(2), 0x02);
		}
	};

	private final ArrayList<String> bigLittleEndianEnum = new ArrayList<String>() {

		private static final long serialVersionUID = 1L;

		{
			add("Big endian");
			add("Little endian");
		}
	};

	private final HashMap<Object, Integer> bigLittleEndianDict = new HashMap<Object, Integer>() {

		private static final long serialVersionUID = 1L;

		{
			put(bigLittleEndianEnum.get(0), 0x00);
			put(bigLittleEndianEnum.get(1), 0x01);
		}
	};

	private final ArrayList<String> blockDataUpdateEnum = new ArrayList<String>() {

		private static final long serialVersionUID = 1L;

		{
			add("Continous update");
			add("Output registers not updated until reading");
		}
	};
	private final HashMap<Object, Integer> blockDataUpdateDict = new HashMap<Object, Integer>() {

		private static final long serialVersionUID = 1L;

		{
			put(blockDataUpdateEnum.get(0), 0x00);
			put(blockDataUpdateEnum.get(1), 0x01);
		}
	};

	private final ArrayList<String> outSelEnum = new ArrayList<String>() {

		private static final long serialVersionUID = 1L;

		{
			add("LPF1");
			add("HPF");
			add("LPF2");
		}
	};
	private final HashMap<Object, Integer> outSelDict = new HashMap<Object, Integer>() {

		private static final long serialVersionUID = 1L;

		{
			put(outSelEnum.get(0), 0x00);
			put(outSelEnum.get(1), 0x01);
			put(outSelEnum.get(1), 0x02);
		}
	};

	private final ArrayList<String> intSelEnum = new ArrayList<String>() {

		private static final long serialVersionUID = 1L;

		{
			add("LPF1");
			add("HPF");
			add("LPF2");
		}
	};
	private final HashMap<Object, Integer> intSelDict = new HashMap<Object, Integer>() {

		private static final long serialVersionUID = 1L;

		{
			put(intSelEnum.get(0), 0x00);
			put(intSelEnum.get(1), 0x01);
			put(intSelEnum.get(2), 0x02);
		}
	};

	private final ArrayList<String> bootModeEnum = new ArrayList<String>() {

		private static final long serialVersionUID = 1L;

		{
			add("Normal");
			add("Reboot memory content");
		}
	};
	private final HashMap<Object, Integer> bootModeDict = new HashMap<Object, Integer>() {

		private static final long serialVersionUID = 1L;

		{
			put(bootModeEnum.get(0), 0x00);
			put(bootModeEnum.get(1), 0x01);
		}
	};

	private final ArrayList<String> fifoModeEnum = new ArrayList<String>() {

		private static final long serialVersionUID = 1L;

		{
			add("Bypass");
			add("FIFO");
			add("Stream");
			add("Stream-to-Fifo");
			add("Bypass-to-Stream");
		}
	};
	private final HashMap<Object, Integer> fifoModeDict = new HashMap<Object, Integer>() {

		private static final long serialVersionUID = 1L;

		{
			put(fifoModeEnum.get(0), 0x00);
			put(fifoModeEnum.get(1), 0x01);
			put(fifoModeEnum.get(2), 0x02);
			put(fifoModeEnum.get(3), 0x03);
			put(fifoModeEnum.get(4), 0x04);
		}
	};

	private final ArrayList<String> andOrEnum = new ArrayList<String>() {

		private static final long serialVersionUID = 1L;

		{
			add("And");
			add("Or");
		}
	};
	private final HashMap<Object, Integer> andOrDict = new HashMap<Object, Integer>() {

		private static final long serialVersionUID = 1L;

		{
			put(andOrEnum.get(0), 0x00);
			put(andOrEnum.get(1), 0x01);
		}
	};

	private final ArrayList<Integer> dataRateValues = new ArrayList<Integer>() {

		private static final long serialVersionUID = 1L;

		{
			add(95);
			add(190);
			add(380);
			add(760);
		}
	};
	private final ArrayList<Double> bandWidthValues = new ArrayList<Double>() {

		private static final long serialVersionUID = 1L;

		{
			add(12.5);
			add(20.0);
			add(25.0);
			add(30.0);
			add(35.0);
			add(50.0);
			add(70.0);
			add(100.0);
		}
	};
	private final HashMap<Integer, HashMap<Double, Integer>> DRBW = new HashMap<Integer, HashMap<Double, Integer>>() {

		private static final long serialVersionUID = 1L;

		{
			put(dataRateValues.get(0), new HashMap<Double, Integer>() {
				private static final long serialVersionUID = 1L;

				{
					put(bandWidthValues.get(0), 0x00);
					put(bandWidthValues.get(2), 0x01);
				}
			});
			put(dataRateValues.get(1), new HashMap<Double, Integer>() {

				private static final long serialVersionUID = 1L;

				{
					put(bandWidthValues.get(0), 0x04);
					put(bandWidthValues.get(2), 0x05);
					put(bandWidthValues.get(5), 0x06);
					put(bandWidthValues.get(6), 0x07);
				}
			});
			put(dataRateValues.get(2), new HashMap<Double, Integer>() {

				private static final long serialVersionUID = 1L;

				{
					put(bandWidthValues.get(1), 0x08);
					put(bandWidthValues.get(2), 0x09);
					put(bandWidthValues.get(5), 0x0a);
					put(bandWidthValues.get(7), 0x0b);
				}
			});
			put(dataRateValues.get(3), new HashMap<Double, Integer>() {

				private static final long serialVersionUID = 1L;

				{
					put(bandWidthValues.get(3), 0x0c);
					put(bandWidthValues.get(4), 0x0d);
					put(bandWidthValues.get(5), 0x0e);
					put(bandWidthValues.get(7), 0x0f);
				}
			});
		}
	};

	private final ArrayList<Double> highPassFilterCutOffFrequencyValues = new ArrayList<Double>() {

		private static final long serialVersionUID = 1L;

		{
			add(51.4);
			add(27.0);
			add(13.5);
			add(7.2);
			add(3.5);
			add(1.8);
			add(0.9);
			add(0.45);
			add(0.18);
			add(0.09);
			add(0.045);
			add(0.018);
			add(0.009);
		}
	};
	private final HashMap<Double, HashMap<Integer, Integer>> HPCF = new HashMap<Double, HashMap<Integer, Integer>>() {

		private static final long serialVersionUID = 1L;

		{
			put(highPassFilterCutOffFrequencyValues.get(0),
					new HashMap<Integer, Integer>() {

						private static final long serialVersionUID = 1L;

						{
							put(dataRateValues.get(3), 0x00);
						}
					});
			put(highPassFilterCutOffFrequencyValues.get(1),
					new HashMap<Integer, Integer>() {

						private static final long serialVersionUID = 1L;

						{
							put(dataRateValues.get(3), 0x00);
							put(dataRateValues.get(2), 0x01);
						}
					});
			put(highPassFilterCutOffFrequencyValues.get(2),
					new HashMap<Integer, Integer>() {

						private static final long serialVersionUID = 1L;

						{
							put(dataRateValues.get(1), 0x00);
							put(dataRateValues.get(2), 0x01);
							put(dataRateValues.get(3), 0x02);
						}
					});
			put(highPassFilterCutOffFrequencyValues.get(3),
					new HashMap<Integer, Integer>() {

						private static final long serialVersionUID = 1L;

						{
							put(dataRateValues.get(0), 0x00);
							put(dataRateValues.get(1), 0x01);
							put(dataRateValues.get(2), 0x02);
							put(dataRateValues.get(3), 0x03);
						}
					});
			put(highPassFilterCutOffFrequencyValues.get(4),
					new HashMap<Integer, Integer>() {

						private static final long serialVersionUID = 1L;

						{
							put(dataRateValues.get(0), 0x01);
							put(dataRateValues.get(1), 0x02);
							put(dataRateValues.get(2), 0x03);
							put(dataRateValues.get(3), 0x04);
						}
					});
			put(highPassFilterCutOffFrequencyValues.get(5),
					new HashMap<Integer, Integer>() {

						private static final long serialVersionUID = 1L;

						{
							put(dataRateValues.get(0), 0x02);
							put(dataRateValues.get(1), 0x03);
							put(dataRateValues.get(2), 0x04);
							put(dataRateValues.get(3), 0x05);
						}
					});
			put(highPassFilterCutOffFrequencyValues.get(6),
					new HashMap<Integer, Integer>() {

						private static final long serialVersionUID = 1L;

						{
							put(dataRateValues.get(0), 0x03);
							put(dataRateValues.get(1), 0x04);
							put(dataRateValues.get(2), 0x05);
							put(dataRateValues.get(3), 0x06);
						}
					});
			put(highPassFilterCutOffFrequencyValues.get(7),
					new HashMap<Integer, Integer>() {

						private static final long serialVersionUID = 1L;

						{
							put(dataRateValues.get(0), 0x04);
							put(dataRateValues.get(1), 0x05);
							put(dataRateValues.get(2), 0x06);
							put(dataRateValues.get(3), 0x07);
						}
					});
			put(highPassFilterCutOffFrequencyValues.get(8),
					new HashMap<Integer, Integer>() {

						private static final long serialVersionUID = 1L;

						{
							put(dataRateValues.get(0), 0x05);
							put(dataRateValues.get(1), 0x06);
							put(dataRateValues.get(2), 0x07);
							put(dataRateValues.get(3), 0x08);
						}
					});
			put(highPassFilterCutOffFrequencyValues.get(9),
					new HashMap<Integer, Integer>() {

						private static final long serialVersionUID = 1L;

						{
							put(dataRateValues.get(0), 0x06);
							put(dataRateValues.get(1), 0x07);
							put(dataRateValues.get(2), 0x08);
							put(dataRateValues.get(3), 0x09);
						}
					});
			put(highPassFilterCutOffFrequencyValues.get(10),
					new HashMap<Integer, Integer>() {

						private static final long serialVersionUID = 1L;

						{
							put(dataRateValues.get(0), 0x07);
							put(dataRateValues.get(1), 0x08);
							put(dataRateValues.get(2), 0x09);
						}
					});
			put(highPassFilterCutOffFrequencyValues.get(11),
					new HashMap<Integer, Integer>() {

						private static final long serialVersionUID = 1L;

						{
							put(dataRateValues.get(0), 0x08);
							put(dataRateValues.get(1), 0x09);
						}
					});
			put(highPassFilterCutOffFrequencyValues.get(12),
					new HashMap<Integer, Integer>() {

						private static final long serialVersionUID = 1L;

						{
							put(dataRateValues.get(0), 0x09);
						}
					});
		}
	};

	private final ArrayList<String> highPassFilterModes = new ArrayList<String>() {

		private static final long serialVersionUID = 1L;

		{
			add("Normal with reset.");
			add("Reference signal for filtering.");
			add("Normal.");
			add("Autoreset on interrupt.");
		}
	};
	private final HashMap<String, Integer> hpmDict = new HashMap<String, Integer>() {

		private static final long serialVersionUID = 1L;

		{
			put(highPassFilterModes.get(0), 0x00);
			put(highPassFilterModes.get(1), 0x01);
			put(highPassFilterModes.get(2), 0x02);
			put(highPassFilterModes.get(3), 0x03);
		}
	};

	// For calibration purposes
	private double meanX = 0.0;
	private double maxX = 0.0;
	private double minX = 0.0;
	private double meanY = 0.0;
	private double maxY = 0.0;
	private double minY = 0.0;
	private double meanZ = 0.0;
	private double maxZ = 0.0;
	private double minZ = 0.0;
	private double gain = 1.0;
	// private int x0;
	private boolean ifWriteBlock;

	//
	//

	public AdafruitL3GD20(int address, boolean ifWriteBlock) {
		bitOps = new BitOps();
		try {
			final I2CBus bus = I2CFactory.getInstance(I2CBus.BUS_1);

			device = bus.getDevice(address);
		} catch (Exception e) {
			e.printStackTrace();
		}
		this.ifWriteBlock = ifWriteBlock;

		// x0 = 0;
	}

	public void init() {
		System.out.println("Initiating...");
		if (getFullScaleValue() == fullScaleEnum.get(0))
			gain = 0.00875;
		else if (getFullScaleValue() == fullScaleEnum.get(1))
			gain = 0.0175;
		else if (getFullScaleValue() == fullScaleEnum.get(2))
			gain = 0.07;
		System.out.println("Gain set to: " + gain);
	}

	// def __log(self, register, mask, current, new):
	// register = '0b' + bin(register)[2:].zfill(8)
	// mask = '0b' + bin(mask)[2:].zfill(8)
	// current = '0b' + bin(current)[2:].zfill(8)
	// new = '0b' + bin(new)[2:].zfill(8)
	// print('Change in register:' + register + ' mask:' + mask + ' from:' +
	// current + ' to:' + new)

	private void writeToRegister(int register, int mask, int value) {
		int current = 0;
		try {
			current = device.read(register); // Get current value
		} catch (Exception e) {
			e.printStackTrace();
		}
		int newVal = bitOps.setValueUnderMask(value, current, mask);
		// if self.__ifLog:
		// self.__log(register, mask, current, new)
		if (ifWriteBlock) {
			try {
				device.write(register, (byte) newVal);
			} catch (Exception e) {
				e.printStackTrace();
			}
		}
	}

	private int readFromRegister(int register, int mask) {
		int current = 0;
		try {
			current = device.read(register); // Get current value
		} catch (Exception e) {
			e.printStackTrace();
		}
		return bitOps.getValueUnderMask(current, mask);
	}

	private Object readFromRegisterWithDictionaryMatch(int register, int mask,
			HashMap<Object, Integer> dictionary) {
		int current = readFromRegister(register, mask);
		for (Object key : dictionary.keySet())
			if (dictionary.get(key) == current)
				return key;
		return null;
	}

	private void writeToRegisterWithDictionaryCheck(int register, int mask,
			Object value, HashMap<Object, Integer> dictionary,
			String dictionaryName) {
		if (!dictionary.containsKey(value)) {
			try {
				throw new Exception("Value:" + value + " is not in range of: "
						+ dictionaryName);
			} catch (Exception e) {
				e.printStackTrace();
			}
		}
		writeToRegister(register, mask, dictionary.get(value));
	}

	private double mean(ArrayList<Double> list) {
		double sum = 0.0;
		for (double d : list)
			sum += d;
		return sum / list.size();
	}

	private double max(ArrayList<Double> list) {
		double max = 0.0;
		for (double d : list)
			if (d > max)
				max = d;
		return max;
	}

	private double min(ArrayList<Double> list) {
		double min = Double.MAX_VALUE;
		for (double d : list)
			if (d < min)
				min = d;
		return min;
	}

	/**
	 * Returns (min, mean, max)
	 */
	private void calibrateX() {
		System.out.println("Calibrating axis X, please do not move sensor...");
		ArrayList<Double> buff = new ArrayList<Double>();
		for (int t = 0; t < 20; t++) {
			while (getAxisDataAvailableValue().get(0) == 0) {
				try {
					TimeUnit.MICROSECONDS.sleep(100);
				} catch (Exception e) {
					e.printStackTrace();
				}
			}
			buff.add(getRawOutXValue());
		}
		meanX = mean(buff);
		maxX = max(buff);
		minX = min(buff);
		System.out.println("Done: (min=" + minX + ";mean=" + meanX + ";max="
				+ maxX + ")");
	}

	/**
	 * Returns (min, mean, max)
	 */
	private void calibrateY() {
		System.out.print("Calibrating axis Y, please do not move sensor...");
		ArrayList<Double> buff = new ArrayList<Double>();
		for (int t = 0; t < 20; t++) {
			while (getAxisDataAvailableValue().get(1) == 0) {
				try {
					TimeUnit.MICROSECONDS.sleep(100);
				} catch (Exception e) {
					e.printStackTrace();
				}
			}
			buff.add(getRawOutYValue());
		}
		meanY = mean(buff);
		maxY = max(buff);
		minY = min(buff);
		System.out.println("Done: (min=" + minX + ";mean=" + meanX + ";max="
				+ maxX + ")");
	}

	/**
	 * Returns (min, mean, max)
	 */
	private void calibrateZ() {
		System.out.print("Calibrating axis Z, please do not move sensor...");
		ArrayList<Double> buff = new ArrayList<Double>();
		for (int t = 0; t < 20; t++) {
			while (getAxisDataAvailableValue().get(2) == 0) {
				try {
					TimeUnit.MICROSECONDS.sleep(100);
				} catch (Exception e) {
					e.printStackTrace();
				}
			}
			buff.add(getRawOutZValue());
		}
		meanY = mean(buff);
		maxY = max(buff);
		minY = min(buff);
		System.out.println("Done: (min=" + minX + ";mean=" + meanX + ";max="
				+ maxX + ")");
	}

	public void calibrate() {
		calibrateX();
		calibrateY();
		calibrateZ();
	}

	public HashMap<String, Object> returnConfiguration() throws IOException {
		HashMap<String, Object> config = new HashMap<String, Object>();
		config.put("Device Id.", getDeviceIdValue());
		config.put("Data rate and bandwidth.", getDataRateAndBandwidth());
		config.put("Axis X enabled.", getAxisXEnabled());
		config.put("Axis Y enabled.", getAxisYEnabled());
		config.put("Axis Z enabled.", getAxisZEnabled());
		config.put("Power mode.", getPowerMode());
		config.put("Cut off frequency.", getHighPassCutOffFreq());
		config.put("INT1 Enabled", getINT1Enabled());
		config.put("Boot status available on INT1",
				getBootStatusOnINT1Enabled());
		config.put("Interrupt active configuration on INT1",
				getActiveConfINT1Level());
		config.put("Push-pull/open drain", getPushPullOrOpenDrainValue());
		config.put("Date-ready on DRDY/INT2", getDataReadyOnINT2Enabled());
		config.put("FIFO watermark interrupt on DRDY/INT2",
				getFifoWatermarkOnINT2Enabled());
		config.put("FIFO overrun interrupt in DRDY/INT2",
				getFifoOverrunOnINT2Enabled());
		config.put("FIFO empty interrupt on DRDY/INT2",
				getFifoEmptyOnINT2Enabled());
		config.put("SPI mode", getSpiModeValue());
		config.put("Full scale selection", getFullScaleValue());
		config.put("Big/Little endian", getBigLittleEndianValue());
		config.put("Block data update", getBlockDataUpdateValue());
		config.put("Boot mode", getBootModeValue());
		config.put("Fifo enabled", getFifoEnabled());
		config.put("High pass filter enabled", getHighPassFilterEnabled());
		config.put("INT1 selection configuration", getINT1SelectionValue());
		config.put("Out selection configuration", getOutSelectionValue());
		config.put("Reference value for interrupt generation",
				getReferenceValue());
		config.put("(X, Y, Z) axis overrun", getAxisOverrunValue());
		config.put("(X, Y, Z) data available", getAxisDataAvailableValue());
		config.put("Fifo threshold - watermark level", getFifoThresholdValue());
		config.put("Fifo mode", getFifoModeValue());
		config.put("Fifo stored data level", getFifoStoredDataLevelValue());
		config.put("Fifo empty", getIsFifoEmptyValue());
		config.put("Fifo full", getIsFifoFullValue());
		config.put("Fifo filling is greater or equal than watermark level",
				getIsFifoGreaterOrEqualThanWatermarkValue());
		config.put("Interrupt combination", getInt1CombinationValue());
		config.put("Latch interrupt request", getInt1LatchRequestEnabled());
		config.put("Int 1 generation on Z higher than threshold",
				getInt1GenerationOnZHighEnabled());
		config.put("Int 1 generation on Z lower than threshold",
				getInt1GenerationOnZLowEnabled());
		config.put("Int 1 generation on Y higher than threshold",
				getInt1GenerationOnYHighEnabled());
		config.put("Int 1 generation on Y lower than threshold",
				getInt1GenerationOnYLowEnabled());
		config.put("Int 1 generation on X higher than threshold",
				getInt1GenerationOnXHighEnabled());
		config.put("Int 1 generation on X lower than threshold",
				getInt1GenerationOnXLowEnabled());
		config.put("Int1 active", getInt1ActiveValue());
		config.put("Z high event occured", getZHighEventOccuredValue());
		config.put("Z low event occured", getZLowEventOccuredValue());
		config.put("Y high event occured", getYHighEventOccuredValue());
		config.put("Y low event occured", getYLowEventOccuredValue());
		config.put("X high event occured", getXHighEventOccuredValue());
		config.put("X low event occured", getXLowEventOccuredValue());
		config.put("(X,Y,Z) INT1 threshold value", getInt1ThresholdValues());
		config.put("Int 1 duration wait", getInt1DurationWaitEnabled());
		config.put("Int 1 duration value", getInt1DurationValue());
		return config;
	}

	/**
	 * Device Id.
	 */
	public int getDeviceIdValue() throws IOException {
		return readFromRegister(REG_R_WHO_AM_I, 0xff);
	}

	public void setAxisXEnabled(boolean enabled) {
		writeToRegisterWithDictionaryCheck(REG_RW_CTRL_REG1,
				MASK_CTRL_REG1_Xen, enabled, enabledDict, "EnabledEnum");
	}

	/**
	 * Axis X enabled.
	 */
	public boolean getAxisXEnabled() {
		return (Boolean) readFromRegisterWithDictionaryMatch(REG_RW_CTRL_REG1,
				MASK_CTRL_REG1_Xen, enabledDict);
	}

	public void setAxisYEnabled(boolean enabled) {
		writeToRegisterWithDictionaryCheck(REG_RW_CTRL_REG1,
				MASK_CTRL_REG1_Yen, enabled, enabledDict, "EnabledEnum");
	}

	/**
	 * Axis Y enabled.
	 */
	public boolean getAxisYEnabled() {
		return (Boolean) readFromRegisterWithDictionaryMatch(REG_RW_CTRL_REG1,
				MASK_CTRL_REG1_Yen, enabledDict);
	}

	public void setAxisZEnabled(boolean enabled) {
		writeToRegisterWithDictionaryCheck(REG_RW_CTRL_REG1,
				MASK_CTRL_REG1_Zen, enabled, enabledDict, "EnabledEnum");
	}

	/**
	 * Axis Z enabled.
	 */
	public boolean getAxisZEnabled() {
		return (Boolean) readFromRegisterWithDictionaryMatch(REG_RW_CTRL_REG1,
				MASK_CTRL_REG1_Zen, enabledDict);
	}

	public void setPowerMode(String mode) {
		if (!powerModeDict.containsKey(mode)) {
			try {
				throw new Exception("Value:" + mode
						+ " is not in range of: PowerModeEnum");
			} catch (Exception e) {
				e.printStackTrace();
			}
		}
		if (powerModeDict.get(mode) == 0)

			// Power-down
			writeToRegister(REG_RW_CTRL_REG1, MASK_CTRL_REG1_PD, 0);
		else if (powerModeDict.get(mode) == 1)
			// Sleep
			writeToRegister(REG_RW_CTRL_REG1, MASK_CTRL_REG1_PD
					| MASK_CTRL_REG1_Zen | MASK_CTRL_REG1_Yen
					| MASK_CTRL_REG1_Xen, 8);
		else if (powerModeDict.get(mode) == 2)
			// Normal
			writeToRegister(REG_RW_CTRL_REG1, MASK_CTRL_REG1_PD, 1);
	}

	/**
	 * Power mode.
	 */
	public String getPowerMode() {
		int powermode = readFromRegister(REG_RW_CTRL_REG1, MASK_CTRL_REG1_PD
				| MASK_CTRL_REG1_Xen | MASK_CTRL_REG1_Yen | MASK_CTRL_REG1_Zen);
		System.out.println(Integer.toBinaryString(powermode));
		int dictval = 4;
		if (!bitOps.checkBit(powermode, 3))
			dictval = 0;
		else if (powermode == Integer.parseInt("0b1000"))
			dictval = 1;
		else if (bitOps.checkBit(powermode, 3))
			dictval = 2;
		for (String key : powerModeDict.keySet())
			if (powerModeDict.get(key) == dictval)
				return key;
		return null;
	}

	private String zfill(String str, int width) {
		String res = "";
		for (int i = 0; i < (width - str.length()); i++) {
			res += "0";
		}
		return res;
	}

	public void printDataRateAndBandwidthAvailableValues() {
		for (int dr : DRBW.keySet()) {
			System.out.print("Output data rate: " + dr + "[Hz]");
			for (double bw : DRBW.get(dr).keySet()) {
				System.out.print(" Bandwidth: "
						+ bw
						+ " (DRBW="
						+ "0b"
						+ zfill(Integer.toBinaryString(DRBW.get(dr).get(bw))
								.substring(2), 4) + ")");
			}
		}
	}

	public void setDataRateAndBandwidth(double datarate, double bandwidth) {

		if (!DRBW.containsKey(datarate)) {
			try {
				throw new Exception("Data rate:" + datarate
						+ " not in range of data rate values.");
			} catch (Exception e) {
				e.printStackTrace();
			}
		}
		if (!DRBW.get(datarate).containsKey(bandwidth)) {
			try {
				throw new Exception("Bandwidth: " + bandwidth
						+ " cannot be assigned to data rate: " + datarate);
			} catch (Exception e) {
				e.printStackTrace();
			}
		}
		int bits = DRBW.get(datarate).get(bandwidth);
		writeToRegister(REG_RW_CTRL_REG1,
				MASK_CTRL_REG1_DR | MASK_CTRL_REG1_BW, bits);
	}

	/**
	 * Data rate and bandwidth.
	 */
	public ArrayList<Double> getDataRateAndBandwidth() {
		ArrayList<Double> list = new ArrayList<Double>();
		int current = readFromRegister(REG_RW_CTRL_REG1, MASK_CTRL_REG1_DR
				| MASK_CTRL_REG1_BW);
		for (double dr : DRBW.keySet()) {
			for (double bw : DRBW.get(dr).keySet()) {
				if (DRBW.get(dr).get(bw) == current) {
					list.add(dr);
					list.add(bw);
					return list;
				}
			}
		}
		return list;

	}

	public void printHighPassFilterCutOffFrequencyAvailableValues() {
		for (double freq : HPCF.keySet()) {
			System.out.print("High pass cut off: " + freq + "[Hz]");
			for (double odr : HPCF.get(freq).keySet()) {
				System.out.print(" Output data rate: "
						+ odr
						+ " (HPCF="
						+ "0b"
						+ zfill(Integer.toBinaryString(HPCF.get(freq).get(odr))
								.substring(2), 4) + ")");
			}
		}

	}

	public void setHighPassCutOffFreq(int freq) {
		if (!HPCF.containsKey(freq)) {
			try {
				throw new Exception(
						"Frequency:"
								+ freq
								+ " is not in range of high pass frequency cut off values.");
			} catch (Exception e) {
				e.printStackTrace();
			}
		}

		double datarate = getDataRateAndBandwidth().get(0);
		if (!HPCF.get(freq).containsKey(datarate)) {
			try {
				throw new Exception("Frequency: " + freq
						+ " cannot be assigned to data rate: " + datarate);
			} catch (Exception e) {
				e.printStackTrace();
			}
		}
		int bits = HPCF.get(freq).get(datarate);
		writeToRegister(REG_RW_CTRL_REG2, MASK_CTRL_REG2_HPCF, bits);
	}

	/**
	 * Cut off frequency.
	 */
	public double getHighPassCutOffFreq() {
		int current = readFromRegister(REG_RW_CTRL_REG2, MASK_CTRL_REG2_HPCF);
		double datarate = getDataRateAndBandwidth().get(0);
		for (double freq : HPCF.keySet()) {
			for (double dr : HPCF.get(freq).keySet()) {
				if (dr == datarate) {
					if (HPCF.get(freq).get(datarate) == current) {
						return freq;
					}
				}
			}
		}
		return datarate;
	}

	public void setHighPassFilterMode(String mode) {
		if (!hpmDict.containsKey(mode)) {
			try {
				throw new Exception("EnabledEnum:" + mode
						+ " is not in range of high pass frequency modes.");
			} catch (Exception e) {
				e.printStackTrace();
			}
		}
		int bits = hpmDict.get(mode);
		writeToRegister(REG_RW_CTRL_REG2, MASK_CTRL_REG2_HPM, bits);

	}

	/**
	 * High pass filter mode
	 */
	public String getHighPassFilterMode() {
		int current = readFromRegister(REG_RW_CTRL_REG2, MASK_CTRL_REG2_HPM);
		for (String mode : hpmDict.keySet())
			if (hpmDict.get(mode) == current)
				return mode;
		return null;
	}

	public void setINT1Enabled(boolean enabled) {
		writeToRegisterWithDictionaryCheck(REG_RW_CTRL_REG3,
				MASK_CTRL_REG3_I1_Int1, enabled, enabledDict, "EnabledEnum");
	}

	/**
	 * INT1 Enabled
	 */
	public boolean getINT1Enabled() {
		return (Boolean) readFromRegisterWithDictionaryMatch(REG_RW_CTRL_REG3,
				MASK_CTRL_REG3_I1_Int1, enabledDict);
	}

	public void setBootStatusOnINT1Enabled(boolean enabled) {
		writeToRegisterWithDictionaryCheck(REG_RW_CTRL_REG3,
				MASK_CTRL_REG3_I1_BOOT, enabled, enabledDict, "EnabledEnum");
	}

	/**
	 * Boot status available on INT1
	 */
	public boolean getBootStatusOnINT1Enabled() {
		return (Boolean) readFromRegisterWithDictionaryMatch(REG_RW_CTRL_REG3,
				MASK_CTRL_REG3_I1_BOOT, enabledDict);
	}

	public void setActiveConfINT1Level(String level) {
		writeToRegisterWithDictionaryCheck(REG_RW_CTRL_REG3,
				MASK_CTRL_REG3_H_LACTIVE, level, levelDict, "LevelEnum");
	}

	/**
	 * Interrupt active configuration on INT1
	 */
	public String getActiveConfINT1Level() {
		return (String) readFromRegisterWithDictionaryMatch(REG_RW_CTRL_REG3,
				MASK_CTRL_REG3_H_LACTIVE, levelDict);
	}

	public void setPushPullOrOpenDrainValue(int value) {
		writeToRegisterWithDictionaryCheck(REG_RW_CTRL_REG3,
				MASK_CTRL_REG3_PP_OD, value, outputDict, "OutputEnum");
	}

	/**
	 * Push-pull/open drain
	 */
	public int getPushPullOrOpenDrainValue() {
		return (Integer) readFromRegisterWithDictionaryMatch(REG_RW_CTRL_REG3,
				MASK_CTRL_REG3_PP_OD, outputDict);
	}

	public void setDataReadyOnINT2Enabled(boolean enabled) {
		writeToRegisterWithDictionaryCheck(REG_RW_CTRL_REG3,
				MASK_CTRL_REG3_I2_DRDY, enabled, enabledDict, "EnabledEnum");
	}

	/**
	 * Date-ready on DRDY/INT2
	 */
	public boolean getDataReadyOnINT2Enabled() {
		return (Boolean) readFromRegisterWithDictionaryMatch(REG_RW_CTRL_REG3,
				MASK_CTRL_REG3_I2_DRDY, enabledDict);
	}

	public void setFifoWatermarkOnINT2Enabled(boolean enabled) {
		writeToRegisterWithDictionaryCheck(REG_RW_CTRL_REG3,
				MASK_CTRL_REG3_I2_WTM, enabled, enabledDict, "EnabledEnum");
	}

	/**
	 * FIFO watermark interrupt on DRDY/INT2
	 */
	public boolean getFifoWatermarkOnINT2Enabled() {
		return (Boolean) readFromRegisterWithDictionaryMatch(REG_RW_CTRL_REG3,
				MASK_CTRL_REG3_I2_WTM, enabledDict);
	}

	public void setFifoOverrunOnINT2Enabled(boolean enabled) {
		writeToRegisterWithDictionaryCheck(REG_RW_CTRL_REG3,
				MASK_CTRL_REG3_I2_ORUN, enabled, enabledDict, "EnabledEnum");
	}

	/**
	 * FIFO overrun interrupt in DRDY/INT2
	 */
	public boolean getFifoOverrunOnINT2Enabled() {
		return (Boolean) readFromRegisterWithDictionaryMatch(REG_RW_CTRL_REG3,
				MASK_CTRL_REG3_I2_ORUN, enabledDict);
	}

	public void setFifoEmptyOnINT2Enabled(boolean enabled) {
		writeToRegisterWithDictionaryCheck(REG_RW_CTRL_REG3,
				MASK_CTRL_REG3_I2_EMPTY, enabled, enabledDict, "EnabledEnum");
	}

	/**
	 * FIFO empty interrupt on DRDY/INT2
	 */
	public boolean getFifoEmptyOnINT2Enabled() {
		return (Boolean) readFromRegisterWithDictionaryMatch(REG_RW_CTRL_REG3,
				MASK_CTRL_REG3_I2_EMPTY, enabledDict);
	}

	public void setSpiModeValue(int value) {
		writeToRegisterWithDictionaryCheck(REG_RW_CTRL_REG4,
				MASK_CTRL_REG4_SIM, value, simModeDict, "SimModeEnum");
	}

	/**
	 * SPI mode
	 */
	public int getSpiModeValue() {
		return (Integer) readFromRegisterWithDictionaryMatch(REG_RW_CTRL_REG4,
				MASK_CTRL_REG4_SIM, simModeDict);
	}

	public void setFullScaleValue(String value) {
		writeToRegisterWithDictionaryCheck(REG_RW_CTRL_REG4, MASK_CTRL_REG4_FS,
				value, fullScaleDict, "FullScaleEnum");
	}

	/**
	 * Full scale selection
	 */
	private String getFullScaleValue() {
		return (String) readFromRegisterWithDictionaryMatch(REG_RW_CTRL_REG4,
				MASK_CTRL_REG4_FS, fullScaleDict);
	}

	public void setBigLittleEndianValue(int value) {
		writeToRegisterWithDictionaryCheck(REG_RW_CTRL_REG4,
				MASK_CTRL_REG4_BLE, value, bigLittleEndianDict,
				"BigLittleEndianEnum");
	}

	/**
	 * Big/Little endian
	 */
	public int getBigLittleEndianValue() {
		return (Integer) readFromRegisterWithDictionaryMatch(REG_RW_CTRL_REG4,
				MASK_CTRL_REG4_BLE, bigLittleEndianDict);
	}

	public void setBlockDataUpdateValue(int value) {
		writeToRegisterWithDictionaryCheck(REG_RW_CTRL_REG4,
				MASK_CTRL_REG4_BDU, value, blockDataUpdateDict,
				"BlockDataUpdateEnum");
	}

	/**
	 * Block data update
	 */
	public int getBlockDataUpdateValue() {
		return (Integer) readFromRegisterWithDictionaryMatch(REG_RW_CTRL_REG4,
				MASK_CTRL_REG4_BDU, blockDataUpdateDict);
	}

	public void setBootModeValue(int value) {
		writeToRegisterWithDictionaryCheck(REG_RW_CTRL_REG5,
				MASK_CTRL_REG5_BOOT, value, bootModeDict, "BootModeEnum");
	}

	/**
	 * Boot mode
	 */
	public int getBootModeValue() {
		return (Integer) readFromRegisterWithDictionaryMatch(REG_RW_CTRL_REG5,
				MASK_CTRL_REG5_BOOT, bootModeDict);
	}

	public void setFifoEnabled(boolean enabled) {
		writeToRegisterWithDictionaryCheck(REG_RW_CTRL_REG5,
				MASK_CTRL_REG5_FIFO_EN, enabled, enabledDict, "EnabledEnum");
	}

	/**
	 * Fifo enabled
	 */
	public boolean getFifoEnabled() {
		return (Boolean) readFromRegisterWithDictionaryMatch(REG_RW_CTRL_REG5,
				MASK_CTRL_REG5_FIFO_EN, enabledDict);
	}

	public void setHighPassFilterEnabled(boolean enabled) {
		writeToRegisterWithDictionaryCheck(REG_RW_CTRL_REG5,
				MASK_CTRL_REG5_HPEN, enabled, enabledDict, "EnabledEnum");
	}

	/**
	 * High pass filter enabled
	 */
	public boolean getHighPassFilterEnabled() {
		return (Boolean) readFromRegisterWithDictionaryMatch(REG_RW_CTRL_REG5,
				MASK_CTRL_REG5_HPEN, enabledDict);
	}

	public void setINT1SelectionValue(int value) {
		writeToRegisterWithDictionaryCheck(REG_RW_CTRL_REG5,
				MASK_CTRL_REG5_INT_SEL, value, intSelDict, "IntSelEnum");
	}

	/**
	 * INT1 selection configuration
	 */
	public int getINT1SelectionValue() {
		return (Integer) readFromRegisterWithDictionaryMatch(REG_RW_CTRL_REG5,
				MASK_CTRL_REG5_INT_SEL, intSelDict);
	}

	public void setOutSelectionValue(int value) {
		writeToRegisterWithDictionaryCheck(REG_RW_CTRL_REG5,
				MASK_CTRL_REG5_OUT_SEL, value, outSelDict, "OutSelEnum");
	}

	/**
	 * Out selection configuration
	 */
	public int getOutSelectionValue() {
		return (Integer) readFromRegisterWithDictionaryMatch(REG_RW_CTRL_REG5,
				MASK_CTRL_REG5_OUT_SEL, outSelDict);
	}

	public void setReference_Value(int value) {
		writeToRegister(REG_RW_REFERENCE, 0xff, value);
	}

	/**
	 * Reference value for interrupt generation
	 */
	public int getReferenceValue() {
		return readFromRegister(REG_RW_REFERENCE, 0xff);
	}

	/**
	 * Output temperature
	 */
	public int getOutTempValue() {
		return readFromRegister(REG_R_OUT_TEMP, 0xff);
	}

	/**
	 * (X, Y, Z) axis overrun
	 */
	public ArrayList<Integer> getAxisOverrunValue() {
		ArrayList<Integer> list = new ArrayList<Integer>();
		int zor = 0;
		int yor = 0;
		int xor = 0;
		if (readFromRegister(REG_R_STATUS_REG, MASK_STATUS_REG_ZYXOR) == 0x01) {
			zor = readFromRegister(REG_R_STATUS_REG, MASK_STATUS_REG_ZOR);
			yor = readFromRegister(REG_R_STATUS_REG, MASK_STATUS_REG_YOR);
			xor = readFromRegister(REG_R_STATUS_REG, MASK_STATUS_REG_XOR);
		}
		list.add(xor);
		list.add(yor);
		list.add(zor);
		return list;
	}

	/**
	 * (X, Y, Z) data available
	 */
	private ArrayList<Integer> getAxisDataAvailableValue() {
		ArrayList<Integer> list = new ArrayList<Integer>();
		int zda = 0;
		int yda = 0;
		int xda = 0;
		if (readFromRegister(REG_R_STATUS_REG, MASK_STATUS_REG_ZYXDA) == 0x01) {
			zda = readFromRegister(REG_R_STATUS_REG, MASK_STATUS_REG_ZDA);
			yda = readFromRegister(REG_R_STATUS_REG, MASK_STATUS_REG_YDA);
			xda = readFromRegister(REG_R_STATUS_REG, MASK_STATUS_REG_XDA);
		}
		list.add(xda);
		list.add(yda);
		list.add(zda);
		return list;
	}

	/**
	 * Raw X angular speed data
	 */
	private double getRawOutXValue() {
		int l = readFromRegister(REG_R_OUT_X_L, 0xff);
		int h_u2 = readFromRegister(REG_R_OUT_X_H, 0xff);
		int h = bitOps.twosComplementToByte(h_u2);
		if (h < 0)
			return (h * 256 - l) * gain;
		else
			return (h * 256 + l) * gain;
	}

	/**
	 * Raw Y angular speed data
	 */
	private double getRawOutYValue() {
		int l = readFromRegister(REG_R_OUT_Y_L, 0xff);
		int h_u2 = readFromRegister(REG_R_OUT_Y_H, 0xff);
		int h = bitOps.twosComplementToByte(h_u2);
		if (h < 0)
			return (h * 256 - l) * gain;
		else
			return (h * 256 + l) * gain;
	}

	/**
	 * Raw Z angular speed data
	 */
	private double getRawOutZValue() {
		int l = readFromRegister(REG_R_OUT_Z_L, 0xff);
		int h_u2 = readFromRegister(REG_R_OUT_Z_H, 0xff);
		int h = bitOps.twosComplementToByte(h_u2);
		if (h < 0)
			return (h * 256 - l) * gain;
		else
			return (h * 256 + l) * gain;
	}

	/**
	 * Raw [X, Y, Z] values of angular speed
	 */
	public ArrayList<Double> getRawOutValue() {
		return new ArrayList<Double>() {

			private static final long serialVersionUID = 1L;

			{
				add(getRawOutXValue());
				add(getRawOutYValue());
				add(getRawOutZValue());
			}
		};
	}

	/**
	 * Calibrated X angular speed data
	 */
	private double getCalOutXValue() {
		double x = getRawOutXValue();
		if (x >= minX && x <= maxX)
			return 0.0;
		else
			return x - meanX;
	}

	/**
	 * Calibrated Y angular speed data
	 */
	private double getCalOutYValue() {
		double y = getRawOutYValue();
		if (y >= minY && y <= maxY)
			return 0.0;
		else
			return y - meanY;
	}

	/**
	 * Calibrated Z angular speed data
	 */
	private double getCalOutZValue() {
		double z = getRawOutZValue();
		if (z >= minZ && z <= maxZ)
			return 0.0;
		else
			return z - meanZ;
	}

	/**
	 * Calibrated [X, Y, Z] value of angular speed, calibrated
	 */
	public ArrayList<Double> getCalOutValue() {
		return new ArrayList<Double>() {

			private static final long serialVersionUID = 1L;

			{
				add(getCalOutXValue());
				add(getCalOutYValue());
				add(getCalOutZValue());
			}
		};
	}

	public void setFifoThresholdValue(int value) {
		writeToRegister(REG_RW_FIFO_CTRL_REG, MASK_FIFO_CTRL_REG_WTM, value);
	}

	/**
	 * Fifo threshold - watermark level
	 */
	public int getFifoThresholdValue() {
		return readFromRegister(REG_RW_FIFO_CTRL_REG, MASK_FIFO_CTRL_REG_WTM);
	}

	public void setFifoModeValue(int value) {
		writeToRegisterWithDictionaryCheck(REG_RW_FIFO_CTRL_REG,
				MASK_FIFO_CTRL_REG_FM, value, fifoModeDict, "FifoModeEnum");
	}

	/**
	 * Fifo mode
	 */
	public int getFifoModeValue() {
		return (Integer) readFromRegisterWithDictionaryMatch(
				REG_RW_FIFO_CTRL_REG, MASK_FIFO_CTRL_REG_FM, fifoModeDict);
	}

	/**
	 * Fifo stored data level
	 */
	public int getFifoStoredDataLevelValue() {
		return readFromRegister(REG_R_FIFO_SRC_REG, MASK_FIFO_SRC_REG_FSS);
	}

	/**
	 * Fifo empty
	 */
	public int getIsFifoEmptyValue() {
		return (Integer) readFromRegisterWithDictionaryMatch(
				REG_R_FIFO_SRC_REG, MASK_FIFO_SRC_REG_EMPTY, enabledDict);
	}

	/**
	 * Fifo full
	 */
	public boolean getIsFifoFullValue() {
		return (Boolean) readFromRegisterWithDictionaryMatch(
				REG_R_FIFO_SRC_REG, MASK_FIFO_SRC_REG_OVRN, enabledDict);
	}

	/**
	 * Fifo filling is greater or equal than watermark level
	 */
	public boolean getIsFifoGreaterOrEqualThanWatermarkValue()
			throws IOException {
		return (Boolean) readFromRegisterWithDictionaryMatch(
				REG_R_FIFO_SRC_REG, MASK_FIFO_SRC_REG_WTM, enabledDict);
	}

	public void setInt1Combination_Value(int value) {
		writeToRegisterWithDictionaryCheck(REG_RW_INT1_CFG_REG,
				MASK_INT1_CFG_ANDOR, value, andOrDict, "AndOrEnum");
	}

	/**
	 * Interrupt combination
	 */
	public int getInt1CombinationValue() {
		return (Integer) readFromRegisterWithDictionaryMatch(
				REG_RW_INT1_CFG_REG, MASK_INT1_CFG_ANDOR, andOrDict);
	}

	public void setInt1LatchRequestEnabled(boolean value) {
		writeToRegisterWithDictionaryCheck(REG_RW_INT1_CFG_REG,
				MASK_INT1_CFG_LIR, value, enabledDict, "EnabledEnum");
	}

	/**
	 * Latch interrupt request
	 */
	public boolean getInt1LatchRequestEnabled() {
		return (Boolean) readFromRegisterWithDictionaryMatch(
				REG_RW_INT1_CFG_REG, MASK_INT1_CFG_LIR, enabledDict);
	}

	public void setInt1GenerationOnZHighEnabled(boolean value) {
		writeToRegisterWithDictionaryCheck(REG_RW_INT1_CFG_REG,
				MASK_INT1_CFG_ZHIE, value, enabledDict, "EnabledEnum");
	}

	/**
	 * Int 1 generation on Z higher than threshold
	 */
	public boolean getInt1GenerationOnZHighEnabled() {
		return (Boolean) readFromRegisterWithDictionaryMatch(
				REG_RW_INT1_CFG_REG, MASK_INT1_CFG_ZHIE, enabledDict);
	}

	public void setInt1GenerationOnZLowEnabled(boolean value) {
		writeToRegisterWithDictionaryCheck(REG_RW_INT1_CFG_REG,
				MASK_INT1_CFG_ZLIE, value, enabledDict, "EnabledEnum");
	}

	/**
	 * Int 1 generation on Z lower than threshold
	 */
	public boolean getInt1GenerationOnZLowEnabled() {
		return (Boolean) readFromRegisterWithDictionaryMatch(
				REG_RW_INT1_CFG_REG, MASK_INT1_CFG_ZLIE, enabledDict);
	}

	public void setInt1GenerationOnYHighEnabled(boolean value) {
		writeToRegisterWithDictionaryCheck(REG_RW_INT1_CFG_REG,
				MASK_INT1_CFG_YHIE, value, enabledDict, "EnabledEnum");
	}

	/**
	 * Int 1 generation on Y higher than threshold
	 */
	public boolean getInt1GenerationOnYHighEnabled() {
		return (Boolean) readFromRegisterWithDictionaryMatch(
				REG_RW_INT1_CFG_REG, MASK_INT1_CFG_YHIE, enabledDict);
	}

	//
	public void setInt1GenerationOnYLowEnabled(boolean value) {
		writeToRegisterWithDictionaryCheck(REG_RW_INT1_CFG_REG,
				MASK_INT1_CFG_YLIE, value, enabledDict, "EnabledEnum");
	}

	/**
	 * Int 1 generation on Y lower than threshold
	 */
	public boolean getInt1GenerationOnYLowEnabled() {
		return (Boolean) readFromRegisterWithDictionaryMatch(
				REG_RW_INT1_CFG_REG, MASK_INT1_CFG_YLIE, enabledDict);
	}

	public void setInt1GenerationOnXHighEnabled(boolean value) {
		writeToRegisterWithDictionaryCheck(REG_RW_INT1_CFG_REG,
				MASK_INT1_CFG_XHIE, value, enabledDict, "EnabledEnum");
	}

	/**
	 * Int 1 generation on X higher than threshold
	 */
	public boolean getInt1GenerationOnXHighEnabled() {
		return (Boolean) readFromRegisterWithDictionaryMatch(
				REG_RW_INT1_CFG_REG, MASK_INT1_CFG_XHIE, enabledDict);
	}

	public void setInt1GenerationOnXLowEnabled(boolean value) {
		writeToRegisterWithDictionaryCheck(REG_RW_INT1_CFG_REG,
				MASK_INT1_CFG_XLIE, value, enabledDict, "EnabledEnum");
	}

	/**
	 * Int 1 generation on X lower than threshold
	 */
	public boolean getInt1GenerationOnXLowEnabled() {
		return (Boolean) readFromRegisterWithDictionaryMatch(
				REG_RW_INT1_CFG_REG, MASK_INT1_CFG_XLIE, enabledDict);
	}

	/**
	 * Int1 active
	 */
	public boolean getInt1ActiveValue() {
		return (Boolean) readFromRegisterWithDictionaryMatch(
				REG_R_INT1_SRC_REG, MASK_INT1_SRC_IA, enabledDict);
	}

	/**
	 * Z high event occured
	 */
	public boolean getZHighEventOccuredValue() {
		return (Boolean) readFromRegisterWithDictionaryMatch(
				REG_R_INT1_SRC_REG, MASK_INT1_SRC_ZH, enabledDict);
	}

	/**
	 * Z low event occured
	 */
	public boolean getZLowEventOccuredValue() {
		return (Boolean) readFromRegisterWithDictionaryMatch(
				REG_R_INT1_SRC_REG, MASK_INT1_SRC_ZL, enabledDict);
	}

	/**
	 * Y high event occured
	 */
	public boolean getYHighEventOccuredValue() {
		return (Boolean) readFromRegisterWithDictionaryMatch(
				REG_R_INT1_SRC_REG, MASK_INT1_SRC_YH, enabledDict);
	}

	/**
	 * Y low event occured
	 */
	public boolean getYLowEventOccuredValue() {
		return (Boolean) readFromRegisterWithDictionaryMatch(
				REG_R_INT1_SRC_REG, MASK_INT1_SRC_YL, enabledDict);
	}

	/**
	 * X high event occured
	 */
	public boolean getXHighEventOccuredValue() {
		return (Boolean) readFromRegisterWithDictionaryMatch(
				REG_R_INT1_SRC_REG, MASK_INT1_SRC_XH, enabledDict);
	}

	/**
	 * X low event occured
	 */
	public boolean getXLowEventOccuredValue() {
		return (Boolean) readFromRegisterWithDictionaryMatch(
				REG_R_INT1_SRC_REG, MASK_INT1_SRC_XL, enabledDict);
	}

	public void setInt1ThresholdXValue(int value) {
		writeToRegister(REG_RW_INT1_THS_XH, MASK_INT1_THS_H,
				(value & 0x7f00) >> 8);
		writeToRegister(REG_RW_INT1_THS_XL, MASK_INT1_THS_L, value & 0x00ff);
	}

	public void setInt1ThresholdYValue(int value) {
		writeToRegister(REG_RW_INT1_THS_YH, MASK_INT1_THS_H,
				(value & 0x7f00) >> 8);
		writeToRegister(REG_RW_INT1_THS_YL, MASK_INT1_THS_L, value & 0x00ff);
	}

	public void setInt1ThresholdZValue(int value) {
		writeToRegister(REG_RW_INT1_THS_ZH, MASK_INT1_THS_H,
				(value & 0x7f00) >> 8);
		writeToRegister(REG_RW_INT1_THS_ZL, MASK_INT1_THS_L, value & 0x00ff);
	}

	/**
	 * (X,Y,Z) INT1 threshold value
	 */
	public ArrayList<Integer> getInt1ThresholdValues() {
		ArrayList<Integer> list = new ArrayList<Integer>();
		int xh = readFromRegister(REG_RW_INT1_THS_XH, MASK_INT1_THS_H);
		int xl = readFromRegister(REG_RW_INT1_THS_XL, MASK_INT1_THS_L);
		int yh = readFromRegister(REG_RW_INT1_THS_YH, MASK_INT1_THS_H);
		int yl = readFromRegister(REG_RW_INT1_THS_YL, MASK_INT1_THS_L);
		int zh = readFromRegister(REG_RW_INT1_THS_ZH, MASK_INT1_THS_H);
		int zl = readFromRegister(REG_RW_INT1_THS_ZL, MASK_INT1_THS_L);

		list.add(xh * 256 + xl);
		list.add(yh * 256 + yl);
		list.add(zh * 256 + zl);
		return list;
	}

	public void setInt1DurationWaitEnabled(boolean value) {
		writeToRegisterWithDictionaryCheck(REG_RW_INT1_DURATION,
				MASK_INT1_DURATION_WAIT, value, enabledDict, "EnabledEnum");
	}

	/**
	 * Int 1 duration wait
	 */
	public boolean getInt1DurationWaitEnabled() {
		return (Boolean) readFromRegisterWithDictionaryMatch(
				REG_RW_INT1_DURATION, MASK_INT1_DURATION_WAIT, enabledDict);
	}

	//
	public void setInt1DurationValue(int value) {
		writeToRegister(REG_RW_INT1_DURATION, MASK_INT1_DURATION_D, value);
	}

	/**
	 * Int 1 duration value
	 */
	public int getInt1DurationValue() {
		return readFromRegister(REG_RW_INT1_DURATION, MASK_INT1_DURATION_D);
	}

}
