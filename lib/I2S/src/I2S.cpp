/*
	Author: bitluni 2019
	License: 
	Creative Commons Attribution ShareAlike 2.0
	https://creativecommons.org/licenses/by-sa/2.0/
	
	For further details check out: 
		https://youtube.com/bitlunislab
		https://github.com/bitluni
		http://bitluni.net
*/
#include "Arduino.h"
#include "FreeRTOS.h"
#include "I2S.h"
#include "Log.h"
#include <soc/rtc.h>
#include <driver/i2c.h>


i2s_dev_t *i2sDevices[] = {&I2S0, &I2S1};

I2S::I2S(const int i2sIndex, QueueHandle_t queue)
{
	this->i2sIndex = i2sIndex;
	this->queue = queue;
	interruptHandle = 0;
	dmaBufferCount = 0;
	dmaBufferActive = 0;
	dmaBuffers = 0;
	stopSignal = false;
}

void IRAM_ATTR I2S::interrupt(void *arg)
{
	volatile i2s_dev_t &i2s = *i2sDevices[((I2S *)arg)->i2sIndex];
	i2s.int_clr.val = i2s.int_raw.val;
	((I2S *)arg)->interrupt();
}

void I2S::interrupt()
{
	// Serial.println("i2s IRQ");
	static int c = 0;
	unsigned short *buf = (unsigned short *)dmaBuffers[dmaBufferActive]->buffer;
	for (int i = 0; i < 16; i++)
		buf[i] = c++;
	dmaBufferActive = (dmaBufferActive + 1) % dmaBufferCount;

	int xHigherPriorityTaskWoken = pdFALSE;  
	xQueueSendFromISR(queue, &dmaBufferActive, &xHigherPriorityTaskWoken);

	if (stopSignal)
	{
		i2sStop();
		stopSignal = false;
	}
}

void I2S::reset()
{
	volatile i2s_dev_t &i2s = *i2sDevices[i2sIndex];
	const unsigned long lc_conf_reset_flags = I2S_IN_RST_M | I2S_OUT_RST_M | I2S_AHBM_RST_M | I2S_AHBM_FIFO_RST_M;
	i2s.lc_conf.val |= lc_conf_reset_flags;
	i2s.lc_conf.val &= ~lc_conf_reset_flags;

	const uint32_t conf_reset_flags = I2S_RX_RESET_M | I2S_RX_FIFO_RESET_M | I2S_TX_RESET_M | I2S_TX_FIFO_RESET_M;
	i2s.conf.val |= conf_reset_flags;
	i2s.conf.val &= ~conf_reset_flags;
	while (i2s.state.rx_fifo_reset_back)
		;
}

void I2S::i2sStop()
{
	volatile i2s_dev_t &i2s = *i2sDevices[i2sIndex];
	esp_intr_disable(interruptHandle);
	reset();
	i2s.conf.rx_start = 0;
	i2s.conf.tx_start = 0;
}

void I2S::startTX()
{
	volatile i2s_dev_t &i2s = *i2sDevices[i2sIndex];
	DEBUG_PRINTLN("I2S TX");
	esp_intr_disable(interruptHandle);
	reset();
	dmaBufferActive = 0;
	DEBUG_PRINT("Sample count ");
	DEBUG_PRINTLN(dmaBuffers[0]->sampleCount());
	i2s.out_link.addr = (uint32_t) & (dmaBuffers[0]->descriptor);
	i2s.out_link.start = 1;
	i2s.int_clr.val = i2s.int_raw.val;
	i2s.int_ena.val = 0;
	i2s.int_ena.out_eof = 1;
	i2s.int_ena.out_dscr_err = 1;
	//enable interrupt
	esp_intr_enable(interruptHandle);
	//start transmission
	i2s.conf.tx_start = 1;
}

void I2S::startRX()
{
	volatile i2s_dev_t &i2s = *i2sDevices[i2sIndex];
	DEBUG_PRINTLN("I2S RX");
	esp_intr_disable(interruptHandle);
	reset();
	dmaBufferActive = 0;
	DEBUG_PRINT("Sample count ");
	DEBUG_PRINTLN(dmaBuffers[0]->sampleCount());
	i2s.rx_eof_num = dmaBuffers[0]->sampleCount();
	i2s.in_link.addr = (uint32_t) & (dmaBuffers[0]->descriptor);
	i2s.in_link.start = 1;
	i2s.int_clr.val = i2s.int_raw.val;
	i2s.int_ena.val = 1; //0;
	i2s.int_ena.in_done = 1;
	esp_intr_enable(interruptHandle);
	i2s.conf.rx_start = 1;
}

void I2S::resetDMA()
{
	volatile i2s_dev_t &i2s = *i2sDevices[i2sIndex];
	i2s.lc_conf.in_rst = 1;
	i2s.lc_conf.in_rst = 0;
	i2s.lc_conf.out_rst = 1;
	i2s.lc_conf.out_rst = 0;
}

void I2S::resetFIFO()
{
	volatile i2s_dev_t &i2s = *i2sDevices[i2sIndex];
	i2s.conf.rx_fifo_reset = 1;
	i2s.conf.rx_fifo_reset = 0;
	i2s.conf.tx_fifo_reset = 1;
	i2s.conf.tx_fifo_reset = 0;
}

bool I2S::initParallelInputMode(const int *pinMap, long sampleRate, int baseClock, int wordSelect)
{
	volatile i2s_dev_t &i2s = *i2sDevices[i2sIndex];
	//route peripherals
	const int deviceBaseIndex[] = {I2S0I_DATA_IN0_IDX, I2S1I_DATA_IN0_IDX};
	const int deviceClockIndex[] = {I2S0I_BCK_IN_IDX, I2S1I_BCK_IN_IDX};
	const int deviceWordSelectIndex[] = {I2S0I_WS_IN_IDX, I2S1I_WS_IN_IDX};
	const periph_module_t deviceModule[] = {PERIPH_I2S0_MODULE, PERIPH_I2S1_MODULE};
	//works only since indices of the pads are sequential
	for (int i = 0; i < 24; i++)
		if (pinMap[i] > -1)
		{
			PIN_FUNC_SELECT(GPIO_PIN_MUX_REG[pinMap[i]], PIN_FUNC_GPIO);
			gpio_set_direction((gpio_num_t)pinMap[i], (gpio_mode_t)GPIO_MODE_DEF_INPUT);
			gpio_matrix_in(pinMap[i], deviceBaseIndex[i2sIndex] + i, false);
		}
	if (baseClock > -1)
		gpio_matrix_in(baseClock, deviceClockIndex[i2sIndex], false);
	if (wordSelect > -1)
		gpio_matrix_in(wordSelect, deviceWordSelectIndex[i2sIndex], false);

	//enable I2S peripheral
	periph_module_enable(deviceModule[i2sIndex]);

	//reset i2s
	i2s.conf.rx_reset = 1;
	i2s.conf.rx_reset = 0;
	i2s.conf.tx_reset = 1;
	i2s.conf.tx_reset = 0;

	resetFIFO();
	resetDMA();

	//parallel mode
	i2s.conf2.val = 0;
	i2s.conf2.lcd_en = 1;
	//from technical datasheet figure 64
	//i2s.conf2.lcd_tx_sdx2_en = 1;
	//i2s.conf2.lcd_tx_wrx2_en = 1;

	i2s.sample_rate_conf.val = 0;
	i2s.sample_rate_conf.rx_bits_mod = 16;

	//maximum rate
	i2s.clkm_conf.val = 0;
	i2s.clkm_conf.clka_en = 0;
	i2s.clkm_conf.clkm_div_num = 6; //3//80000000L / sampleRate;
	i2s.clkm_conf.clkm_div_a = 6;   // 0;
	i2s.clkm_conf.clkm_div_b = 1;   // 0;
	i2s.sample_rate_conf.rx_bck_div_num = 2;

	i2s.fifo_conf.val = 0;
	i2s.fifo_conf.rx_fifo_mod_force_en = 1;
	i2s.fifo_conf.rx_fifo_mod = 1; //byte packing 0A0B_0B0C = 0, 0A0B_0C0D = 1, 0A00_0B00 = 3,
	i2s.fifo_conf.rx_data_num = 32;
	i2s.fifo_conf.dscr_en = 1; //fifo will use dma

	i2s.conf1.val = 0;
	i2s.conf1.tx_stop_en = 1;
	i2s.conf1.tx_pcm_bypass = 1;

	i2s.conf_chan.val = 0;
	i2s.conf_chan.rx_chan_mod = 0;

	//high or low (stereo word order)
	i2s.conf.rx_right_first = 1;

	i2s.timing.val = 0;

	//clear serial mode flags
	i2s.conf.rx_msb_right = 0;
	i2s.conf.rx_msb_shift = 0;
	i2s.conf.rx_mono = 0;
	i2s.conf.rx_short_sync = 0;

	//allocate disabled i2s interrupt
	const int interruptSource[] = {ETS_I2S0_INTR_SOURCE, ETS_I2S1_INTR_SOURCE};
	esp_intr_alloc(interruptSource[i2sIndex], ESP_INTR_FLAG_INTRDISABLED | ESP_INTR_FLAG_LEVEL3 | ESP_INTR_FLAG_IRAM, &interrupt, this, &interruptHandle);
	return true;
}

bool I2S::initParallelOutputMode(const int *pinMap, long sampleRate, int baseClock, int wordSelect)
{
	volatile i2s_dev_t &i2s = *i2sDevices[i2sIndex];
	//route peripherals
	//in parallel mode only upper 16 bits are interesting in this case
	const int deviceBaseIndex[] = {I2S0O_DATA_OUT0_IDX, I2S1O_DATA_OUT0_IDX};
	const int deviceClockIndex[] = {I2S0O_BCK_OUT_IDX, I2S1O_BCK_OUT_IDX};
	const int deviceWordSelectIndex[] = {I2S0O_WS_OUT_IDX, I2S1O_WS_OUT_IDX};
	const periph_module_t deviceModule[] = {PERIPH_I2S0_MODULE, PERIPH_I2S1_MODULE};
	//works only since indices of the pads are sequential
	for (int i = 0; i < 24; i++)
		if (pinMap[i] > -1)
		{
			PIN_FUNC_SELECT(GPIO_PIN_MUX_REG[pinMap[i]], PIN_FUNC_GPIO);
			gpio_set_direction((gpio_num_t)pinMap[i], (gpio_mode_t)GPIO_MODE_DEF_OUTPUT);
			gpio_matrix_out(pinMap[i], deviceBaseIndex[i2sIndex] + i, false, false);
		}
	if (baseClock > -1)
		gpio_matrix_out(baseClock, deviceClockIndex[i2sIndex], false, false);
	if (wordSelect > -1)
		gpio_matrix_out(wordSelect, deviceWordSelectIndex[i2sIndex], false, false);

	//enable I2S peripheral
	periph_module_enable(deviceModule[i2sIndex]);

	//reset i2s
	i2s.conf.tx_reset = 1;
	i2s.conf.tx_reset = 0;
	i2s.conf.rx_reset = 1;
	i2s.conf.rx_reset = 0;

	resetFIFO();
	resetDMA();

	//parallel mode
	i2s.conf2.val = 0;
	i2s.conf2.lcd_en = 1;
	//from technical datasheet figure 64
	//i2s.conf2.lcd_tx_sdx2_en = 1;
	//i2s.conf2.lcd_tx_wrx2_en = 1;

	i2s.sample_rate_conf.val = 0;
	i2s.sample_rate_conf.tx_bits_mod = 16; //16

	//clock setup
	//xtal is 40M
	//chip revision 0
	//fxtal * (sdm2 + 4) / (2 * (odir + 2))
	//chip revision 1
	//fxtal * (sdm2 + (sdm1 / 256) + (sdm0 / 65536) + 4) / (2 * (odir + 2))
	//fxtal * (sdm2 + (sdm1 / 256) + (sdm0 / 65536) + 4) needs to be btween 350M and 500M
	//rtc_clk_apll_enable(enable, sdm0, sdm1, sdm2, odir);
	//                           0-255 0-255  0-63  0-31
	//sdm seems to be simply a fixpoint number with 16bits frational part
	//0xA7f00 is the highest value I was able to use. it's just shy of 580MHz. That's a max freq of 145MHz
	//freq = 40000000L * (4 + sdm) / 4
	//sdm = freq / 10000000L - 4;
	long freq = min(sampleRate, 36249999L) * 8; //there are two 1/2 factors in the I2S pipeline for the frequency and another I missed
	long sdm = long(freq * 0.0065536) - 0x40000;
	rtc_clk_apll_enable(true, sdm & 255, (sdm >> 8) & 255, sdm >> 16, 0);
	i2s.clkm_conf.val = 0;
	i2s.clkm_conf.clka_en = 1;
	i2s.clkm_conf.clkm_div_num = 2; //clockN;
	i2s.clkm_conf.clkm_div_a = 1;   //clockA;
	i2s.clkm_conf.clkm_div_b = 0;   //clockB;
	i2s.sample_rate_conf.tx_bck_div_num = 2;

	i2s.fifo_conf.val = 0;
	i2s.fifo_conf.tx_fifo_mod_force_en = 1;
	i2s.fifo_conf.tx_fifo_mod = 1;  //byte packing 0A0B_0B0C = 0, 0A0B_0C0D = 1, 0A00_0B00 = 3,
	i2s.fifo_conf.tx_data_num = 32; //fifo length
	i2s.fifo_conf.dscr_en = 1;		//fifo will use dma

	i2s.conf1.val = 0;
	i2s.conf1.tx_stop_en = 1;
	i2s.conf1.tx_pcm_bypass = 1;

	i2s.conf_chan.val = 0;
	i2s.conf_chan.tx_chan_mod = 0;

	//high or low (stereo word order)
	i2s.conf.tx_right_first = 1;

	i2s.timing.val = 0;

	//clear serial mode flags
	i2s.conf.tx_msb_right = 0;
	i2s.conf.tx_msb_shift = 0;
	i2s.conf.tx_mono = 0;
	i2s.conf.tx_short_sync = 0;

	//allocate disabled i2s interrupt
	const int interruptSource[] = {ETS_I2S0_INTR_SOURCE, ETS_I2S1_INTR_SOURCE};
	esp_intr_alloc(interruptSource[i2sIndex], ESP_INTR_FLAG_INTRDISABLED | ESP_INTR_FLAG_LEVEL3 | ESP_INTR_FLAG_IRAM, &interrupt, this, &interruptHandle);
	return true;
}

void HandleESPError2(esp_err_t err, String msg) {
  if (err != ESP_OK) {
    Serial.println("ESP ERROR!");
    Serial.println(err);
    Serial.println(msg);
  }
}

bool I2S::initAudioRXMode(const i2s_pin_config_t *pinConfig, const i2s_config_t *i2sConfig/*, int queueSize, void *queue*/) {
	// volatile i2s_dev_t &i2s = *i2sDevices[i2sIndex];
	
	// i2s_driver_install((i2s_port_t)i2sIndex, i2sConfig/*, queueSize, queue*/, 0, NULL);
	HandleESPError2(
		i2s_driver_install((i2s_port_t)i2sIndex, i2sConfig, 0, NULL),
		"failed to install i2s driver");
	HandleESPError2(
		i2s_set_pin((i2s_port_t)i2sIndex, pinConfig),
		"failed to config i2s pins");
	HandleESPError2(
		i2s_set_sample_rates((i2s_port_t)i2sIndex, i2sConfig->sample_rate),
		"failed to set i2s sample rate");

	resetFIFO();
	resetDMA();

	const int interruptSource[] = {ETS_I2S0_INTR_SOURCE, ETS_I2S1_INTR_SOURCE};
	esp_intr_alloc(interruptSource[i2sIndex], ESP_INTR_FLAG_INTRDISABLED | ESP_INTR_FLAG_LEVEL3 | ESP_INTR_FLAG_IRAM, &interrupt, this, &interruptHandle);
	return true;
}

void I2S::getRXAudio(int dmaBufferOffset, int count, int channel, float *audio) {
	int start = dmaBufferOffset - count;
	bool wrap = start < 0;
	int audioIdx = 0;
	if (wrap) {
		start += dmaBufferCount;

		for (int i = start; i < dmaBufferCount; i++) {
			Serial.println(i);
			Serial.println((int)dmaBuffers);
			DMABuffer *buf = dmaBuffers[i];
			const int samples = buf->sampleCount();
			for (int j = channel; j < samples; j+=2) {
				Serial.println(j);
				// this only works for 24 bit left aligned input
				audio[audioIdx] = (float)(*(int*)(buf->buffer + 4*j) >> 8);
				audioIdx++;
			}
		}

		if (dmaBufferOffset > 0) {
			for (int i = 0; i < dmaBufferOffset; i++) {
				DMABuffer *buf = dmaBuffers[i];
				const int samples = buf->sampleCount();
				for (int j = channel; j < samples; j+=2) {
					Serial.println(j);
					// this only works for 24 bit left aligned input
					audio[audioIdx] = (float)(*(int*)(buf->buffer + 4*j) >> 8);
					audioIdx++;
				}
			}
		}
	} else {
		for (int i = start; i < dmaBufferOffset; i++) {
			DMABuffer *buf = dmaBuffers[i];
			const int samples = buf->sampleCount();
			for (int j = channel; j < samples; j+=2) {
				Serial.println(j);
				// this only works for 24 bit left aligned input
				audio[audioIdx] = (float)(*(int*)(buf->buffer + 4*j) >> 8);
				audioIdx++;
			}
		}
	}
}

/// simple ringbuffer of blocks of size bytes each
void I2S::allocateDMABuffers(int count, int bytes)
{
	dmaBufferCount = count;
	dmaBuffers = (DMABuffer **)malloc(sizeof(DMABuffer *) * dmaBufferCount);
	if (!dmaBuffers)
		DEBUG_PRINTLN("Failed to allocate DMABuffer array");
	for (int i = 0; i < dmaBufferCount; i++)
	{
		dmaBuffers[i] = DMABuffer::allocate(bytes);
		if (i)
			dmaBuffers[i - 1]->next(dmaBuffers[i]);
	}
	dmaBuffers[dmaBufferCount - 1]->next(dmaBuffers[0]);
}

void I2S::deleteDMABuffers()
{
	if (!dmaBuffers)
		return;
	for (int i = 0; i < dmaBufferCount; i++)
		dmaBuffers[i]->destroy();
	free(dmaBuffers);
	dmaBuffers = 0;
	dmaBufferCount = 0;
}

void I2S::stop()
{
	stopSignal = true;
	while (stopSignal)
		;
}