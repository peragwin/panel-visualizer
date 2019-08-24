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
#pragma once

#include "esp_heap_caps.h"
#include "soc/soc.h"
#include "soc/gpio_sig_map.h"
#include "soc/i2s_reg.h"
#include "soc/i2s_struct.h"
#include "soc/io_mux_reg.h"
#include "driver/gpio.h"
#include "driver/periph_ctrl.h"
#include "driver/i2s.h"
#include "rom/lldesc.h"
#include "DMABuffer.h"

#include "Arduino.h"
#include "FreeRTOS.h"

class I2S
{
  public:
	int i2sIndex;
	intr_handle_t interruptHandle;
	int dmaBufferCount;
	int dmaBufferActive;
	DMABuffer **dmaBuffers;
	volatile bool stopSignal;

	QueueHandle_t queue;

	/// hardware index [0, 1]
	I2S(const int i2sIndex = 0, QueueHandle_t queue = NULL);
	void reset();

	void stop();

	void i2sStop();
	void startTX();
	void startRX();

	void resetDMA();
	void resetFIFO();
	bool initParallelOutputMode(const int *pinMap, long APLLFreq = 1000000, int baseClock = -1, int wordSelect = -1);
	bool initParallelInputMode(const int *pinMap, long sampleRate = 1000000, int baseClock = -1, int wordSelect = -1);

	bool initAudioRXMode(const i2s_pin_config_t *pinConfig, const i2s_config_t *i2sConfig);
	void getRXAudio(int dmaBufferOffset, int count, int channel, float *audio);

	void allocateDMABuffers(int count, int bytes);
	void deleteDMABuffers();

  protected:
	virtual void interrupt();
	
  private:
	static void IRAM_ATTR interrupt(void *arg);
};