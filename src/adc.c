#include "adc.h"
#include "stm32f1xx_ll_adc.h"
#include "stm32f1xx_ll_bus.h"

uint16_t read_adc(uint32_t channel) {
    LL_ADC_REG_SetSequencerRanks(ADC1, LL_ADC_REG_RANK_1, channel);
    LL_ADC_REG_StartConversionSWStart(ADC1);

    while (LL_ADC_IsActiveFlag_EOS(ADC1) == 0){};
	LL_ADC_ClearFlag_EOS(ADC1);
    uint16_t adc_value = LL_ADC_REG_ReadConversionData12(ADC1);
    return adc_value;
}

void adc_init(void) {
  LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_ADC1);

  LL_ADC_SetDataAlignment(ADC1, LL_ADC_DATA_ALIGN_RIGHT);
  LL_ADC_SetSequencersScanMode(ADC1, LL_ADC_SEQ_SCAN_DISABLE);

  LL_ADC_SetChannelSamplingTime(ADC1, LL_ADC_CHANNEL_0,
                                LL_ADC_SAMPLINGTIME_55CYCLES_5);
  LL_ADC_SetChannelSamplingTime(ADC1, LL_ADC_CHANNEL_1,
                                LL_ADC_SAMPLINGTIME_55CYCLES_5);

  LL_ADC_Enable(ADC1);

  if (LL_ADC_IsEnabled(ADC1) == 0) {
    LL_ADC_StartCalibration(ADC1);
    while (LL_ADC_IsCalibrationOnGoing(ADC1))
      ;
  }
}
