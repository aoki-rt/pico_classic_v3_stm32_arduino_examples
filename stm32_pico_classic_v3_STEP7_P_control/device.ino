// Copyright 2025 RT Corporation
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.


uint16_t RTanalogRead(uint32_t ulPin){
  ADC_HandleTypeDef AdcHandle = {};
  ADC_ChannelConfTypeDef AdcChannelConf = {};
  __IO uint16_t uhADCxConvertedValue = 0;
  uint32_t bank = 0;

  PinName pin = analogInputToPinName(ulPin);

  AdcHandle.Instance = (ADC_TypeDef *)pinmap_peripheral(pin, PinMap_ADC);
  AdcHandle.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  AdcHandle.Init.Resolution = ADC_RESOLUTION_12B;
  AdcHandle.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  AdcHandle.Init.GainCompensation = 0;
  AdcHandle.Init.ScanConvMode = DISABLE;
  AdcHandle.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  AdcHandle.Init.LowPowerAutoWait = DISABLE;
  AdcHandle.Init.ContinuousConvMode = DISABLE;
  AdcHandle.Init.NbrOfConversion = 1;
  AdcHandle.Init.DiscontinuousConvMode = DISABLE;
  AdcHandle.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  AdcHandle.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  AdcHandle.Init.DMAContinuousRequests = DISABLE;
  AdcHandle.Init.Overrun = ADC_OVR_DATA_PRESERVED;
  AdcHandle.Init.OversamplingMode = DISABLE;

  AdcHandle.State = HAL_ADC_STATE_RESET;
  AdcHandle.DMA_Handle = NULL;
  AdcHandle.Lock = HAL_UNLOCKED;

  if (HAL_ADC_Init(&AdcHandle) != HAL_OK) {
    return 0;
  }

  pinmap_pinout(pin, PinMap_ADC);

  AdcChannelConf.Channel = get_adc_channel(pin, &bank);
  AdcChannelConf.Rank = ADC_REGULAR_RANK_1;
  AdcChannelConf.SamplingTime = ADC_SAMPLETIME_47CYCLES_5;
  AdcChannelConf.SingleDiff = ADC_SINGLE_ENDED;
  AdcChannelConf.OffsetNumber = ADC_OFFSET_NONE;
  AdcChannelConf.Offset = 0;
  if (HAL_ADC_ConfigChannel(&AdcHandle, &AdcChannelConf) != HAL_OK) {
    return 0;
  }

  if (HAL_ADC_Start(&AdcHandle) != HAL_OK) {
    return 0;
  }

  if (HAL_ADC_PollForConversion(&AdcHandle, 10) != HAL_OK) {
    return 0;
  }

  if ((HAL_ADC_GetState(&AdcHandle) & HAL_ADC_STATE_REG_EOC) == HAL_ADC_STATE_REG_EOC) {
    uhADCxConvertedValue = HAL_ADC_GetValue(&AdcHandle);
  }

  if (HAL_ADC_Stop(&AdcHandle) != HAL_OK) {
    return 0;
  }

  return uhADCxConvertedValue;
}
