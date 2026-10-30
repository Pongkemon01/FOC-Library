// foc-lib port configuration for the STM32F303RE (Nucleo-F303RE + X-NUCLEO-IHM08M1).
// Included by foc_lib.h when built with -DFOC_LIB_USE_CONF (platformio.ini).
#ifndef FOC_LIB_CONF_H
#define FOC_LIB_CONF_H

// Run FOC_Update's call tree (and the ADC callback in Src/freertos.c) from CCM SRAM:
// 0 wait states, against 2 for flash at 72 MHz. The stock linker script maps .ccmram* to
// CCMRAM with a flash load address, but the startup file copies only .data, so main()
// (USER CODE 1) copies _siccmram -> _sccmram.._eccmram before HAL_Init.
#define FOC_FAST_CODE __attribute__((section(".ccmram.text")))

// Board, sensing and motor constants: this setup is foc_lib.h's reference, so its defaults
// apply and none are overridden here. Another board or motor defines them here instead, e.g.
//   #define R_SHUNT 0.005f
//   #define CURRENT_SENSE_POLARITY (1.0f)
//   #define POLE_PAIRS 4
// HFI (standstill start, low speed) is always on. Only a motor with too little saliency for HFI
// would turn it off: Align -> I-f start and the eSMO only.
//   #define FOC_HFI 0

#endif // FOC_LIB_CONF_H
