#ifndef __ERR_PORT_H__
#define __ERR_PORT_H__

#ifdef __cplusplus
extern "C" {
#endif

#ifdef STM32F103xB
#include "stm32f1xx.h"
#endif

#if defined(STM32F407xx) || defined(STM32F405xx)
#include "stm32f4xx.h"
#endif

#ifdef STM32L4R5xx
#include "stm32l4xx.h"
#endif

#define ERR_CODE_SUCCESS 	HAL_OK
#define ERR_CODE_FAIL 		HAL_ERROR
#define ERR_CODE_NULL_PTR 	(uint32_t)0xFFFE

typedef uint32_t err_code_t;

#ifdef __cplusplus
}
#endif

#endif  /* __ERR_PORT_H__ */




