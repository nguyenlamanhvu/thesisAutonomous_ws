#ifndef __BASECONTROL_PUBLIC_H__
#define __BASECONTROL_PUBLIC_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "err_code.h"

err_code_t BaseControl_Init(void);
err_code_t BaseControl_Main(void);

#ifdef __cplusplus
}
#endif

#endif /* __BASECONTROL_PUBLIC_H__ */
