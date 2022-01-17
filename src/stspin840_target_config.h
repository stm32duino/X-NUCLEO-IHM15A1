/**************************************************************************//**
  * @file    stspin840_target_config.h
  * @author  STMicroelectronics
  * @version V1.0.0
  * @date    January 12th, 2022
  * @brief   Predefines values for the STSPIN840 parameters
  * and for the devices parameters
  * @note    (C) COPYRIGHT 2022 STMicroelectronics
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT(c) 2022 STMicroelectronics</center></h2>
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __STSPIN840_TARGET_CONFIG_H
#define __STSPIN840_TARGET_CONFIG_H

#ifdef __cplusplus
extern "C" {
#endif

/** @addtogroup BSP
  * @{
  */

/** @addtogroup STSPIN840
  * @{
  */

/** @addtogroup STSPIN840_Exported_Constants STSPIN840 Exported Constants
  * @{
  */

/** @defgroup Predefined_STSPIN840_Parameters_Values Predefined STSPIN840 Parameters Values
  * @{
  */

/// The maximum number of STSPIN840 devices
#define MAX_NUMBER_OF_DEVICES                 (1)

/// The maximum number of Brush DC motors connected to the STSPIN840
#define MAX_NUMBER_OF_BRUSH_DC_MOTORS                    (2)

/// Frequency of PWM of Input Bridge A in Hz up to 100000Hz
#define STSPIN840_CONF_PARAM_FREQ_PWM_A  (20000)

/// Frequency of PWM of Input Bridge B in Hz up to 100000Hz
/// On the X-NUCLEO-IHM01A15A1 expansion board the PWM_A and PWM_B
/// share the same timer, so the frequency must be the same
#define STSPIN840_CONF_PARAM_FREQ_PWM_B  (20000)

/// Frequency of PWM used for RefA pin in Hz up to 100000Hz
#define STSPIN840_CONF_PARAM_FREQ_PWM_REF_A  (20000)

/// Frequency of PWM used for RefB pin in Hz up to 100000Hz
/// On the X-NUCLEO-IHM01A15A1 expansion board the REF_A and REF_B
/// share the same timer, so the frequency must be the same
#define STSPIN840_CONF_PARAM_FREQ_PWM_REF_B  (20000)

/// Duty cycle of PWM used for RefA pin (from 0 to 100)
#define STSPIN840_CONF_PARAM_DC_PWM_REF_A  (50)

/// Duty cycle of PWM used for RefB pin (from 0 to 100)
#define STSPIN840_CONF_PARAM_DC_PWM_REF_B  (50)


/**
  * @}
  */

/**
  * @}
  */

/**
  * @}
  */

/**
  * @}
  */

#ifdef __cplusplus
}
#endif

#endif /* __STSPIN840_TARGET_CONFIG_H */
