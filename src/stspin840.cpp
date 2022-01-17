/**
  ******************************************************************************
  * @file    stspin840.cpp
  * @author  STMicroelectronics
  * @version V1.0.0
  * @date    January 12th, 2022
  * @brief   Stspin840 driver
  * @note     (C) COPYRIGHT 2022 STMicroelectronics
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

/* Includes ------------------------------------------------------------------*/
#include "stspin840.h"

/* Private constants ---------------------------------------------------------*/

/// The Number of Stspin840 devices required for initialisation is not supported
#define STSPIN840_ERROR_0   (0xC000)
/// Error: Access a motor index greater than the one of the current bridge configuration
#define STSPIN840_ERROR_1   (0xC001)
/// Error: Use of a bridgeId greater than BRIDGE_B
#define STSPIN840_ERROR_2   (0xC002)

/// Maximum frequency of the PWMs in Hz
#define STSPIN840_MAX_PWM_FREQ   (100000)

/// Minimum frequency of the PWMs in Hz
#define STSPIN840_MIN_PWM_FREQ   (2)

/* Private variables ---------------------------------------------------------*/
uint8_t Stspin840::numberOfDevices = 0;

/* Private function prototypes -----------------------------------------------*/

/******************************************************//**
 * @brief  Attaches a user callback to the error Handler.
 * The call back will be then called each time the library
 * detects an error
 * @param[in] callback Name of the callback to attach
 * to the error Handler
 * @retval None
 **********************************************************/
void Stspin840::Stspin840_AttachErrorHandler(void (*callback)(uint16_t))
{
  errorHandlerCallback = (void (*)(uint16_t))callback;
}

/******************************************************//**
 * @brief Disable the specified bridge
 * @param[in] bridgeId (from 0 for bridge A to 1 for bridge B)
 * @retval None
 * @note  Bridge A and bridge B share the same enable pin.
 * When bridge A is disabled, bridge B is disabled and
 * reversely
 **********************************************************/
void Stspin840::Stspin840_DisableBridge(uint8_t bridgeId)
{
  Stspin840_Board_DisableBridge(bridgeId);
  devicePrm.bridgeEnabled[bridgeId] = FALSE;
}

/******************************************************//**
 * @brief Enable the specified bridge
 * @param[in] bridgeId (from 0 for bridge A to 1 for bridge B)
 * @retval None
 * @note  Bridge A and bridge B share the same enable pin.
 * When bridge A is enabled, bridge B is enabled and
 * reversely
 **********************************************************/
void Stspin840::Stspin840_EnableBridge(uint8_t bridgeId)
{
  Stspin840_Board_EnableBridge(bridgeId, 1);
  devicePrm.bridgeEnabled[bridgeId] = TRUE;
}


/******************************************************//**
 * @brief Get the motor current direction
 * @param[in] motorId from 0 to (MAX_NUMBER_OF_BRUSH_DC_MOTORS - 1)
 * @retval direction
 **********************************************************/
motorDir_t Stspin840::Stspin840_GetDirection(uint8_t motorId)
{
  if (motorId >= MAX_NUMBER_OF_BRUSH_DC_MOTORS) {
    Stspin840_ErrorHandler(STSPIN840_ERROR_1);
  }

  return devicePrm.direction[motorId];
}

/******************************************************//**
 * @brief Starts the Stspin840 library
 * @param[in] pInit pointer to the initialization data
 * @retval None
 * @note  Only one device is currently supported
 **********************************************************/
void Stspin840::Stspin840_Init(void *pInit)
{
  if (pInit == 0) {
    /* Set context variables to the predefined values from stspin840_target_config.h */
    Stspin840_SetDeviceParamsToPredefinedValues();
  } else {
    /* Set context variables to the predefined values from stspin840_target_config.h */
    Stspin840_SetDeviceParamsToGivenValues(stspin840DriverInstance, (Stspin840_Init_t *)pInit);
  }
  /* Initialise PWM for REFA/B pins */
  Stspin840_Board_PwmInit(PWM_REF_A, 0);
  Stspin840_Board_PwmInit(PWM_REF_B, 0);

  /* Deinit PWM input bridges by stopping them */
  Stspin840_Board_PwmStop(0);
  Stspin840_Board_PwmStop(1);

  stspin840DriverInstance++;

  if (stspin840DriverInstance > MAX_NUMBER_OF_DEVICES) {
    /* Initialization Error */
    Stspin840_ErrorHandler(STSPIN840_ERROR_0);
  }
}

/******************************************************//**
 * @brief  Get the PWM frequency of the specified bridge
 * @param[in] bridgeId 0 for bridge A, 1 for bridge B
 * @retval Freq in Hz
 **********************************************************/
uint32_t Stspin840::Stspin840_GetBridgeInputPwmFreq(uint8_t bridgeId)
{
  if (bridgeId > BRIDGE_B) {
    Stspin840_ErrorHandler(STSPIN840_ERROR_2);
  }

  return (devicePrm.bridgePwmFreq[(bridgeId << 1)]);
}

/******************************************************//**
 * @brief  Get the status of the bridge enabling of the corresponding bridge
 * @param[in] bridgeId from 0 for bridge A to 1 for bridge B
  **********************************************************/
uint16_t Stspin840::Stspin840_GetBridgeStatus(uint8_t bridgeId)
{
  uint16_t status = (uint16_t)Stspin840_Board_GetFaultPinState(bridgeId);

  return (status);
}

/******************************************************//**
 * @brief  Returns the current speed of the specified motor
 * @param[in] motorId from 0 to (MAX_NUMBER_OF_BRUSH_DC_MOTORS - 1)
 * @retval current speed in % from 0 to 100
 **********************************************************/
uint16_t Stspin840::Stspin840_GetCurrentSpeed(uint8_t motorId)
{
  uint16_t speed = 0;

  if (motorId >= MAX_NUMBER_OF_BRUSH_DC_MOTORS) {
    Stspin840_ErrorHandler(STSPIN840_ERROR_1);
  } else if (devicePrm.motionState[motorId] != INACTIVE) {
    speed = devicePrm.speed[motorId];
  }

  return (speed);
}

/******************************************************//**
 * @brief Returns the device state
 * @param[in] motorId from 0 to (MAX_NUMBER_OF_BRUSH_DC_MOTORS - 1)
 * @retval State (STEADY or INACTIVE)
 **********************************************************/
motorState_t Stspin840::Stspin840_GetDeviceState(uint8_t motorId)
{
  motorState_t state =  INACTIVE;

  if (motorId >= MAX_NUMBER_OF_BRUSH_DC_MOTORS) {
    Stspin840_ErrorHandler(STSPIN840_ERROR_1);
  } else {
    state =  devicePrm.motionState[motorId];
  }
  return (state);
}

/******************************************************//**
 * @brief Returns the FW version of the library
 * @retval STSPIN840_FW_VERSION
 **********************************************************/
uint32_t Stspin840::Stspin840_GetFwVersion(void)
{
  return (STSPIN840_FW_VERSION);
}

/******************************************************//**
 * @brief  Returns the max  speed of the specified motor
 * @param[in] motorId from 0 to (MAX_NUMBER_OF_BRUSH_DC_MOTORS - 1)
 * @retval maxSpeed in % from 0 to 100
 **********************************************************/
uint16_t Stspin840::Stspin840_GetMaxSpeed(uint8_t motorId)
{
  uint16_t speed = 0;
  if (motorId >= MAX_NUMBER_OF_BRUSH_DC_MOTORS) {
    Stspin840_ErrorHandler(STSPIN840_ERROR_1);
  } else {
    speed =  devicePrm.speed[motorId];
  }
  return (speed);
}

/******************************************************//**
* @brief  Returns the number of devices
* @retval number of devices
**********************************************************/
uint8_t Stspin840::Stspin840_GetNbDevices(void)
{
  return (numberOfDevices);
}

/******************************************************//**
 * @brief  Return the duty cycle of PWM used for REF
 * @param[in] refId 0 for bridge A, 1 for bridge B
 * @retval duty cycle in % (from 0 to 100)
 **********************************************************/
uint8_t Stspin840::Stspin840_GetRefPwmDc(uint8_t refId)
{
  uint32_t duty = 0;

  if (duty == 0) {
    duty = devicePrm.refPwmDc[refId];
  }
  return (duty);
}

/******************************************************//**
 * @brief  Return the frequency of PWM used for REF
 * @param[in] refId 0 for bridge A, 1 for bridge B
 * @retval Frequency in Hz
 **********************************************************/
uint32_t Stspin840::Stspin840_GetRefPwmFreq(uint8_t refId)
{
  uint32_t freq = 0;

  if (refId == 0) {
    freq = devicePrm.refPwmFreq[refId];
  }
  return (freq);
}

/******************************************************//**
 * @brief  Immediately stops the motor and disable the power bridge
 * @param[in] motorId from 0 to (MAX_NUMBER_OF_BRUSH_DC_MOTORS - 1)
 * @retval None
 **********************************************************/
void Stspin840::Stspin840_HardHiz(uint8_t motorId)
{
  if (motorId >= MAX_NUMBER_OF_BRUSH_DC_MOTORS) {
    Stspin840_ErrorHandler(STSPIN840_ERROR_1);
  } else {
    if (devicePrm.bridgeEnabled[motorId] != FALSE) {
      /* Disable the bridge */
      Stspin840_DisableBridge(motorId);
    }
    /* Disable the PWM */
    Stspin840_HardStop(motorId);
  }
}

/******************************************************//**
 * @brief  Stops the motor without disabling the bridge
 * @param[in] motorId from 0 to (MAX_NUMBER_OF_BRUSH_DC_MOTORS - 1)
 * @retval none
 **********************************************************/
void Stspin840::Stspin840_HardStop(uint8_t motorId)
{
  if (motorId >= MAX_NUMBER_OF_BRUSH_DC_MOTORS) {
    Stspin840_ErrorHandler(STSPIN840_ERROR_1);
  } else if (devicePrm.motionState[motorId] != INACTIVE) {
    /* Disable corresponding PWM */
    Stspin840_Board_PwmStop(motorId);

    /* Set inactive state */
    devicePrm.motionState[motorId] = INACTIVE;
  }
}

/******************************************************//**
 * @brief Read id
 * @retval Id of the Stspin840 Driver Instance
 **********************************************************/
uint16_t Stspin840::Stspin840_ReadId(void)
{
  return (stspin840DriverInstance);
}

/******************************************************//**
 * @brief Release reset (exit standby mode)
 * @retval None
 **********************************************************/
void Stspin840::Stspin840_ReleaseReset(void)
{
  Stspin840_Board_ReleaseReset();

  /* Start PWM used for REFA/B pins */
  Stspin840_Board_PwmSetFreq(PWM_REF_A, devicePrm.refPwmFreq[0], devicePrm.refPwmDc[0]);
  Stspin840_Board_PwmSetFreq(PWM_REF_B, devicePrm.refPwmFreq[1], devicePrm.refPwmDc[1]);
}

/******************************************************//**
 * @brief Reset (enter standby mode)
 * @retval None
 **********************************************************/
void Stspin840::Stspin840_Reset()
{
  uint8_t loop;
  for (loop = 0; loop < STSPIN840_NB_MAX_MOTORS; loop++) {
    /* Stop motor if needed*/
    if (devicePrm.motionState[loop] != INACTIVE) {
      Stspin840_HardStop(loop);
    }
    /* Disable bridge if needed */
    if (devicePrm.bridgeEnabled[loop] != FALSE) {
      Stspin840_DisableBridge(loop);
    }
  }

  /* Stop PWM used for REFA/B pins */
  Stspin840_Board_PwmStop(PWM_REF_A);
  Stspin840_Board_PwmStop(PWM_REF_B);

  /* Reset the STBY/RESET pin */
  Stspin840_Board_Reset();
}

/******************************************************//**
 * @brief  Runs the motor
 * @param[in] motorId from 0 to (MAX_NUMBER_OF_BRUSH_DC_MOTORS - 1)
 * @param[in] direction FORWARD or BACKWARD
 * @retval None
 * @note  For unidirectionnal motor, direction parameter has
 * no effect
 **********************************************************/
void Stspin840::Stspin840_Run(uint8_t motorId, motorDir_t direction)
{
  if (motorId >= MAX_NUMBER_OF_BRUSH_DC_MOTORS) {
    Stspin840_ErrorHandler(STSPIN840_ERROR_1);
  } else if ((devicePrm.motionState[motorId] == INACTIVE) ||
             (devicePrm.direction[motorId] != direction)) {

    /* Release reset if required */
    if (Stspin840_GetResetState() == 0) {
      Stspin840_ReleaseReset();
    }

    /* Eventually deactivate motor */
    if (devicePrm.motionState[motorId] != INACTIVE) {
      Stspin840_HardStop(motorId);
    }

    /* Set direction */
    Stspin840_SetDirection(motorId, direction);

    /* Switch to steady state */
    devicePrm.motionState[motorId] = STEADY;

    /* Enable bridge */
    if (devicePrm.bridgeEnabled[motorId] == FALSE) {
      Stspin840_EnableBridge(motorId);
    }
    /* Set PWM */
    Stspin840_Board_PwmSetFreq(motorId, devicePrm.bridgePwmFreq[motorId], devicePrm.speed[motorId]);
  }
}

/******************************************************//**
 * @brief  Specifies the direction
 * @param[in] motorId from 0 to (MAX_NUMBER_OF_BRUSH_DC_MOTORS - 1)
 * @param[in] dir FORWARD or BACKWARD
 * @note The direction change is only applied if the device
 * is in INACTIVE state. To change direction while motor is
 * running, use the Run function
 * @retval None
 **********************************************************/
void Stspin840::Stspin840_SetDirection(uint8_t motorId, motorDir_t dir)
{
  if (motorId >= MAX_NUMBER_OF_BRUSH_DC_MOTORS) {
    Stspin840_ErrorHandler(STSPIN840_ERROR_1);
  } else if (devicePrm.motionState[motorId] == INACTIVE) {
    Stspin840_Board_SetDirectionGpio(motorId, dir);
    devicePrm.direction[motorId] = dir;
  }
}

/******************************************************//**
 * @brief  Changes the PWM frequency of the bridge input
 * @param[in] bridgeId 0 for bridge A, 1 for bridge B
 * @param[in] newFreq in Hz
 * @retval None
 * @note 1)The PWM is only enabled when the motor is requested
 * to run.
 * 2) If the two bridges share the same timer, their frequency
 * has to be the same
 * 3) If the two bridges share the same timer, the frequency
 * is updated on the fly if there is only one motor running
 * on the targeted bridge.
 **********************************************************/
void Stspin840::Stspin840_SetBridgeInputPwmFreq(uint8_t bridgeId, uint32_t newFreq)
{
  if (bridgeId > BRIDGE_B) {
    Stspin840_ErrorHandler(STSPIN840_ERROR_2);
  }

  if (newFreq > STSPIN840_MAX_PWM_FREQ) {
    newFreq = STSPIN840_MAX_PWM_FREQ;
  }
  devicePrm.bridgePwmFreq[bridgeId] = newFreq;
  Stspin840_Board_PwmSetFreq(bridgeId, devicePrm.bridgePwmFreq[bridgeId], devicePrm.speed[bridgeId]);
}

/******************************************************//**
 * @brief  Changes the max speed of the specified device
 * @param[in] motorId from 0 to (MAX_NUMBER_OF_BRUSH_DC_MOTORS - 1)
 * @param[in] newMaxSpeed in % from 0 to 100
 * @retval true if the command is successfully executed, else false
 **********************************************************/
bool Stspin840::Stspin840_SetMaxSpeed(uint8_t motorId, uint16_t newMaxSpeed)
{
  bool cmdExecuted = FALSE;

  if (motorId >= MAX_NUMBER_OF_BRUSH_DC_MOTORS) {
    Stspin840_ErrorHandler(STSPIN840_ERROR_1);
  } else {
    devicePrm.speed[motorId] = newMaxSpeed;
    /* Set PWM DC*/
    Stspin840_Board_PwmSetFreq(motorId, devicePrm.bridgePwmFreq[motorId], devicePrm.speed[motorId]);

    cmdExecuted = TRUE;
  }
  return cmdExecuted;
}

/******************************************************//**
 * @brief  Changes the duty cycle of the PWM used for REF
 * @param[in] refId 0 for bridge A, 1 for bridge B
 * @param[in] newDc new duty cycle from 0 to 100
 * @retval None
 **********************************************************/
void Stspin840::Stspin840_SetRefPwmDc(uint8_t refId, uint8_t newDc)
{
  if (refId > BRIDGE_B) {
    Stspin840_ErrorHandler(STSPIN840_ERROR_2);
  }

  if (newDc > 100) {
    newDc = 100;
  }

  devicePrm.refPwmDc[refId] = newDc;
  /* Immediately set the PWM frequency  for ref */
  Stspin840_Board_PwmSetFreq((refId + 2), devicePrm.refPwmFreq[refId], devicePrm.refPwmDc[refId]);

}
/******************************************************//**
 * @brief  Changes the frequency of PWM used for REF
 * @param[in] refId 0 for bridge A, 1 for bridge B
 * @param[in] newFreq in Hz
 * @retval None
 **********************************************************/
void Stspin840::Stspin840_SetRefPwmFreq(uint8_t refId, uint32_t newFreq)
{
  if (newFreq > STSPIN840_MAX_PWM_FREQ) {
    newFreq = STSPIN840_MAX_PWM_FREQ;
  }

  devicePrm.refPwmFreq[refId] = newFreq;
  /* Immediately set the PWM frequency  for ref */
  Stspin840_Board_PwmSetFreq((refId + 2), devicePrm.refPwmFreq[refId], devicePrm.refPwmDc[refId]);

}

/******************************************************//**
 * @brief Error handler which calls the user callback (if defined)
 * @param[in] error Number of the error
 * @retval None
 **********************************************************/
void Stspin840::Stspin840_ErrorHandler(uint16_t error)
{
  if (errorHandlerCallback != 0) {
    (void) errorHandlerCallback(error);
  } else {
    while (1) {
      /* Infinite loop */
    }
  }
}

/******************************************************//**
 * @brief  Handlers of the fault interrupt which calls the user callback (if defined)
 * @retval None
 **********************************************************/
void Stspin840::Stspin840_FaultInterruptHandler(void)
{
  bool status;

  status = Stspin840_GetBridgeStatus(BRIDGE_A);
  if (status != devicePrm.bridgeEnabled[BRIDGE_A]) {
    devicePrm.bridgeEnabled[BRIDGE_A] = status;
  }
  status = Stspin840_GetBridgeStatus(BRIDGE_B);
  if (status != devicePrm.bridgeEnabled[BRIDGE_B]) {
    devicePrm.bridgeEnabled[BRIDGE_B] = status;
  }
  if (int_cb != 0) {
    int_cb();
  }
}

/******************************************************//**
 * @brief  Get the status of the bridge enabling of the corresponding bridge
 * @retval State of the Enable&Fault pin (shared for bridge A and B)
  **********************************************************/
uint8_t Stspin840::Stspin840_GetResetState(void)
{
  uint8_t status = Stspin840_Board_GetResetPinState();

  return (status);
}

/******************************************************//**
 * @brief  Set the parameters of the device to values of pInitPrm structure
 * @param[in] deviceId (from 0 to MAX_NUMBER_OF_DEVICES -1)
 * @param pInitPrm pointer to a structure containing the initial device parameters
 * @retval None
 **********************************************************/
void Stspin840::Stspin840_SetDeviceParamsToGivenValues(uint8_t deviceId, Stspin840_Init_t *pInitPrm)
{
  uint32_t i;

  devicePrm.bridgePwmFreq[BRIDGE_A] = pInitPrm->bridgePwmFreq[BRIDGE_A];
  devicePrm.bridgePwmFreq[BRIDGE_B] = pInitPrm->bridgePwmFreq[BRIDGE_B];;
  devicePrm.refPwmFreq[BRIDGE_A] = pInitPrm->refPwmFreq[BRIDGE_A];
  devicePrm.refPwmDc[BRIDGE_A] = pInitPrm->refPwmDc[BRIDGE_A];
  devicePrm.refPwmFreq[BRIDGE_B] = pInitPrm->refPwmFreq[BRIDGE_B];
  devicePrm.refPwmDc[BRIDGE_B] = pInitPrm->refPwmDc[BRIDGE_B];

  for (i = 0; i < MAX_NUMBER_OF_BRUSH_DC_MOTORS; i++) {
    devicePrm.speed[i] = 100;
    devicePrm.direction[i] = FORWARD;
    devicePrm.motionState[i] = INACTIVE;
  }
  for (i = 0; i < STSPIN840_NB_BRIDGES; i++) {
    devicePrm.bridgeEnabled[i] = FALSE;
  }
}
/******************************************************//**
 * @brief  Sets the parameters of the device to predefined values
 * from stspin840_target_config.h
 * @retval None
 **********************************************************/
void Stspin840::Stspin840_SetDeviceParamsToPredefinedValues(void)
{
  uint32_t i;

  devicePrm.bridgePwmFreq[BRIDGE_A] = STSPIN840_CONF_PARAM_FREQ_PWM_A;
  devicePrm.bridgePwmFreq[BRIDGE_B] = STSPIN840_CONF_PARAM_FREQ_PWM_B;
  devicePrm.refPwmFreq[BRIDGE_A] = STSPIN840_CONF_PARAM_FREQ_PWM_REF_A;
  devicePrm.refPwmDc[BRIDGE_A] = STSPIN840_CONF_PARAM_DC_PWM_REF_A;
  devicePrm.refPwmFreq[BRIDGE_B] = STSPIN840_CONF_PARAM_FREQ_PWM_REF_B;
  devicePrm.refPwmDc[BRIDGE_B] = STSPIN840_CONF_PARAM_DC_PWM_REF_B;

  for (i = 0; i < MAX_NUMBER_OF_BRUSH_DC_MOTORS; i++) {
    devicePrm.speed[i] = 100;
    devicePrm.direction[i] = FORWARD;
    devicePrm.motionState[i] = INACTIVE;
  }
  for (i = 0; i < STSPIN840_NB_BRIDGES; i++) {
    devicePrm.bridgeEnabled[i] = FALSE;
  }
}

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
