/**
  ******************************************************************************
  * @file    stspin840.h
  * @author  STMicroelectronics
  * @version V1.0.0
  * @date    January 12th, 2022
  * @brief   Header for Stspin840 driver
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
#ifndef __STSPIN840_H
#define __STSPIN840_H

/* Includes ------------------------------------------------------------------*/
#include "Arduino.h"
#include "stspin840_target_config.h"
#include "motor_def.h"
#include "BDCMotor.h"

/* Exported Constants --------------------------------------------------------*/

/// Current FW major version
#define STSPIN840_FW_MAJOR_VERSION (uint8_t)(1)
/// Current FW minor version
#define STSPIN840_FW_MINOR_VERSION (uint8_t)(0)
/// Current FW patch version
#define STSPIN840_FW_PATCH_VERSION (uint8_t)(0)
/// Current FW version
#define STSPIN840_FW_VERSION (uint32_t)((STSPIN840_FW_MAJOR_VERSION<<16)|\
                                            (STSPIN840_FW_MINOR_VERSION<<8)|\
                                            (STSPIN840_FW_PATCH_VERSION))

///Max number of Brush DC motors
#define STSPIN840_NB_MAX_MOTORS (2)
///Number of Bridges
#define STSPIN840_NB_BRIDGES (2)

/// Bridge A
#define BRIDGE_A         (0)
/// Bridge B
#define BRIDGE_B         (1)

/// PWM id for PWM_A
#define PWM_A           (0)
/// PWM id for PWM_B
#define PWM_B           (1)
/// PWM id for PWM_REF_A
#define PWM_REF_A       (2)
/// PWM id for PWM_REF_A
#define PWM_REF_B       (3)

/* Exported Types  -------------------------------------------------------*/

/* Typedefs ------------------------------------------------------------------*/
template <typename T>
struct Callback;

template <typename Ret, typename... Params>
struct Callback<Ret(Params...)> {
  template <typename... Args>
  static Ret callback(Args... args)
  {
    return func(args...);
  }
  static std::function<Ret(Params...)> func;
};

template <typename Ret, typename... Params>
std::function<Ret(Params...)> Callback<Ret(Params...)>::func;

typedef void (*FaultInterruptHandler_Callback)(void);

/// Device Parameters Structure Type
typedef struct {
  /// Pwm frequency of the bridge input
  uint32_t bridgePwmFreq[STSPIN840_NB_BRIDGES];
  /// Pwm frequency of the ref pin
  uint32_t refPwmFreq[STSPIN840_NB_BRIDGES];
  /// Pwm Duty Cycle of the ref pin
  uint8_t refPwmDc[STSPIN840_NB_BRIDGES];
  /// Speed% (from 0 to 100) of the corresponding motor
  uint16_t speed[STSPIN840_NB_MAX_MOTORS];
  /// FORWARD or BACKWARD direction of the motors
  motorDir_t direction[STSPIN840_NB_MAX_MOTORS];
  /// Current State of the motors
  motorState_t motionState[STSPIN840_NB_MAX_MOTORS];
  /// Current State of the bridges
  bool bridgeEnabled[STSPIN840_NB_BRIDGES];
} deviceParams_t;

/// Motor driver initialization structure definition
typedef struct {
  uint32_t bridgePwmFreq[STSPIN840_NB_BRIDGES];
  uint32_t refPwmFreq[STSPIN840_NB_BRIDGES];
  uint8_t refPwmDc[STSPIN840_NB_BRIDGES];
} Stspin840_Init_t;

class Stspin840 : public BDCMotor {
  public:

    /*** Constructor and Destructor Methods ***/

    /**
     * @brief Constructor.
     * @param flag_and_enableA_pin  pin name of the ENA pin of the component.
     * @param flag_and_enableB_pin  pin name of the ENB pin of the component.
     * @param standby_reset_pin     pin name of the STBY\RST pin of the component.
     * @param dirA_pin              pin name for the direction pin for bridge A.
     * @param dirB_pin              pin name for the direction pin for bridge B.
     * @param pwmA_pin              pin name for the PWM input for bridge A.
     * @param pwmB_pin              pin name for the PWM input for bridge B.
     * @param pwmRefA_pin           pin name for the REFA pin of the component.
     * @param pwmRefB_pin           pin name for the REFB pin of the component.
     */
    Stspin840(uint8_t flag_and_enableA_pin, uint8_t flag_and_enableB_pin, uint8_t standby_reset_pin, uint8_t dirA_pin, uint8_t dirB_pin, uint8_t pwmA_pin, uint8_t pwmB_pin, uint8_t pwmRefA_pin, uint8_t pwmRefB_pin) :
      BDCMotor(),
      flag_and_enableA(flag_and_enableA_pin),
      flag_and_enableB(flag_and_enableB_pin),
      standby_reset(standby_reset_pin),
      dirA(dirA_pin),
      dirB(dirB_pin),
      pwmA(pwmA_pin),
      pwmB(pwmB_pin),
      pwmRefA(pwmRefA_pin),
      pwmRefB(pwmRefB_pin)
    {
      TIM_TypeDef *pwmA_instance;
      TIM_TypeDef *pwmB_instance;
      TIM_TypeDef *pwmRefA_instance;
      TIM_TypeDef *pwmRefB_instance;

      pinMode(flag_and_enableA, OUTPUT);
      pinMode(flag_and_enableB, OUTPUT);
      pinMode(standby_reset, OUTPUT);
      pinMode(dirA, OUTPUT);
      pinMode(dirB, OUTPUT);

      Callback<void()>::func = std::bind(&Stspin840::Stspin840_FaultInterruptHandler, this);
      callback_handler = static_cast<FaultInterruptHandler_Callback>(Callback<void()>::callback);

      pwmA_instance = (TIM_TypeDef *)pinmap_peripheral(digitalPinToPinName(pwmA), PinMap_PWM);
      pwmA_channel = STM_PIN_CHANNEL(pinmap_function(digitalPinToPinName(pwmA), PinMap_PWM));

      pwmB_instance = (TIM_TypeDef *)pinmap_peripheral(digitalPinToPinName(pwmB), PinMap_PWM);
      pwmB_channel = STM_PIN_CHANNEL(pinmap_function(digitalPinToPinName(pwmB), PinMap_PWM));

      pwmRefA_instance = (TIM_TypeDef *)pinmap_peripheral(digitalPinToPinName(pwmRefA), PinMap_PWM);
      pwmRefA_channel = STM_PIN_CHANNEL(pinmap_function(digitalPinToPinName(pwmRefA), PinMap_PWM));

      pwmRefB_instance = (TIM_TypeDef *)pinmap_peripheral(digitalPinToPinName(pwmRefB), PinMap_PWM);
      pwmRefB_channel = STM_PIN_CHANNEL(pinmap_function(digitalPinToPinName(pwmRefB), PinMap_PWM));

      pwmA_timer = new HardwareTimer(pwmA_instance);
      pwmB_timer = new HardwareTimer(pwmB_instance);
      pwmRefA_timer = new HardwareTimer(pwmRefA_instance);
      pwmRefB_timer = new HardwareTimer(pwmRefB_instance);
      first_time_pwmA = true;
      first_time_pwmB = true;
      first_time_pwmRefA = true;
      first_time_pwmRefB = true;

      errorHandlerCallback = 0;
      memset(&devicePrm, 0, sizeof(deviceParams_t));
      stspin840DriverInstance = numberOfDevices++;
    }

    /**
     * @brief Destructor.
     */
    virtual ~Stspin840(void)
    {
      free(pwmA_timer);
      free(pwmB_timer);
      free(pwmRefA_timer);
      free(pwmRefB_timer);
    }


    /*** Public Component Related Methods ***/

    /**
     * @brief Public functions inherited from the Component Class
     */

    /**
     * @brief  Initialize the component.
     * @param  init Pointer to device specific initialization structure.
     * @retval "0" in case of success, an error code otherwise.
     */
    virtual int init(void *init = NULL)
    {
      Stspin840_Init((void *) init);
      return 0;
    }

    /**
     * @brief  Getting the ID of the component.
     * @param  id Pointer to an allocated variable to store the ID into.
     * @retval "0" in case of success, an error code otherwise.
     */
    virtual int read_id(uint8_t *id = NULL)
    {
      *id = Stspin840_ReadId();
      return 0;
    }

    /**
     * @brief Public functions inherited from the BCDMotor Class
     */

    /**
     * @brief  Disabling the specified bridge.
     * @param  bridgeId from 0 for bridge A to 1 for bridge B.
     * @retval None.
     */
    virtual void disable_bridge(unsigned int bridgeId)
    {
      Stspin840_DisableBridge(bridgeId);
    }

    /**
     * @brief  Enabling the specified bridge.
     * @param  bridgeId from 0 for bridge A to 1 for bridge B
     * @retval None.
     */
    virtual void enable_bridge(unsigned int bridgeId)
    {
      Stspin840_EnableBridge(bridgeId);
    }

    /**
     * @brief  Getting the PWM frequency of the specified bridge;
     * @param  bridgeId from 0 for bridge A to 1 for bridge B.
     * @retval The frequency in Hz of the specified bridge input PWM.
     */
    virtual unsigned int get_bridge_input_pwm_freq(unsigned int bridgeId)
    {
      return (unsigned int) Stspin840_GetBridgeInputPwmFreq(bridgeId);
    }

    /**
     * @brief  Getting the bridge status.
     * @param  bridgeId from 0 for bridge A to 1 for bridge B.
     * @retval The status.
     */
    virtual unsigned int get_bridge_status(unsigned int bridgeId)
    {
      return (unsigned int) Stspin840_GetBridgeStatus(bridgeId);
    }

    /**
     * @brief  Getting the device State.
     * @param  motorId from 0 to (MAX_NUMBER_OF_BRUSH_DC_MOTORS - 1).
     * @retval The device state (STEADY or INACTIVE)
     */
    virtual unsigned int get_device_state(unsigned int motorId)
    {
      return (motorState_t) Stspin840_GetDeviceState(motorId);
    }

    /**
     * @brief  Getting the current speed in % of the specified motor.
     * @param  motorId from 0 to (MAX_NUMBER_OF_BRUSH_DC_MOTORS - 1).
     * @retval The current speed in %.
     */
    virtual unsigned int get_speed(unsigned int motorId)
    {
      return (unsigned int) Stspin840_GetCurrentSpeed(motorId);
    }

    /**
     * @brief  Stopping the motor and disabling the power bridge immediately.
     * @param  motorId from 0 to (MAX_NUMBER_OF_BRUSH_DC_MOTORS - 1).
     * @retval None.
     */
    virtual void hard_hiz(unsigned int motorId)
    {
      Stspin840_HardHiz(motorId);
    }

    /**
     * @brief  Stopping the motor immediately.
     * @param  motorId from 0 to (MAX_NUMBER_OF_BRUSH_DC_MOTORS - 1).
     * @retval None.
     */
    virtual void hard_stop(unsigned int motorId)
    {
      Stspin840_HardStop(motorId);
    }

    /**
     * @brief  Running the motor.
     * @param  motorId from 0 to (MAX_NUMBER_OF_BRUSH_DC_MOTORS - 1).
     * @param  direction The direction of rotation.
     * @retval None.
     */
    virtual void run(unsigned int motorId, direction_t direction)
    {
      Stspin840_Run(motorId, (motorDir_t) direction);
    }

    /**
     * @brief  Setting the PWM frequency of the specified bridge.
     * @param  bridgeId from 0 for bridge A to 1 for bridge B.
     * @param  frequency of the PWM in Hz
     * @retval None.
     */
    virtual void set_bridge_input_pwm_freq(unsigned int bridgeId, unsigned int frequency)
    {
      Stspin840_SetBridgeInputPwmFreq(bridgeId, frequency);
    }

    /**
     * @brief  Setting the dual bridge configuration mode.
     * @param  configuration. The bridge configuration.
     * @retval None.
     */
    virtual void set_dual_full_bridge_config(unsigned int configuration)
    {
      (void)configuration;
      /* Not implemented, do nothing */
      return;
    }

    /**
     * @brief  Setting the speed in %.
     * @param  motorId from 0 to (MAX_NUMBER_OF_BRUSH_DC_MOTORS - 1).
     * @param  speed The new speed in %.
     * @retval "true" in case of success, "false" otherwise.
     */
    virtual bool set_speed(unsigned int motorId, unsigned int speed)
    {
      return (bool) Stspin840_SetMaxSpeed(motorId, speed);
    }

    /**
     * @brief Public functions NOT inherited
     */

    /**
     * @brief  Attaching an error handler.
     * @param  fptr An error handler.
     * @retval None.
     */
    virtual void attach_error_handler(void (*fptr)(uint16_t error))
    {
      Stspin840_AttachErrorHandler((void (*)(uint16_t error)) fptr);
    }

    /**
     * @brief  Getting the motor current direction.
     * @param  motorId from 0 to (MAX_NUMBER_OF_BRUSH_DC_MOTORS - 1).
     * @retval direction The direction of rotation.
     */
    virtual direction_t get_direction(unsigned int motorId)
    {
      return (direction_t) Stspin840_GetDirection(motorId);
    }

    /**
     * @brief  Getting the version of the firmware.
     * @param  None.
     * @retval The version of the firmware.
     */
    virtual unsigned int get_fw_version(void)
    {
      return (unsigned int) Stspin840_GetFwVersion();
    }

    /**
     * @brief  Getting the duty cycle of the PWM used for REF.
     * @param  refId Id of the reference PWM signal.
     * @retval duty cycle in % (from 0 to 100)
     */
    virtual unsigned int get_ref_pwm_dc(unsigned int refId)
    {
      return (unsigned int) Stspin840_GetRefPwmDc(refId);
    }

    /**
     * @brief  Getting the frequency of the PWM used for REF.
     * @param  refId Id of the reference PWM signal.
     * @retval frequency in Hz.
     */
    virtual unsigned int get_ref_pwm_freq(unsigned int refId)
    {
      return (unsigned int) Stspin840_GetRefPwmFreq(refId);
    }

    /**
     * @brief  Releasing the reset (exiting standby mode).
     * @param  None.
     * @retval None.
     */
    virtual void release_reset(void)
    {
      Stspin840_ReleaseReset();
    }

    /**
     * @brief  Resetting (entering standby mode).
     * @param  None.
     * @retval None.
     */
    virtual void reset(void)
    {
      Stspin840_Reset();
    }

    /**
     * @brief  Setting the direction of rotation of the firmware.
     * @param  motorId from 0 to (MAX_NUMBER_OF_BRUSH_DC_MOTORS - 1).
     * @param direction The direction of rotation.
     * @retval None
     */
    virtual void set_direction(unsigned int motorId, direction_t direction)
    {
      Stspin840_SetDirection(motorId, (motorDir_t) direction);
    }

    /**
     * @brief  Setting the duty cycle of the PWM used for REF.
     * @param  refId Id of the reference PWM signal.
     * @param  newDc new duty cycle from 0 to 100
     * @retval None.
     */
    virtual void set_ref_pwm_dc(unsigned int refId, unsigned int newDc)
    {
      Stspin840_SetRefPwmDc(refId, newDc);
    }

    /**
     * @brief  Setting the frequency of the PWM used for REF.
     * @param  refId Id of the reference PWM signal.
     * @param  frequency in Hz.
     * @retval None.
     */
    virtual void set_ref_pwm_freq(unsigned int refId, unsigned int frequency)
    {
      Stspin840_SetRefPwmFreq(refId, frequency);
    }

    /**
     * @brief Public static functions
     */
    uint8_t get_nb_devices(void)
    {
      return Stspin840_GetNbDevices();
    }

    /*** Public Interrupt Related Methods ***/

    /**
     * @brief  Attaching an interrupt handler to the FLAG interrupt.
     * @param  fptr An interrupt handler.
     * @retval None.
     */
    void attach_flag_irq(void (*fptr)(void))
    {
      int_cb = fptr;
    }

  protected:

    /*** Protected Component Related Methods ***/

    void Stspin840_Init(void *pInit);
    uint16_t Stspin840_ReadId(void);
    void Stspin840_AttachErrorHandler(void (*callback)(uint16_t));
    void Stspin840_FaultInterruptHandler(void);
    void Stspin840_ErrorHandler(uint16_t error);
    void Stspin840_DisableBridge(uint8_t bridgeId);
    void Stspin840_EnableBridge(uint8_t bridgeId);
    uint32_t Stspin840_GetBridgeInputPwmFreq(uint8_t bridgeId);
    uint16_t Stspin840_GetBridgeStatus(uint8_t bridgeId);
    uint16_t Stspin840_GetCurrentSpeed(uint8_t motorId);
    motorState_t Stspin840_GetDeviceState(uint8_t motorId);
    motorDir_t Stspin840_GetDirection(uint8_t motorId);
    uint16_t Stspin840_GetMaxSpeed(uint8_t motorId);
    uint32_t Stspin840_GetFwVersion(void);
    uint8_t Stspin840_GetRefPwmDc(uint8_t refId);
    uint32_t Stspin840_GetRefPwmFreq(uint8_t refId);
    void Stspin840_HardHiz(uint8_t motorId);
    void Stspin840_HardStop(uint8_t motorId);
    void Stspin840_ReleaseReset(void);
    void Stspin840_Reset(void);
    void Stspin840_Run(uint8_t motorId, motorDir_t direction);
    void Stspin840_SetBridgeInputPwmFreq(uint8_t bridgeId, uint32_t newFreq);
    void Stspin840_SetDirection(uint8_t motorId, motorDir_t dir);
    bool Stspin840_SetMaxSpeed(uint8_t motorId, uint16_t newMaxSpeed);
    void Stspin840_SetRefPwmDc(uint8_t refId, uint8_t newDc);
    void Stspin840_SetRefPwmFreq(uint8_t refId, uint32_t newFreq);
    uint8_t Stspin840_GetNbDevices(void);

    /**
     * @brief Functions to initialize the registers
     */
    void Stspin840_SetDeviceParamsToGivenValues(uint8_t deviceId, Stspin840_Init_t *pInitPrm);
    void Stspin840_SetDeviceParamsToPredefinedValues(void);

    /**
     * @brief  Get the state of the standby/reset pin
     */
    uint8_t Stspin840_GetResetState(void);

    /*** Component's I/O Methods ***/

    /**
     * @brief  Making the CPU wait.
     * @param  None.
     * @retval None.
     */
    void Stspin840_Board_Delay(uint32_t ms_delay)
    {
      delay(ms_delay);
    }

    /**
     * @brief  Disable bridge.
     * @param  bridgeId the bridge id.
     * @retval None.
     */
    void Stspin840_Board_DisableBridge(uint8_t bridgeId)
    {
      if (bridgeId == 0) {
        detachInterrupt(flag_and_enableA);
        pinMode(flag_and_enableA, OUTPUT);
        digitalWrite(flag_and_enableA, 0);
      } else {
        detachInterrupt(flag_and_enableB);
        pinMode(flag_and_enableB, OUTPUT);
        digitalWrite(flag_and_enableB, 0);
      }
    }

    /**
     * @brief  Enable bridge.
     * @param  bridgeId the bridge id.
     * @param  addDelay if different from 0, a delay is added after bridge activation.
     * @retval None.
     */
    void Stspin840_Board_EnableBridge(uint8_t bridgeId, uint8_t addDelay)
    {
      if (bridgeId == 0) {
        pinMode(flag_and_enableA, OUTPUT);
        digitalWrite(flag_and_enableA, 1);
        if (addDelay) {
          Stspin840_Board_Delay(1);
        }
        pinMode(flag_and_enableA, INPUT_PULLUP);
        attachInterrupt(flag_and_enableA, callback_handler, FALLING);
      } else {
        pinMode(flag_and_enableB, OUTPUT);
        digitalWrite(flag_and_enableB, 1);
        if (addDelay) {
          Stspin840_Board_Delay(1);
        }
        pinMode(flag_and_enableB, INPUT_PULLUP);
        attachInterrupt(flag_and_enableB, callback_handler, FALLING);
      }
    }

    /**
     * @brief  Get the status of the reset Pin.
     * @param  None.
     * @retval the digital state of the pin.
     */
    uint8_t Stspin840_Board_GetResetPinState(void)
    {
      return ((uint16_t)digitalRead(standby_reset));
    }

    /**
     * @brief  Get the status of the flag and enable Pin.
     * @param  bridgeId the bridge id.
     * @retval the digital state of the pin.
     */
    uint8_t Stspin840_Board_GetFaultPinState(uint8_t bridgeId)
    {
      if (bridgeId == 0) {
        return ((uint16_t)digitalRead(flag_and_enableA));
      } else {
        return ((uint16_t)digitalRead(flag_and_enableB));
      }
    }

    /**
    * @brief  Deinitialising the PWM.
    * @param  pwmId 0 for bridge A PWM, 1 for bridge B PWM, 2 for REFA PWM, 3 for REFB PWM.
    * @retval None.
    */
    void Stspin840_Board_PwmDeInit(uint8_t pwmId)
    {
      (void)pwmId;
    }

    /**
    * @brief  Initialising the PWM.
    * @param  pwmId 0 for bridge A PWM, 1 for bridge B PWM, 2 for REFA PWM, 3 for REFB PWM.
    * @param  onlyChannel.
    * @retval None.
    */
    void Stspin840_Board_PwmInit(uint8_t pwmId, uint8_t onlyChannel)
    {
      (void)pwmId;
      (void)onlyChannel;
    }

    /**
     * @brief  Setting the frequency of PWM.
     *         The frequency of bridge A and B controls directly the speed of the device.
     * @param  pwmId 0 for bridge A PWM, 1 for bridge B PWM, 2 for REFA PWM, 3 for REFB PWM.
     * @param  newFreq frequency to apply in Hz.
     * @param  duty Duty cycle to use from 0 to 100 (reversed in REF pins).
     * @retval None.
     */
    void Stspin840_Board_PwmSetFreq(uint8_t pwmId, uint32_t newFreq, uint8_t duty)
    {
      switch (pwmId) {
        case 0:
        default:
          /* Setting the period and the duty-cycle of PWM A. */
          if (!first_time_pwmA) {
            pwmA_timer->pauseChannel(pwmA_channel);
          } else {
            first_time_pwmA = false;
          }
          pwmA_timer->setPWM(pwmA_channel, pwmA, newFreq, duty);
          break;
        case 1:
          /* Setting the period and the duty-cycle of PWM B. */
          if (!first_time_pwmB) {
            pwmB_timer->pauseChannel(pwmB_channel);
          } else {
            first_time_pwmB = false;
          }
          pwmB_timer->setPWM(pwmB_channel, pwmB, newFreq, duty);
          break;
        case 2:
          // Reverse duty cycle for ref pins (100% = 0%)
          duty = 100 - duty;
          /* Setting the period and the duty-cycle of PWM RefA. */
          if (!first_time_pwmRefA) {
            pwmRefA_timer->pauseChannel(pwmRefA_channel);
          } else {
            first_time_pwmRefA = false;
          }
          pwmRefA_timer->setPWM(pwmRefA_channel, pwmRefA, newFreq, duty);
          break;
        case 3:
          // Reverse duty cycle for ref pins (100% = 0%)
          duty = 100 - duty;
          /* Setting the period and the duty-cycle of PWM RefB. */
          if (!first_time_pwmRefB) {
            pwmRefB_timer->pauseChannel(pwmRefB_channel);
          } else {
            first_time_pwmRefB = false;
          }
          pwmRefB_timer->setPWM(pwmRefB_channel, pwmRefB, newFreq, duty);
          break;
      }
    }

    /**
     * @brief  Stopping the PWM.
     * @param  pwmId 0 for bridge A PWM, 1 for bridge B PWM, 2 for REFA PWM, 3 for REFB PWM.
     * @retval None.
     */
    void Stspin840_Board_PwmStop(uint8_t pwmId)
    {
      switch (pwmId) {
        case 0:
        default:
          if (!first_time_pwmA) {
            pwmA_timer->pauseChannel(pwmA_channel);
          } else {
            first_time_pwmA = false;
          }
          pwmA_timer->setPWM(pwmA_channel, pwmA, devicePrm.bridgePwmFreq[0], 0);
          break;
        case 1:
          if (!first_time_pwmB) {
            pwmB_timer->pauseChannel(pwmB_channel);
          } else {
            first_time_pwmB = false;
          }
          pwmB_timer->setPWM(pwmB_channel, pwmB, devicePrm.bridgePwmFreq[1], 0);
          break;
        case 2:
          if (!first_time_pwmRefA) {
            pwmRefA_timer->pauseChannel(pwmRefA_channel);
          } else {
            first_time_pwmRefA = false;
          }
          pwmRefA_timer->setPWM(pwmRefA_channel, pwmRefA, devicePrm.refPwmFreq[0], 0);
          break;
        case 3:
          if (!first_time_pwmRefB) {
            pwmRefB_timer->pauseChannel(pwmRefB_channel);
          } else {
            first_time_pwmRefB = false;
          }
          pwmRefB_timer->setPWM(pwmRefB_channel, pwmRefB, devicePrm.refPwmFreq[1], 0);
          break;
      }
    }

    /**
     * @brief  Putting the device in standby mode.
     * @param  None.
     * @retval None.
     */
    void Stspin840_Board_ReleaseReset()
    {
      digitalWrite(standby_reset, 1);
      Stspin840_Board_Delay(1);
    }

    /**
     * @brief  Putting the device in reset mode.
     * @param  None.
     * @retval None.
     */
    void Stspin840_Board_Reset()
    {
      digitalWrite(standby_reset, 0);
    }

    /**
     * @brief  Setting the direction of rotation.
     * @param  bridgeId 0 for bridge A, 1 for bridge B.
     * @param  gpioState direction of rotation: "1" for forward, "0" for backward.
     * @retval None.
     */
    void Stspin840_Board_SetDirectionGpio(uint8_t bridgeId, uint8_t gpioState)
    {
      if (bridgeId == 0) {
        digitalWrite(dirA, gpioState);
      } else {
        digitalWrite(dirB, gpioState);
      }
    }

    /*** Component's Instance Variables ***/

    /* Flag Interrupt. */
    uint8_t flag_and_enableA;
    uint8_t flag_and_enableB;
    void (*int_cb)(void);
    FaultInterruptHandler_Callback callback_handler;

    /* Standby/reset pin. */
    uint8_t standby_reset;

    /* Direction pin of bridge A. */
    uint8_t dirA;

    /* Direction pin of bridge B. */
    uint8_t dirB;

    /* Pulse Width Modulation pin for bridge A input. */
    HardwareTimer *pwmA_timer;
    uint32_t pwmA_channel;
    uint8_t pwmA;
    bool first_time_pwmA;

    /* Pulse Width Modulation pin for bridge A input. */
    HardwareTimer *pwmB_timer;
    uint32_t pwmB_channel;
    uint8_t pwmB;
    bool first_time_pwmB;

    /* Pulse Width Modulation pin for RefA signal. */
    HardwareTimer *pwmRefA_timer;
    uint32_t pwmRefA_channel;
    uint8_t pwmRefA;
    bool first_time_pwmRefA;

    /* Pulse Width Modulation pin for RefB signal. */
    HardwareTimer *pwmRefB_timer;
    uint32_t pwmRefB_channel;
    uint8_t pwmRefB;
    bool first_time_pwmRefB;

    /* Data. */
    void (*errorHandlerCallback)(uint16_t error);
    deviceParams_t devicePrm;
    uint8_t stspin840DriverInstance;

    /* Static data. */
    static uint8_t numberOfDevices;
};

#endif /* #ifndef __STSPIN840_H */


/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
