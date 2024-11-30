#pragma once

// The BTStack GATT compiler doesn't offer a good way to have stable char IDs.
// Handle this by defining stable IDs in one place.

// Environmental
#define ENV_VOC_INDEX_INTAKE 216aa791_97d0_46ac_8752_60bbc00611e1_01
#define ENV_VOC_INDEX_EXHAUST 216aa791_97d0_46ac_8752_60bbc00611e1_02
#define ENV_VOC_RAW_INTAKE c3acb286_8071_427b_bbed_d64987373f23_01
#define ENV_VOC_RAW_EXHAUST c3acb286_8071_427b_bbed_d64987373f23_02
#define ENV_AGGREGATE 75134bec_dd06_49b1_bac2_c15e05fd7199_01

// Fan
#define FAN_POWER 2B04_01
#define FAN_POWER_OVERRIDE 2B04_02
#define FAN_POWER_PASSIVE 2B04_03
#define FAN_POWER_AUTOMATIC 2B04_04
#define FAN_POWER_COEFFICIENT 2B04_05
#define FAN_POWER_THERMAL_LIMIT 45d2e7d7_40c4_46a6_a160_43eb02d01e27_01
#define FAN_TACHOMETER 03f61fe0_9fe7_4516_98e6_056de551687f_01
#define FAN_POWER_TACHO_AGGREGATE 79cd747f_91af_49a6_95b2_5b597c683129_01
#define FAN_AGGREGATE 75134bec_dd06_49b1_bac2_c15e05fd7199_02

// Fan Policy
#define FAN_POLICY_COOLDOWN 2B16_01
#define FAN_POLICY_VOC_PASSIVE_MAX 216aa791_97d0_46ac_8752_60bbc00611e1_03
#define FAN_POLICY_VOC_IMPROVE_MIN 216aa791_97d0_46ac_8752_60bbc00611e1_04

// NeoPixel
#define WS2812_UPDATE_SPAN 5d91b6ce_7db1_4e06_b8cb_d75e7dd49aae_01
#define WS2812_TOTAL_COMPONENTS 2AEA_01

// Display
#define DISPLAY_BRIGHTNESS 2B04_06
#define DISPLAY_UI 86a25d55_1893_4d01_8ea8_8970f622c243_01

// Photocatalytic
#define PHOTOCATALYTIC_POWER 2B04_07
#define PHOTOCATALYTIC_POWER_OVERRIDE 2B04_08

// Servo
#define SERVO_VENT_RANGE 9c327c7f_188f_4345_950f_bd586f13f324_01
#define SERVO_VENT_POWER 0543a134_244f_405b_9d43_0351a5336ef7_01

// Config
#define CONFIG_REBOOT f48a18bb_e03c_4583_8006_5b54422e2045_01
#define CONFIG_FLAGS d4b66bf4_3d8f_4746_b6a2_8a59d2eac3ce_01
#define CONFIG_CHECKPOINT_SENSOR_CALIBRATION a84b00c0_7102_4cc6_a4ea_a65050502d3f_01
#define CONFIG_RESET_SENSOR_CALIBRATION 75bf055c_02be_466f_8c7d_6ebc72078048_01
#define CONFIG_RESET_SETTINGS f2810b13_8cd7_4d6f_bb1b_e276db7fadbf_01
#define CONFIG_VOC_GATING_THRESHOLD 216aa791_97d0_46ac_8752_60bbc00611e1_05
#define CONFIG_VOC_GATING_THRESHOLD_OVERRIDE 216aa791_97d0_46ac_8752_60bbc00611e1_06
#define CONFIG_VOC_CALIBRATE_ENABLED ee786ac0_7700_47dd_b7de_9958f96303f2_01
#define CONFIG_PINS_DEFAULT 5b1dc210_6a51_4cf9_bda7_085604199856_01
#define CONFIG_PINS 2e9410cb_30fd_4b2c_8c95_934226a9ba29_01
#define CONFIG_PINS_ERROR 0f6d7c4b_c30c_45b2_b32a_0e5b130429f0_01
