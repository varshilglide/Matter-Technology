/**
  *****************************************************************************
  * @file       common.h
  *             This file provides common functionalities for all files.
  *
  * @author     Glide Technology Pvt. Ltd. <www.glidemtech.com>
  * @version    1.0
  * @date       06, Jul, 2017
  *****************************************************************************
  */

/* Define to prevent recursive inclusion ------------------------------------*/
#ifndef COMMON_H
#define COMMON_H



/** @defgroup   common_module Common
  * @{
  */

/** @addtogroup common_module
  * @{
  */

/* Exported constant macros -------------------------------------------------*/

/** @defgroup   common_exported_constant_macros Common: Exported constant macros
  * @{
  */

//#define TRUE    1	/**< success response */
//#define FALSE   0	/**< failure response */

//#define STATUS_ERROR    "ERROR"	/**< failure response */
//#define STATUS_OK   "OK"	/**< success response */

/**
  * @}
  */

/* Exported function macros -------------------------------------------------*/
/* Exported types -----------------------------------------------------------*/

/** @defgroup   common_exported_types Common: Exported types
  * @{
  */

/**
  *  The enumeration of return type.
  */
//typedef enum {
//	FALSE = 0,	/**< success return response */
//	TRUE = 1	/**< failure return response */
//} result_t;	/**< return type */

typedef enum {
    RET_OK = 0,	/**< success return response */
    RET_ERR = 1	/**< failure return response */
} ret_t;	/**< return type */

/**
  *  The enumeration of boot mode.
  */
typedef enum {
    BOOT_MODE_NORMAL      = 0,      /**< Normal mode - Goto to firmware straight */
    BOOT_MODE_FW_TRANSFER = 1,      /**< Transfer firmware mode - Transfer firmware copy to main firmware location */
    BOOT_MODE_FW_GET      = 2,      /**< Get Firmware mode - Get firmware from phone to internal flash memory */
} boot_mode_t;	/**< boot mode */


/**
  * @}
  */

/* Exported variables -------------------------------------------------------*/
/* Exported functions -------------------------------------------------------*/

/**
  * @}
  */

/**
  * @}
  */

#endif /* COMMON_H */

/* End of file --------------------------------------------------------------*/
