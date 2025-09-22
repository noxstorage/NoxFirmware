/**
 ******************************************************************************
 * @file   sigfox_data.h
 * @author MCD Application Team
 * @brief  provides encrypted data
 ******************************************************************************
 * @attention
 *
 * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
 * All rights reserved.</center></h2>
 *
 * This software component is licensed by ST under Ultimate Liberty license
 * SLA0044, the "License"; You may not use this file except in compliance with
 * the License. You may obtain a copy of the License at:
 *                             www.st.com/SLA0044
 *
 ******************************************************************************
 */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __SIGFOX_DATA_H__
#define __SIGFOX_DATA_H__

/* Includes ------------------------------------------------------------------*/
/* Exported types ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/

// V1

#define SIGFOX_KEY_1A 6A, A4, 84, 64, E8, D0, 9E, 3E, 73, F9, 3E, BC, 67, D3, 0C, 66
#define SIGFOX_ID_1A 02, 22, 2B, A2 // V 1 3 A
#define SIGFOX_PAC_1A AD, 3A, E5, A3, 49, A8, 78, 47

#define SIGFOX_KEY_1B 1F, 34, 1A, E4, F1, 80, 8F, 8A, E1, 27, E3, FC, 8F, 95, B6, A4
#define SIGFOX_ID_1B 02, 22, 2B, B7 // V 1 3 B
#define SIGFOX_PAC_1B B1, 62, 57, B7, C1, A4, C1, 0D

#define SIGFOX_KEY_1C 1A, 01, 18, 3C, 4D, 45, CD, B0, C7, B7, 49, CF, 35, 22, 8C, 85
#define SIGFOX_ID_1C 02, 22, 2C, 65 // V 1 3 C
#define SIGFOX_PAC_1C CD, 25, 41, 29, 87, 51, A7, 91

#define SIGFOX_KEY_1D B2, C0, 34, 3D, D2, 10, 4B, F1, 37, 74, A5, 2A, 32, FC, D9, E6
#define SIGFOX_ID_1D 02, 22, 2C, 66 // V 1 3 D
#define SIGFOX_PAC_1D 25, C9, 2D, 26, F6, EF, 43, 84

#define SIGFOX_KEY_1E F8, 70, 0E, 97, 23, 31, 8A, 64, CE, 3F, F3, C1, 60, FC, 02, C5
#define SIGFOX_ID_1E 02, 22, 2C, 60 // V 1 3 E
#define SIGFOX_PAC_1E 95, B9, 6E, 7F, 7E, C9, 02, 43

#define SIGFOX_KEY_1F 81, DE, 91, 27, 1D, 7F, E4, E6, 50, 13, 17, 37, 42, 47, 37, 8D
#define SIGFOX_ID_1F 02, 22, 2C, 50 // V 1 3 F
#define SIGFOX_PAC_1F 2B, 29, D8, D4, 69, D3, 48, 4D

#define SIGFOX_KEY_1G 82, 8C, 2B, 84, AE, 82, 82, 84, EA, BA, E2, CA, 50, 7C, 40, 34
#define SIGFOX_ID_1G 02, 22, 2C, 5F // V 1 3 G
#define SIGFOX_PAC_1G FA, CB, 5B, D5, 9B, 57, A4, 00

// V3

#define SIGFOX_KEY_3A 8F, A8, 5A, AD, BB, 3D, BA, 1D, BD, 11, C7, B9, 7C, 8B, 82, A5
#define SIGFOX_ID_3A 02, 22, 2D, 02
#define SIGFOX_PAC_3A C1, C4, A8, 4C, 4A, 74, 9A, 2E

#define SIGFOX_KEY_3B 42, E9, 96, 5A, DB, 90, 13, 1F, 42, 90, 9E, 6B, 74, 49, 67, 42
#define SIGFOX_ID_3B 02, 22, 2D, 03
#define SIGFOX_PAC_3B 79, 12, D8, 94, FE, B6, 95, 5B

#define SIGFOX_KEY_3C C4, 29, 93, C2, E4, 6C, 24, 10, 77, 8C, 5A, B0, 6A, E8, 53, AC
#define SIGFOX_ID_3C 02, 22, 2D, 08
#define SIGFOX_PAC_3C 86, 56, 28, EF, 06, AC, 87, EE

#define SIGFOX_KEY_3D 4B, 16, 48, 8D, 8F, 3C, D8, DF, 07, 3D, 2A, 48, 07, 92, 7C, 7B
#define SIGFOX_ID_3D 02, 22, 2D, 09
#define SIGFOX_PAC_3D 3A, 2C, D0, 7E, 26, D0, 02, 8F

#define SIGFOX_KEY_3E 80, 5D, 09, 72, 09, 40, 28, 53, 5E, AC, 42, 0B, AF, BD, F6, 18
#define SIGFOX_ID_3E 02, 22, 2D, 0A
#define SIGFOX_PAC_3E B0, A9, 28, A4, 33, A1, BA, 74

#define SIGFOX_KEY_3F 8C, 46, 46, 58, 03, 72, 1C, E1, 70, EC, 88, B8, A7, 6E, AC, 97
#define SIGFOX_ID_3F 02, 22, 2D, 07
#define SIGFOX_PAC_3F D7, 44, 0B, 8D, 64, 98, A2, CB

#define SIGFOX_KEY_3G D3, 54, B9, 48, 28, 3D, 3C, 75, D3, 7D, 00, 8A, 66, 35, 05, F9
#define SIGFOX_ID_3G 02, 22, 2D, 0B
#define SIGFOX_PAC_3G 77, 13, 76, EF, 5D, 2A, 66, 95

#define SIGFOX_KEY_3H 8B, 91, C3, 03, A0, B1, 5D, A0, C6, 9E, CD, 59, 0C, 9F, 05, 76
#define SIGFOX_ID_3H 02, 22, 2D, 0C
#define SIGFOX_PAC_3H 07, C0, AF, 78, B6, EF, A9, C3

#define SIGFOX_KEY_3I BE, 6D, F5, DC, 7E, 76, 8D, CC, 8F, 02, 09, B8, 63, 92, 42, C7
#define SIGFOX_ID_3I 02, 22, 2D, 0D
#define SIGFOX_PAC_3I 0F, F5, CA, F7, 49, F9, FC, AC

#define SIGFOX_KEY SIGFOX_KEY_3D
#define SIGFOX_ID SIGFOX_ID_3D
#define SIGFOX_PAC SIGFOX_PAC_3D

/* Exported macros -----------------------------------------------------------*/
/* External variables --------------------------------------------------------*/

#endif /*__SIGFOX_DATA_H__*/

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
