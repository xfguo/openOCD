/***************************************************************************
 *   Copyright (C) 2008 by Xiongfei Guo                                    *
 *   xfguo@credosemi.com                                                   *
 *                                                                         *
 *   This program is free software; you can redistribute it and/or modify  *
 *   it under the terms of the GNU General Public License as published by  *
 *   the Free Software Foundation; either version 2 of the License, or     *
 *   (at your option) any later version.                                   *
 *                                                                         *
 *   This program is distributed in the hope that it will be useful,       *
 *   but WITHOUT ANY WARRANTY; without even the implied warranty of        *
 *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the         *
 *   GNU General Public License for more details.                          *
 *                                                                         *
 *   You should have received a copy of the GNU General Public License     *
 *   along with this program; if not, write to the                         *
 *   Free Software Foundation, Inc.,                                       *
 *   51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA.           *
 ***************************************************************************/

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#include <jtag/interface.h>
#include "bitbang.h"
#include "hello.h"


#define _CRT_SECURE_NO_WARNINGS
#if defined(WIN32)
	#include <windows.h>
#endif

#include "dpcdecl.h" 
#include "djtg.h"
#include "dmgr.h"

HIF hif;

/* my private tap controller state, which tracks state for calling code */
static tap_state_t adapt2_state = TAP_RESET;

static int adapt2_clock;		/* edge detector */

static int clock_count;		/* count clocks in any stable state, only stable states */

static uint32_t adapt2_data;

static int adapt2_read(void)
{
    int tms, tdi, tck, tdo;
    tdi = tms = tck = 0;
	if(!DjtgGetTmsTdiTdoTck(hif, &tms, &tdi, &tdo, &tck)) {
		LOG_ERROR("DjtgGetTmsTdiTdoTck failed\n");
		pre_exit();
	}
    return tdo;
}

static void adapt2_write(int tck, int tms, int tdi)
{
	if(!DjtgSetTmsTdiTck(hif, tms, tdi, tck)) {
		LOG_ERROR("DjtgSetTmsTdiTck failed\n");
		pre_exit();
	}
}

static void adapt2_reset(int trst, int srst)
{
}

static void adapt2_led(int on)
{
}

static struct bitbang_interface adapt2_bitbang = {
		.read = &adapt2_read,
		.write = &adapt2_write,
		.reset = &adapt2_reset,
		.blink = &adapt2_led,
	};

static int adapt2_khz(int khz, int *jtag_speed)
{
	if (khz == 0)
		*jtag_speed = 0;
	else
		*jtag_speed = 64000/khz;
	return ERROR_OK;
}

static int adapt2_speed_div(int speed, int *khz)
{
	if (speed == 0)
		*khz = 0;
	else
		*khz = 64000/speed;

	return ERROR_OK;
}

static int adapt2_speed(int speed)
{
	return ERROR_OK;
}

static int adapt2_init(void)
{
	bitbang_interface = &adapt2_bitbang;

	return ERROR_OK;
}

void pre_exit() {
	if( hif != hifInvalid ) {

		// DJGT API Call: DjtgDisable
		DjtgDisable(hif);

		// DMGR API Call: DmgrClose
		DmgrClose(hif);
	}
}

static int adapt2_quit(void)
{
    pre_exit();
	return ERROR_OK;
}

COMMAND_HANDLER(adapt2_select_device_cmd)
{
	int i;
	int cCodes = 0;
	BYTE rgbSetup[] = {0xaa, 0x22, 0x00};
	BYTE rgbTdo[4];

	INT32 idcode;
	INT32 rgIdcodes[16];
    char *sel_device;

	if (CMD_ARGC == 0) {
		LOG_WARNING("You need to set a port number");
        pre_exit();
        return ERROR_FAIL;
    }
	else
		sel_device = strdup(CMD_ARGV[0]);
	
    LOG_INFO("Select device %s", sel_device);

	// DMGR API Call: DmgrOpen
	if(!DmgrOpen(&hif, sel_device)) {
		LOG_ERROR("Error: Could not open device. Check device name");
        pre_exit();
        return ERROR_FAIL;
	}
	
    // DJTG API CALL: DjtgEnable
	if(!DjtgEnable(hif)) {
		LOG_ERROR("Error: DjtgEnable failed");
        pre_exit();
        return ERROR_FAIL;
	}
	/* Put JTAG scan chain in SHIFT-DR state. RgbSetup contains TMS/TDI bit-pairs. */
	// DJTG API Call: DgtgPutTmsTdiBits
	if(!DjtgPutTmsTdiBits(hif, rgbSetup, NULL, 9, NULL)) {
		LOG_INFO("DjtgPutTmsTdiBits failed");
        pre_exit();
        return ERROR_FAIL;
	}

	/* Get IDCODES from device until we receive a value of 0x00000000 */
	do {
		
		// DJTG API Call: DjtgGetTdoBits
		if(!DjtgGetTdoBits(hif, 0, 0, rgbTdo, 32, NULL)) {
			LOG_ERROR("Error: DjtgGetTdoBits failed");
            pre_exit();
            return ERROR_FAIL;
		}

		// Convert array of bytes into 32-bit value
		idcode = (rgbTdo[3] << 24) | (rgbTdo[2] << 16) | (rgbTdo[1] << 8) | (rgbTdo[0]);

		// Place the IDCODEs into an array for LIFO storage
		rgIdcodes[cCodes] = idcode;

		cCodes++;

	} while( idcode != 0 );

	/* Show the IDCODEs in the order that they are connected on the device */
	LOG_INFO("Ordered JTAG scan chain:");
	for(i=cCodes-2; i >= 0; i--) {
		LOG_INFO("0x%08x", rgIdcodes[i]);
	}


    return ERROR_OK;
}

COMMAND_HANDLER(adapt2_enmu_cmd)
{
	DVC		dvc;
	int		idvc;
	int		cdvc;
	char	szTmp[1024];
	int     fSkip;
	
    if (!DmgrEnumDevices(&cdvc)) {
		LOG_ERROR("Error enumerating devices");
        pre_exit();
        return ERROR_FAIL;
	}
	for (idvc = 0; idvc < cdvc; idvc++) {
		fSkip = 0;

		// DMGR API Call: DmgrGetDvc
		if (!DmgrGetDvc(idvc, &dvc)) {
			LOG_INFO("Error getting device info");
            pre_exit();
            return ERROR_FAIL;
		}

		LOG_INFO("Device: %s", dvc.szName);
		
		/* Read and print product name. */
	
		// DMGR API Call: DmgrGetInfo
		if (!DmgrGetInfo(&dvc, dinfoProdName, szTmp)) {
			sprintf(szTmp, "Not accessible");
		}

		LOG_INFO("  Product Name:           %s",szTmp);

		/* Read and print user name. */	
		// DMGR API Call: DmgrGetInfo

		if (!DmgrGetInfo(&dvc, dinfoUsrName, szTmp)) {
			sprintf(szTmp, "Not accessible");
		}
	
		LOG_INFO("  User Name:              %s",szTmp);


		/* Read and print serial number */
		// DMGR API Call: DmgrGetInfo
		if (!DmgrGetInfo(&dvc, dinfoSN, szTmp)) {
			sprintf(szTmp, "Not accessible");
		}

		LOG_INFO("  Serial Number:          %s", szTmp);

		LOG_INFO("");
	}  // End for
	
    /* Clean up and get out */
	// DMGR API Call: DmgrFreeDvcEnum
	DmgrFreeDvcEnum();

    return ERROR_OK;
}
const struct command_registration digilent_adapt2_command_handlers[] = {
	{
		.name = "adapt2_enmu",
		.handler = adapt2_enmu_cmd,
		.mode = COMMAND_ANY,
		.help = "prints a warm welcome",
		.usage = "[name]",
	},
	{
		.name = "adapt2_select_device",
		.handler = &adapt2_select_device_cmd,
		.mode = COMMAND_CONFIG,
		.help = "set the port of the VPI server",
		.usage = "description_string",
	},
	COMMAND_REGISTRATION_DONE
};


/* The adapt2 driver is used to easily check the code path
 * where the target is unresponsive.
 */
struct jtag_interface adapt2_interface = {
		.name = "adapt2",

		.supported = DEBUG_CAP_TMS_SEQ,
		.commands = digilent_adapt2_command_handlers,
		.transports = jtag_only,

		.execute_queue = &bitbang_execute_queue,

		.speed = &adapt2_speed,
		.khz = &adapt2_khz,
		.speed_div = &adapt2_speed_div,

		.init = &adapt2_init,
		.quit = &adapt2_quit,
	};
