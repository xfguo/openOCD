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
 *                                                                         *
 *   Author          Time         Modefy content                           *
 *   Shanjin Yang    2014-10-9    modefy Simulation of IO port to          * 
 *                                DJTG Batch Operation                     * 
 *                                                                         *
 ****************************************************************************/

#ifdef HAVE_CONFIG_H 
	#include "config.h"
#endif

#include "bb.h"
#include <jtag/interface.h>
#include <jtag/commands.h>

#include <jtag/interface.h>
#include "bb.h"
#include "hello.h"

#define _CRT_SECURE_NO_WARNINGS
#if defined(WIN32)
	#include <windows.h>
#endif

#include "dpcdecl.h" 
#include "djtg.h"
#include "dmgr.h"

HIF hif;
int bb_execute_queue(void);

#define BUFFER_SIZE 65536

BYTE * adept_buf;

static void pre_exit() {
	if( hif != hifInvalid ) {

		// DJGT API Call: DjtgDisable
		DjtgDisable(hif);

		// DMGR API Call: DmgrClose
		DmgrClose(hif);
	}
}

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

static struct bitbang_interface adapt2_bb = {
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
	bitbang_interface = &adapt2_bb;
	
	adept_buf = malloc(BUFFER_SIZE);

	return ERROR_OK;
}

static int adapt2_quit(void)
{
	pre_exit();

	free(adept_buf);

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
	if(!DjtgPutTmsTdiBits(hif, rgbSetup, NULL, 9, fFalse)) {
		LOG_INFO("DjtgPutTmsTdiBits failed");
		pre_exit();
		return ERROR_FAIL;
	}

	/* Get IDCODES from device until we receive a value of 0x00000000 */
	do {
		
		// DJTG API Call: DjtgGetTdoBits
		if(!DjtgGetTdoBits(hif, 0, 0, rgbTdo, 32, fFalse)) {
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

COMMAND_HANDLER(adapt2_enum_cmd)
{
	DVC		dvc;
	int		idvc;
	int		cdvc;
	char	szTmp[1024];
	int	 fSkip;
	
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

		LOG_INFO("  Product Name:		   %s",szTmp);

		/* Read and print user name. */	
		// DMGR API Call: DmgrGetInfo

		if (!DmgrGetInfo(&dvc, dinfoUsrName, szTmp)) {
			sprintf(szTmp, "Not accessible");
		}
	
		LOG_INFO("  User Name:			  %s",szTmp);


		/* Read and print serial number */
		// DMGR API Call: DmgrGetInfo
		if (!DmgrGetInfo(&dvc, dinfoSN, szTmp)) {
			sprintf(szTmp, "Not accessible");
		}

		LOG_INFO("  Serial Number:		  %s", szTmp);

	}  // End for
	
	/* Clean up and get out */
	// DMGR API Call: DmgrFreeDvcEnum
	DmgrFreeDvcEnum();

	return ERROR_OK;
}
const struct command_registration digilent_adapt2_command_handlers[] = {
	{
		.name = "adapt2_enum",
		.handler = adapt2_enum_cmd,
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

		.execute_queue = &bb_execute_queue,

		.speed = &adapt2_speed,
		.khz = &adapt2_khz,
		.speed_div = &adapt2_speed_div,

		.init = &adapt2_init,
		.quit = &adapt2_quit,
	};


/**
 * Function bb_stableclocks
 * issues a number of clock cycles while staying in a stable state.
 * Because the TMS value required to stay in the RESET state is a 1, whereas
 * the TMS value required to stay in any of the other stable states is a 0,
 * this function checks the current stable state to decide on the value of TMS
 * to use.
 */
static void bb_stableclocks(int num_cycles);

struct bitbang_interface *bitbang_interface;

/* DANGER!!!! clock absolutely *MUST* be 0 in idle or reset won't work!
 *
 * Set this to 1 and str912 reset halt will fail.
 *
 * If someone can submit a patch with an explanation it will be greatly
 * appreciated, but as far as I can tell (ØH) DCLK is generated upon
 * clk = 0 in TAP_IDLE. Good luck deducing that from the ARM documentation!
 * The ARM documentation uses the term "DCLK is asserted while in the TAP_IDLE
 * state". With hardware there is no such thing as *while* in a state. There
 * are only edges. So clk => 0 is in fact a very subtle state transition that
 * happens *while* in the TAP_IDLE state. "#&¤"#¤&"#&"#&
 *
 * For "reset halt" the last thing that happens before srst is asserted
 * is that the breakpoint is set up. If DCLK is not wiggled one last
 * time before the reset, then the breakpoint is not set up and
 * "reset halt" will fail to halt.
 *
 */
#define CLOCK_IDLE() 0

/* The bb driver leaves the TCK 0 when in idle */
static void bb_end_state(tap_state_t state)
{
	if (tap_is_state_stable(state))
		tap_set_end_state(state);
	else {
		LOG_ERROR("BUG: %i is not a valid end state", state);
		exit(-1);
	}
}

static void bb_state_move(int skip)
{
	int i = 0, tms = 0;
	uint8_t tms_scan = tap_get_tms_path(tap_get_state(), tap_get_end_state());
	int tms_count = tap_get_tms_path_len(tap_get_state(), tap_get_end_state());

	for (i = skip; i < tms_count; i++) {
		tms = (tms_scan >> i) & 1;
		bitbang_interface->write(0, tms, 0);
		bitbang_interface->write(1, tms, 0);
	}
	bitbang_interface->write(CLOCK_IDLE(), tms, 0);

	tap_set_state(tap_get_end_state());
}

/**
 * Clock a bunch of TMS (or SWDIO) transitions, to change the JTAG
 * (or SWD) state machine.
 */
static int bb_execute_tms(struct jtag_command *cmd)
{
	unsigned num_bits = cmd->cmd.tms->num_bits;
	const uint8_t *bits = cmd->cmd.tms->bits;

	DEBUG_JTAG_IO("TMS: %d bits", num_bits);

	int tms = 0;
	for (unsigned i = 0; i < num_bits; i++) {
		tms = ((bits[i/8] >> (i % 8)) & 1);
		bitbang_interface->write(0, tms, 0);
		bitbang_interface->write(1, tms, 0);
	}
	bitbang_interface->write(CLOCK_IDLE(), tms, 0);

	return ERROR_OK;
}

static void bb_path_move(struct pathmove_command *cmd)
{
	int num_states = cmd->num_states;
	int state_count;
	int tms = 0;

	state_count = 0;
	while (num_states) {
		if (tap_state_transition(tap_get_state(), false) == cmd->path[state_count])
			tms = 0;
		else if (tap_state_transition(tap_get_state(), true) == cmd->path[state_count])
			tms = 1;
		else {
			LOG_ERROR("BUG: %s -> %s isn't a valid TAP transition",
				tap_state_name(tap_get_state()),
				tap_state_name(cmd->path[state_count]));
			exit(-1);
		}

		bitbang_interface->write(0, tms, 0);
		bitbang_interface->write(1, tms, 0);

		tap_set_state(cmd->path[state_count]);
		state_count++;
		num_states--;
	}

	bitbang_interface->write(CLOCK_IDLE(), tms, 0);

	tap_set_end_state(tap_get_state());
}

static void bb_runtest(int num_cycles)
{
	int i;

	tap_state_t saved_end_state = tap_get_end_state();

	/* only do a state_move when we're not already in IDLE */
	if (tap_get_state() != TAP_IDLE) {
		bb_end_state(TAP_IDLE);
		bb_state_move(0);
	}

	/* execute num_cycles */
	for (i = 0; i < num_cycles; i++) {
		bitbang_interface->write(0, 0, 0);
		bitbang_interface->write(1, 0, 0);
	}
	bitbang_interface->write(CLOCK_IDLE(), 0, 0);

	/* finish in end_state */
	bb_end_state(saved_end_state);
	if (tap_get_state() != tap_get_end_state())
		bb_state_move(0);
}

static void bb_stableclocks(int num_cycles)
{
	int tms = (tap_get_state() == TAP_RESET ? 1 : 0);
	int i;

	/* send num_cycles clocks onto the cable */
	for (i = 0; i < num_cycles; i++) {
		bitbang_interface->write(1, tms, 0);
		bitbang_interface->write(0, tms, 0);
	}
}

static void bb_scan(bool ir_scan, enum scan_type type, uint8_t *buffer, int scan_size)
{
	tap_state_t saved_end_state = tap_get_end_state();
	int bit_cnt;

	if (!((!ir_scan &&
			(tap_get_state() == TAP_DRSHIFT)) ||
			(ir_scan && (tap_get_state() == TAP_IRSHIFT)))) {
		if (ir_scan)
			bb_end_state(TAP_IRSHIFT);
		else
			bb_end_state(TAP_DRSHIFT);

		bb_state_move(0);
		bb_end_state(saved_end_state);
	}

	for (bit_cnt = 0; bit_cnt < scan_size; bit_cnt++) {
		int val = 0;
		int tms = (bit_cnt == scan_size-1) ? 1 : 0;
		int tdi;
		int bytec = bit_cnt/8;
		int bcval = 1 << (bit_cnt % 8);

		/* if we're just reading the scan, but don't care about the output
		 * default to outputting 'low', this also makes valgrind traces more readable,
		 * as it removes the dependency on an uninitialised value
		 */
		tdi = 0;
		if ((type != SCAN_IN) && (buffer[bytec] & bcval))
			tdi = 1;

		bitbang_interface->write(0, tms, tdi);

		if (type != SCAN_OUT)
			val = bitbang_interface->read();

		bitbang_interface->write(1, tms, tdi);

		if (type != SCAN_OUT) {
			if (val)
				buffer[bytec] |= bcval;
			else
				buffer[bytec] &= ~bcval;
		}
	}

	if (tap_get_state() != tap_get_end_state()) {
		/* we *KNOW* the above loop transitioned out of
		 * the shift state, so we skip the first state
		 * and move directly to the end state.
		 */
		bb_state_move(1);
	}
}

int bb_execute_queue(void)
{
	struct jtag_command *cmd = jtag_command_queue;	/* currently processed command */
	int scan_size;
	enum scan_type type;
	uint8_t *buffer;
	int retval;

	if (!bitbang_interface) {
		LOG_ERROR("BUG: Bitbang interface called, but not yet initialized");
		exit(-1);
	}

	/* return ERROR_OK, unless a jtag_read_buffer returns a failed check
	 * that wasn't handled by a caller-provided error handler
	 */
	retval = ERROR_OK;

	if (bitbang_interface->blink)
		bitbang_interface->blink(1);

	while (cmd) {
		printf(">> CMD = %d\n", cmd->type);
		switch (cmd->type) {
			case JTAG_RESET:
#ifdef _DEBUG_JTAG_IO_
				LOG_DEBUG("reset trst: %i srst %i",
				cmd->cmd.reset->trst,
				cmd->cmd.reset->srst);
#endif
				if ((cmd->cmd.reset->trst == 1) ||
						(cmd->cmd.reset->srst && (jtag_get_reset_config() & RESET_SRST_PULLS_TRST)))
					tap_set_state(TAP_RESET);
				bitbang_interface->reset(cmd->cmd.reset->trst, cmd->cmd.reset->srst);
				break;
			case JTAG_RUNTEST:
#ifdef _DEBUG_JTAG_IO_
				LOG_DEBUG("runtest %i cycles, end in %s",
						cmd->cmd.runtest->num_cycles,
						tap_state_name(cmd->cmd.runtest->end_state));
#endif
				bb_end_state(cmd->cmd.runtest->end_state);
				bb_runtest(cmd->cmd.runtest->num_cycles);
				break;

			case JTAG_STABLECLOCKS:
				/* this is only allowed while in a stable state.  A check for a stable
				 * state was done in jtag_add_clocks()
				 */
				bb_stableclocks(cmd->cmd.stableclocks->num_cycles);
				break;

			case JTAG_TLR_RESET:
#ifdef _DEBUG_JTAG_IO_
				LOG_DEBUG("statemove end in %s",
						tap_state_name(cmd->cmd.statemove->end_state));
#endif
				bb_end_state(cmd->cmd.statemove->end_state);
				bb_state_move(0);
				break;
			case JTAG_PATHMOVE:
#ifdef _DEBUG_JTAG_IO_
				LOG_DEBUG("pathmove: %i states, end in %s",
						cmd->cmd.pathmove->num_states,
						tap_state_name(cmd->cmd.pathmove->path[cmd->cmd.pathmove->num_states - 1]));
#endif
				bb_path_move(cmd->cmd.pathmove);
				break;
			case JTAG_SCAN:
#ifdef _DEBUG_JTAG_IO_
				LOG_DEBUG("%s scan end in %s",
						(cmd->cmd.scan->ir_scan) ? "IR" : "DR",
					tap_state_name(cmd->cmd.scan->end_state));
#endif
				bb_end_state(cmd->cmd.scan->end_state);
				scan_size = jtag_build_buffer(cmd->cmd.scan, &buffer);
				type = jtag_scan_type(cmd->cmd.scan);
				bb_scan(cmd->cmd.scan->ir_scan, type, buffer, scan_size);
				if (jtag_read_buffer(buffer, cmd->cmd.scan) != ERROR_OK)
					retval = ERROR_JTAG_QUEUE_FAILED;
				if (buffer)
					free(buffer);
				break;
			case JTAG_SLEEP:
#ifdef _DEBUG_JTAG_IO_
				LOG_DEBUG("sleep %" PRIi32, cmd->cmd.sleep->us);
#endif
				jtag_sleep(cmd->cmd.sleep->us);
				break;
			case JTAG_TMS:
				retval = bb_execute_tms(cmd);
				break;
			default:
				LOG_ERROR("BUG: unknown JTAG command type encountered");
				exit(-1);
		}
		cmd = cmd->next;
	}
	if (bitbang_interface->blink)
		bitbang_interface->blink(0);

	return retval;
}
