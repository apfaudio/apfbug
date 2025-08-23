#include "openfpgaloader.h"

#ifdef UNIT_TEST
// Stub definitions for unit testing
uint32_t cmd_handle(pio_jtag_inst_t* jtag, uint8_t* rxbuf, uint32_t count, uint8_t* tx_buf, bool local_host);
enum SignalIdentifier {
  SIG_TCK = 1 << 1,
  SIG_TDI = 1 << 2,
  SIG_TDO = 1 << 3,
  SIG_TMS = 1 << 4,
  SIG_TRST = 1 << 5,
  SIG_SRST = 1 << 6
};
#else
#include "cmd.h"
#endif

#include <string.h>
#include <stdlib.h>
#include <assert.h>

// DirtyJTAG command definitions (from dirtyJtag.cpp)
enum dirtyJtagCmd {
    CMD_STOP = 0x00,
    CMD_INFO = 0x01,
    CMD_FREQ = 0x02,
    CMD_XFER = 0x03,
    CMD_SETSIG = 0x04,
    CMD_GETSIG = 0x05,
    CMD_CLK = 0x06
};

enum CommandModifier {
    EXTEND_LENGTH = 0x40,
    NO_READ = 0x80
};

// JTAG TAP States
typedef enum {
    TEST_LOGIC_RESET = 0,
    RUN_TEST_IDLE = 1,
    SELECT_DR_SCAN = 2,
    CAPTURE_DR = 3,
    SHIFT_DR = 4,
    EXIT1_DR = 5,
    PAUSE_DR = 6,
    EXIT2_DR = 7,
    UPDATE_DR = 8,
    SELECT_IR_SCAN = 9,
    CAPTURE_IR = 10,
    SHIFT_IR = 11,
    EXIT1_IR = 12,
    PAUSE_IR = 13,
    EXIT2_IR = 14,
    UPDATE_IR = 15,
} tap_state_t;

#define TMS_BUFFER_SZ 128

static pio_jtag_inst_t* _jtag;
static int _num_tms = 0;
static uint8_t _curr_tdi = 1;
static tap_state_t _state = RUN_TEST_IDLE;
static uint8_t _tms_buffer[TMS_BUFFER_SZ] = {0};

void ojtag_set_tms(unsigned char tms);
void jtag_flush_tms(bool flush_buffer);
void jtag_set_state(tap_state_t newState);

void dirtyjtag_write_tms(const uint8_t *tms, uint32_t len);
int dirtyjtag_write_tdi(const uint8_t *tx, uint8_t *rx, uint32_t len, bool end);

void jtag_init(pio_jtag_inst_t* jtag)
{
    _jtag = jtag;
}

void ojtag_set_tms(unsigned char tms)
{
	if (_num_tms+1 == TMS_BUFFER_SZ * 8)
		jtag_flush_tms(false);
	if (tms != 0)
		_tms_buffer[_num_tms>>3] |= (0x1) << (_num_tms & 0x7);
	_num_tms++;
}

void jtag_flush_tms(bool flush_buffer)
{
	int ret = 0;
	if (_num_tms != 0) {
		dirtyjtag_write_tms(_tms_buffer, _num_tms);
		memset(_tms_buffer, 0, TMS_BUFFER_SZ);
		_num_tms = 0;
	}
}

void jtag_set_state(tap_state_t newState)
{
    _curr_tdi = 1;
	unsigned char tms = 0;
	while (newState != _state) {
		switch (_state) {
		case TEST_LOGIC_RESET:
			if (newState == TEST_LOGIC_RESET) {
				tms = 1;
			} else {
				tms = 0;
				_state = RUN_TEST_IDLE;
			}
			break;
		case RUN_TEST_IDLE:
			if (newState == RUN_TEST_IDLE) {
				tms = 0;
			} else {
				tms = 1;
				_state = SELECT_DR_SCAN;
			}
			break;
		case SELECT_DR_SCAN:
			switch (newState) {
			case CAPTURE_DR:
			case SHIFT_DR:
			case EXIT1_DR:
			case PAUSE_DR:
			case EXIT2_DR:
			case UPDATE_DR:
				tms = 0;
				_state = CAPTURE_DR;
				break;
			default:
				tms = 1;
				_state = SELECT_IR_SCAN;
			}
			break;
		case SELECT_IR_SCAN:
			switch (newState) {
			case CAPTURE_IR:
			case SHIFT_IR:
			case EXIT1_IR:
			case PAUSE_IR:
			case EXIT2_IR:
			case UPDATE_IR:
				tms = 0;
				_state = CAPTURE_IR;
				break;
			default:
				tms = 1;
				_state = TEST_LOGIC_RESET;
			}
			break;
			/* DR column */
		case CAPTURE_DR:
			if (newState == SHIFT_DR) {
				tms = 0;
				_state = SHIFT_DR;
			} else {
				tms = 1;
				_state = EXIT1_DR;
			}
			break;
		case SHIFT_DR:
			if (newState == SHIFT_DR) {
				tms = 0;
			} else {
				tms = 1;
				_state = EXIT1_DR;
			}
			break;
		case EXIT1_DR:
			switch (newState) {
			case PAUSE_DR:
			case EXIT2_DR:
			case SHIFT_DR:
			case EXIT1_DR:
				tms = 0;
				_state = PAUSE_DR;
				break;
			default:
				tms = 1;
				_state = UPDATE_DR;
			}
			break;
		case PAUSE_DR:
			if (newState == PAUSE_DR) {
				tms = 0;
			} else {
				tms = 1;
				_state = EXIT2_DR;
			}
			break;
		case EXIT2_DR:
			switch (newState) {
			case SHIFT_DR:
			case EXIT1_DR:
			case PAUSE_DR:
				tms = 0;
				_state = SHIFT_DR;
				break;
			default:
				tms = 1;
				_state = UPDATE_DR;
			}
			break;
		case UPDATE_DR:
		case UPDATE_IR:
			if (newState == RUN_TEST_IDLE) {
				tms = 0;
				_state = RUN_TEST_IDLE;
			} else {
				tms = 1;
				_state = SELECT_DR_SCAN;
			}
			break;
			/* IR column */
		case CAPTURE_IR:
			if (newState == SHIFT_IR) {
				tms = 0;
				_state = SHIFT_IR;
			} else {
				tms = 1;
				_state = EXIT1_IR;
			}
			break;
		case SHIFT_IR:
			if (newState == SHIFT_IR) {
				tms = 0;
			} else {
				tms = 1;
				_state = EXIT1_IR;
			}
			break;
		case EXIT1_IR:
			switch (newState) {
			case PAUSE_IR:
			case EXIT2_IR:
			case SHIFT_IR:
			case EXIT1_IR:
				tms = 0;
				_state = PAUSE_IR;
				break;
			default:
				tms = 1;
				_state = UPDATE_IR;
			}
			break;
		case PAUSE_IR:
			if (newState == PAUSE_IR) {
				tms = 0;
			} else {
				tms = 1;
				_state = EXIT2_IR;
			}
			break;
		case EXIT2_IR:
			switch (newState) {
			case SHIFT_IR:
			case EXIT1_IR:
			case PAUSE_IR:
				tms = 0;
				_state = SHIFT_IR;
				break;
			default:
				tms = 1;
				_state = UPDATE_IR;
			}
			break;
		default:
            /* TODO */
            break;
		}
		ojtag_set_tms(tms);
	}
	jtag_flush_tms(false);
}

void dirtyjtag_write_tms(const uint8_t *tms, uint32_t len)
{
	if (len == 0)
		return;
	uint8_t mask = SIG_TCK | SIG_TMS;
	uint8_t buf[64];
	uint32_t buffer_idx = 0;
	for (uint32_t i = 0; i < len; i++)
	{
		uint8_t val = (tms[i >> 3] & (1 << (i & 0x07))) ? SIG_TMS : 0;
		buf[buffer_idx++] = CMD_SETSIG;
		buf[buffer_idx++] = mask;
		buf[buffer_idx++] = val;
		buf[buffer_idx++] = CMD_SETSIG;
		buf[buffer_idx++] = mask;
		buf[buffer_idx++] = val | SIG_TCK;
		if ((buffer_idx + 9) >= sizeof(buf) || (i == len - 1)) {
			// flush the buffer
			if (i == len - 1) {
				// insert tck falling edge
				buf[buffer_idx++] = CMD_SETSIG;
				buf[buffer_idx++] = mask;
				buf[buffer_idx++] = val;
			}
			buf[buffer_idx++] = CMD_STOP;
            cmd_handle(_jtag, buf, buffer_idx, NULL, true);
			buffer_idx = 0;
		}
	}
}

void jtag_read_write(const uint8_t *tdi, unsigned char *tdo, int len, char last)
{
	jtag_flush_tms(false);
	dirtyjtag_write_tdi(tdi, tdo, len, last);
	if (last == 1)
		_state = (_state == SHIFT_DR) ? EXIT1_DR : EXIT1_IR;
}

void jtag_shift_ir(uint8_t *tdi, uint8_t *tdo, int irlen, tap_state_t end_state) {

	if (_state != SHIFT_IR) {
		jtag_set_state(SHIFT_IR);
	}

	/* write tdi (and read tdo) to the selected device
	 * end (ie TMS high) is used only when current device
	 * is the last of the chain and a state change must
	 * be done
	 */
	jtag_read_write(tdi, tdo, irlen, end_state != SHIFT_IR);

	/* it's asked to move out of SHIFT IR state */
	if (end_state != SHIFT_IR) {
		/* move to the requested state */
		jtag_set_state(end_state);
	}
}

int jtag_shift_dr(const uint8_t *tdi, unsigned char *tdo, int drlen, tap_state_t end_state)
{
	/* if current state not shift DR
	 * move to this state
	 */
	if (_state != SHIFT_DR) {
		jtag_set_state(SHIFT_DR);
		jtag_flush_tms(false);  // force transmit tms state
	}

	/* write tdi (and read tdo) to the selected device
	 * end (ie TMS high) is used only when current device
	 * is the last of the chain and a state change must
	 * be done
	 */
	jtag_read_write(tdi, tdo, drlen, end_state != SHIFT_DR);

	/* if it's asked to move in FSM */
	if (end_state != SHIFT_DR) {
		jtag_set_state(end_state);
	}
	return 0;
}

void dirtyjtag_toggle_clk(uint8_t tms, uint8_t tdi, uint32_t clk_len)
{
	int actual_length;
	uint8_t buf[] = {CMD_CLK,
				(uint8_t)(((tms) ? SIG_TMS : 0) | ((tdi) ? SIG_TDI : 0)),
				0,
				CMD_STOP};
	while (clk_len > 0) {
		buf[2] = (clk_len > 64) ? 64 : (uint8_t)clk_len;
        /*
		int ret = libusb_bulk_transfer(dev_handle, DIRTYJTAG_WRITE_EP,
				buf, 4, &actual_length, DIRTYJTAG_TIMEOUT);
                */
        cmd_handle(_jtag, buf, 4, NULL, true);
		clk_len -= buf[2];
	}
}

#define OPTIONS_NO_READ 0
#define OPTIONS_MAX_BITS 128

int dirtyjtag_write_tdi(const uint8_t *tx, uint8_t *rx, uint32_t len, bool end)
{
	int actual_length;
	uint32_t real_bit_len = len - (end ? 1 : 0);
	uint32_t kRealByteLen = (len + 7) / 8;

	uint8_t tx_cpy[kRealByteLen];
	uint8_t tx_buf[512], rx_buf[512];
	uint8_t *tx_ptr, *rx_ptr = rx;

	if (tx)
		memcpy(tx_cpy, tx, kRealByteLen);
	else
		memset(tx_cpy, 0, kRealByteLen);
	tx_ptr = tx_cpy;

	tx_buf[0] = CMD_XFER | (rx ? 0 : OPTIONS_NO_READ);
	uint16_t max_bit_transfer_length = OPTIONS_MAX_BITS;
	// need to cut the bits on byte size.
	assert(max_bit_transfer_length % 8 == 0);
	while (real_bit_len != 0) {
		uint16_t bit_to_send = (real_bit_len > max_bit_transfer_length) ?
			max_bit_transfer_length : real_bit_len;
		size_t byte_to_send = (bit_to_send + 7) / 8;
		size_t header_offset = 0;
		if (bit_to_send > 255) {
			tx_buf[0] |= EXTEND_LENGTH;
			tx_buf[1] = bit_to_send - 256;
			header_offset = 2;
		} else {
			tx_buf[0] &= ~EXTEND_LENGTH;
			tx_buf[1] = bit_to_send;
			header_offset = 2;
		}
		memset(tx_buf + header_offset, 0, byte_to_send);
		for (int i = 0; i < bit_to_send; i++)
			if (tx_ptr[i >> 3] & (1 << (i & 0x07)))
				tx_buf[header_offset + (i >> 3)] |= (0x80 >> (i & 0x07));

		actual_length = 0;

        cmd_handle(_jtag, tx_buf, (byte_to_send+header_offset), rx_buf, true);
        /*
		int ret = libusb_bulk_transfer(dev_handle, DIRTYJTAG_WRITE_EP,
				(unsigned char *)tx_buf, (byte_to_send + header_offset),
				&actual_length, DIRTYJTAG_TIMEOUT);
		if ((ret < 0) || (actual_length != (int)(byte_to_send + header_offset))) {
			cerr << "writeTDI: fill: usb bulk write failed " << ret <<
				"actual length: " << actual_length << endl;
			return EXIT_FAILURE;
		}
		// cerr << actual_length << ", " << bit_to_send << endl;

		if (rx || (_version <= 1)) {
			int transfer_length = (bit_to_send > 255) ? byte_to_send :32;
			do {
				ret = libusb_bulk_transfer(dev_handle, DIRTYJTAG_READ_EP,
					rx_buf, transfer_length, &actual_length, DIRTYJTAG_TIMEOUT);
				if (ret < 0) {
					cerr << "writeTDI: read: usb bulk read failed " << ret << endl;
					return EXIT_FAILURE;
				}
			} while (actual_length == 0);
			assert((size_t)actual_length >= byte_to_send);
		}
        */

		if (rx) {
			for (int i = 0; i < bit_to_send; i++)
				rx_ptr[i >> 3] = (rx_ptr[i >> 3] >> 1) |
						(((rx_buf[i >> 3] << (i&0x07)) & 0x80));
			rx_ptr += byte_to_send;
		}

		real_bit_len -= bit_to_send;
		tx_ptr += byte_to_send;
	}

	/* this step exist only with [D|I]R_SHIFT */
	if (end) {
		int pos = len-1;
		uint8_t sig;
		unsigned char last_bit =
				(tx_cpy[pos >> 3] & (1 << (pos & 0x07))) ? SIG_TDI: 0;

		uint8_t mask = SIG_TMS | SIG_TDI;
		uint8_t val = SIG_TMS | (last_bit);

		if (rx)
		{
			mask |= SIG_TCK;
			uint8_t buf[] = {
				CMD_SETSIG,
				mask,
				val,
				CMD_SETSIG,
				mask,
				val | SIG_TCK,
				CMD_GETSIG,  // <---Read instruction
				CMD_STOP,
			};
            cmd_handle(_jtag, buf, sizeof(buf), &sig, true);
            /*
			if (libusb_bulk_transfer(dev_handle, DIRTYJTAG_WRITE_EP,
									 buf, sizeof(buf), &actual_length,
									 DIRTYJTAG_TIMEOUT) < 0)
			{
				cerr << "writeTDI: last bit error: usb bulk write failed 1" << endl;
				return -EXIT_FAILURE;
			}
			do
			{
				if (libusb_bulk_transfer(dev_handle, DIRTYJTAG_READ_EP,
											&sig, 1, &actual_length,
											DIRTYJTAG_TIMEOUT) < 0)
				{
					cerr << "writeTDI: last bit error: usb bulk read failed" << endl;
					return -EXIT_FAILURE;
				}
			} while (actual_length == 0);
            */
			rx[pos >> 3] >>= 1;
			if (sig & SIG_TDO)
			{
				rx[pos >> 3] |= (1 << (pos & 0x07));
			}
			buf[2] &= ~SIG_TCK;
			buf[3] = CMD_STOP;
            cmd_handle(_jtag, buf, 4, NULL, true);
            /*
			if (libusb_bulk_transfer(dev_handle, DIRTYJTAG_WRITE_EP,
									 buf, 4, &actual_length,
									 DIRTYJTAG_TIMEOUT) < 0)
			{
				cerr << "writeTDI: last bit error: usb bulk write failed 2" << endl;
				return -EXIT_FAILURE;
			}
            */

		} else {
            dirtyjtag_toggle_clk(SIG_TMS, last_bit, 1);
		}
	}
	return EXIT_SUCCESS;
}

// Generate idle clocks
void jtag_go_idle_clocks(pio_jtag_inst_t* jtag, int clocks) {
    uint8_t cmd_buf[4];
    cmd_buf[0] = CMD_CLK;
    cmd_buf[1] = 0;  // TMS=0, TDI=0 (idle state)
    
    while (clocks > 0) {
        int clk_chunk = (clocks > 255) ? 255 : clocks;
        cmd_buf[2] = clk_chunk;
        cmd_buf[3] = CMD_STOP;
        
        uint8_t dummy[4];
        cmd_handle(jtag, cmd_buf, 4, dummy, true);
        clocks -= clk_chunk;
    }
}

// Core Lattice function: write/read operation
// Direct port of openFPGALoader's Lattice::wr_rd function
bool lattice_wr_rd(pio_jtag_inst_t* jtag, uint8_t cmd, 
                   const uint8_t* tx, int tx_len, 
                   uint8_t* rx, int rx_len) {
    int kXferLen = rx_len;
    if (tx_len > rx_len)
        kXferLen = tx_len;

    uint8_t* xfer_tx = malloc(kXferLen);
    uint8_t* xfer_rx = malloc(kXferLen);
    if (!xfer_tx || !xfer_rx) {
        free(xfer_tx);
        free(xfer_rx);
        return false;
    }

    memset(xfer_tx, 0, kXferLen);
    if (tx != NULL && tx_len > 0) {
        for (int i = 0; i < tx_len; i++)
            xfer_tx[i] = tx[i];
    }

    // Step 1: shiftIR(&cmd, NULL, 8, Jtag::PAUSE_IR)
    jtag_shift_ir(&cmd, NULL, 8, PAUSE_IR);

    // Step 2: shiftDR(xfer_tx, xfer_rx, 8 * kXferLen, Jtag::PAUSE_DR)
    if (rx || tx) {
        jtag_shift_dr(xfer_tx, (rx) ? xfer_rx : NULL, 8 * kXferLen, PAUSE_DR);
    }

    // Copy response data
    if (rx) {
        for (int i = 0; i < rx_len; i++)
            rx[i] = xfer_rx[i];
    }

    free(xfer_tx);
    free(xfer_rx);
    return true;
}

// Poll busy flag - direct port of openFPGALoader's pollBusyFlag
bool lattice_poll_busy_flag(pio_jtag_inst_t* jtag) {
    uint8_t rx;
    int timeout = 0;
    do {
        if (!lattice_wr_rd(jtag, LSC_CHECK_BUSY, NULL, 0, &rx, 1))
            return false;
        jtag_go_idle_clocks(jtag, 100);  // Some idle clocks
        if (timeout == 100000) {
            return false;  // timeout
        } else {
            timeout++;
        }
    } while (rx != 0);

    return true;
}

// High-level ECP5 functions - direct ports from openFPGALoader

uint32_t ecp5_jtag_read_id(pio_jtag_inst_t* jtag) {
    // Direct port of Lattice::idCode()
    uint8_t device_id[4];
    if (!lattice_wr_rd(jtag, READ_ID, NULL, 0, device_id, 4))
        return 0;
        
    return device_id[3] << 24 |
           device_id[2] << 16 |
           device_id[1] << 8  |
           device_id[0];
}

bool ecp5_jtag_check_busy(pio_jtag_inst_t* jtag) {
    // Direct port of pollBusyFlag for single check
    uint8_t rx;
    if (!lattice_wr_rd(jtag, LSC_CHECK_BUSY, NULL, 0, &rx, 1))
        return true;  // Assume busy on error
    return (rx & 1) != 0;
}

void ecp5_jtag_enable_config(pio_jtag_inst_t* jtag) {
    // Direct port of Lattice::EnableISC(0x00)
    uint8_t flash_mode = 0x00;
    lattice_wr_rd(jtag, ISC_ENABLE, &flash_mode, 1, NULL, 0);
    
    jtag_go_idle_clocks(jtag, 1000);
    lattice_poll_busy_flag(jtag);
}

void ecp5_jtag_disable_config(pio_jtag_inst_t* jtag) {
    // Direct port of Lattice::DisableISC()
    lattice_wr_rd(jtag, ISC_DISABLE, NULL, 0, NULL, 0);
    
    jtag_go_idle_clocks(jtag, 1000);
    lattice_poll_busy_flag(jtag);
}

void ecp5_jtag_erase(pio_jtag_inst_t* jtag) {
    // Direct port of SRAM erase from openFPGALoader
    uint8_t erase_op = FLASH_ERASE_SRAM;  // Erase SRAM only
    lattice_wr_rd(jtag, ISC_ERASE, &erase_op, 1, NULL, 0);
    
    jtag_go_idle_clocks(jtag, 1000);
    lattice_poll_busy_flag(jtag);
}

// Helper function to reverse bits in a byte (port of ConfigBitstreamParser::reverseByte)
static uint8_t reverse_byte(uint8_t b) {
    uint8_t reversed = 0;
    for (int i = 0; i < 8; i++) {
        reversed = (reversed << 1) | (b & 1);
        b >>= 1;
    }
    return reversed;
}

void ecp5_jtag_load_bitstream(pio_jtag_inst_t* jtag, const uint8_t* bitstream_data, uint32_t size) {
    // Direct port of openFPGALoader's corrected bitstream loading implementation
    
    // Step 1: LSC_INIT_ADDRESS (0x46) - Initialize address pointer
    if (!lattice_wr_rd(jtag, 0x46, NULL, 0, NULL, 0))
        return;
    jtag_go_idle_clocks(jtag, 1000);
    
    // Step 2: LSC_BITSTREAM_BURST (0x7A) - Enter bitstream mode
    if (!lattice_wr_rd(jtag, LSC_BITSTREAM_BURST, NULL, 0, NULL, 0))
        return;
    jtag_go_idle_clocks(jtag, 2);
    
    // Step 3: Send bitstream data in chunks with byte reversal
    const uint32_t chunk_size = 1024;  // Use 1024 bytes like openFPGALoader
    uint32_t bytes_sent = 0;
    uint8_t* tmp_buffer = malloc(chunk_size);
    if (!tmp_buffer) return;
    
    while (bytes_sent < size) {
        uint32_t bytes_to_send = (size - bytes_sent > chunk_size) ? chunk_size : (size - bytes_sent);
        tap_state_t next_state = (bytes_sent + bytes_to_send >= size) ? RUN_TEST_IDLE : SHIFT_DR;
        
        // Apply byte reversal like openFPGALoader: reverseByte(data[i+ii])
        for (uint32_t ii = 0; ii < bytes_to_send; ii++) {
            tmp_buffer[ii] = reverse_byte(bitstream_data[bytes_sent + ii]);
        }
        
        // Send chunk to DR with appropriate end state
        jtag_shift_dr(tmp_buffer, NULL, bytes_to_send * 8, next_state);
        bytes_sent += bytes_to_send;
    }
    
    free(tmp_buffer);
    
    // Step 4: Final idle clocks and return to RUN_TEST_IDLE
    jtag_go_idle_clocks(jtag, 1000);
}

void ecp5_jtag_refresh(pio_jtag_inst_t* jtag) {
    // Direct port of Lattice::loadConfiguration()
    lattice_wr_rd(jtag, LSC_REFRESH, NULL, 0, NULL, 0);
    
    jtag_go_idle_clocks(jtag, 1000);
    lattice_poll_busy_flag(jtag);
}
