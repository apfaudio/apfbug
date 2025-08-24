#ifndef DIRTYJTAG_PROTOCOL_H
#define DIRTYJTAG_PROTOCOL_H

// DirtyJTAG protocol definitions
// These definitions are shared between cmd.c and openfpgaloader.c

// DirtyJTAG command identifiers
enum CommandIdentifier {
    CMD_STOP = 0x00,
    CMD_INFO = 0x01,
    CMD_FREQ = 0x02,
    CMD_XFER = 0x03,
    CMD_SETSIG = 0x04,
    CMD_GETSIG = 0x05,
    CMD_CLK = 0x06,
    CMD_SETVOLTAGE = 0x07,
    CMD_GOTOBOOTLOADER = 0x08
};

// Command modifiers for DirtyJTAG commands
enum CommandModifier {
    // CMD_XFER modifiers
    NO_READ = 0x80,
    EXTEND_LENGTH = 0x40,
    // CMD_CLK modifiers  
    READOUT = 0x80
};

// Signal identifiers for cmd_setsig/cmd_getsig
enum SignalIdentifier {
    SIG_TCK = 1 << 1,
    SIG_TDI = 1 << 2,
    SIG_TDO = 1 << 3,
    SIG_TMS = 1 << 4,
    SIG_TRST = 1 << 5,
    SIG_SRST = 1 << 6
};

#endif // DIRTYJTAG_PROTOCOL_H