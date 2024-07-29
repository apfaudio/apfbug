#ifndef HEATSHRINK_CONFIG_H
#define HEATSHRINK_CONFIG_H

/* Required parameters for static configuration */
#define HEATSHRINK_STATIC_INPUT_BUFFER_SIZE 32
#define HEATSHRINK_STATIC_WINDOW_BITS 8
#define HEATSHRINK_STATIC_LOOKAHEAD_BITS 4

/* Turn on logging for debugging. */
#define HEATSHRINK_DEBUGGING_LOGS 0

/* Use indexing for faster compression. (This requires additional space.) */
#define HEATSHRINK_USE_INDEX 1

#endif
