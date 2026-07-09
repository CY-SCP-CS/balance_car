#include "remote_debug.h"

#include "zf_driver_ipc.h"

#define REMOTE_DEBUG_IPC_PARAM_HEADER 0x52440000u
#define REMOTE_DEBUG_IPC_HEADER_MASK  0xFFFFFF00u
#define REMOTE_DEBUG_IPC_CHANNEL_MASK 0x000000FFu
#define REMOTE_DEBUG_INVALID_CHANNEL  0xFFu

typedef union {
    float  value;
    uint32 bits;
} Remote_Debug_Value_t;

static uint8 g_remote_debug_initialized;

#if CY_CORE_CM7_0
static float *g_remote_debug_bindings[REMOTE_DEBUG_PARAM_CHANNELS];
static uint8 g_remote_debug_pending_channel = REMOTE_DEBUG_INVALID_CHANNEL;
#endif

static void remote_debug_ipc_callback(uint32 data)
{
#if CY_CORE_CM7_0
    if ((data & REMOTE_DEBUG_IPC_HEADER_MASK) == REMOTE_DEBUG_IPC_PARAM_HEADER) {
        uint8 channel = (uint8)(data & REMOTE_DEBUG_IPC_CHANNEL_MASK);
        g_remote_debug_pending_channel =
            channel < REMOTE_DEBUG_PARAM_CHANNELS ? channel : REMOTE_DEBUG_INVALID_CHANNEL;
        return;
    }

    if (g_remote_debug_pending_channel < REMOTE_DEBUG_PARAM_CHANNELS) {
        float *target = g_remote_debug_bindings[g_remote_debug_pending_channel];
        if (target != NULL) {
            Remote_Debug_Value_t decoded;
            decoded.bits = data;
            *target = decoded.value;
        }
    }

    g_remote_debug_pending_channel = REMOTE_DEBUG_INVALID_CHANNEL;
#else
    (void)data;
#endif
}

void remote_debug_init(void)
{
#if CY_CORE_CM7_0
    ipc_communicate_init(IPC_PORT_1, remote_debug_ipc_callback);
    g_remote_debug_initialized = 1u;
#elif CY_CORE_CM7_1
    ipc_communicate_init(IPC_PORT_2, remote_debug_ipc_callback);
    g_remote_debug_initialized = 1u;
#else
    g_remote_debug_initialized = 0u;
#endif
}

void remote_debug_bind(uint8 channel, float *target)
{
#if CY_CORE_CM7_0
    if (channel < REMOTE_DEBUG_PARAM_CHANNELS) {
        g_remote_debug_bindings[channel] = target;
    }
#else
    (void)channel;
    (void)target;
#endif
}

void remote_debug_send_param(uint8 channel, float value)
{
#if CY_CORE_CM7_1
    if (!g_remote_debug_initialized || channel >= REMOTE_DEBUG_PARAM_CHANNELS) {
        return;
    }

    Remote_Debug_Value_t encoded;
    encoded.value = value;

    if (ipc_send_data(REMOTE_DEBUG_IPC_PARAM_HEADER | (uint32)channel) == 0u) {
        (void)ipc_send_data(encoded.bits);
    }
#else
    (void)channel;
    (void)value;
#endif
}
