#ifndef __MAPPED_INPUT_TYPES_H__
#define __MAPPED_INPUT_TYPES_H__

#define MI_MAXIMUM_UPSTREAM_BACKCHANNEL_LEN    (60)
#define MI_MAXIMUM_DOWNSTREAM_BACKCHANNEL_LEN  (60)

#define MI_MAXIMUM_FRAME_BUFFER_LEN            (64)

#define MI_DFU_SPI_PROTOCOL_REV                (0xFD)

typedef void (*tmi_event_callback_t)(uint8_t event_code,uint8_t payload_len, char* payload);

#pragma pack(push)

typedef struct __attribute__( ( packed, aligned( 1 ) ) )
{
  uint8_t                 frame_error: 1;
  uint8_t                 unsupported_spi_protocol_rev: 1;
  uint8_t                 upstream_backchannel_full: 1;
  uint8_t                 is_wireless: 1;
  uint8_t                 enter_bootloader: 1;
  uint8_t                 reserved: 3;
}
mi_master_status_t;

typedef struct __attribute__( ( packed, aligned( 1 ) ) )
{
  uint8_t                 spi_protocol_rev;
  uint8_t                 frame_id;
  uint8_t                 report_mode_request;
  mi_master_status_t      status;
  uint8_t                 backchannel_length;
  uint8_t                 event_data_length;
  uint8_t                 reserved[2];
}
mi_master_header_t;

typedef struct __attribute__( ( packed, aligned( 1 ) ) )
{
  mi_master_header_t      header;
  uint8_t                 buffer [ MI_MAXIMUM_FRAME_BUFFER_LEN ];
}
mi_master_frame_t;

typedef struct __attribute__( ( packed, aligned( 1 ) ) )
{
  uint8_t                 bootloader: 1;
  uint8_t                 reserved: 7;
}
mi_slave_status_t;

typedef struct __attribute__( ( packed, aligned( 1 ) ) )
{
  uint8_t                 spi_protocol_rev;
  uint8_t                 frame_id;
  uint8_t                 report_mode;
  mi_slave_status_t       status;
  uint8_t                 input_data_length;
  uint8_t                 backchannel_length;
  uint8_t                 event_data_length;
  uint8_t                 reserved;
}
mi_slave_header_t;

typedef struct __attribute__( ( packed, aligned( 1 ) ) )
{
  mi_slave_header_t       header;
  uint8_t                 data[ MI_MAXIMUM_FRAME_BUFFER_LEN ];
}
mi_slave_frame_t;

typedef struct __attribute__( ( packed, aligned( 1 ) ) )
{
  uint8_t       system_button: 1;
  uint8_t       menu_button: 1;
  uint8_t       trackpad_button: 1;
  uint8_t       grip_button: 1;
  uint8_t       trigger_button: 1;
  uint8_t       trackpad_finger_present: 1;
  uint8_t       reserved: 2;
}
mi_legacy_mode_buttons_t;

typedef struct __attribute__( ( packed, aligned( 1 ) ) )
{
  uint8_t                   trigger;
  int16_t                   trackpad_x;
  int16_t                   trackpad_y;
  mi_legacy_mode_buttons_t  buttons;
}
mi_legacy_mode_0x01_input_data_t;

typedef struct __attribute__( ( packed, aligned( 1 ) ) )
{
  uint16_t     duration_us;
  uint16_t     count;
  uint16_t     interval_us;
  uint8_t      priority;
}
mi_haptic_pulse_t;

typedef struct __attribute__( ( packed, aligned( 1 ) ) )
{
  uint8_t      song_id;
}
mi_haptic_song_t;

typedef enum
{
  MI_PROTOCOL_REVISON_NOP                       = 0,
  MI_PROTOCOL_REVISON_V1_CONTROLLER             = 1,  // Data structure like the Vive Wand
  MI_PROTOCOL_REVISON_V2_CONTROLLER             = 2,  // Data structure like knuckles (Currently not supported)
  MI_PROTOCOL_REVISON_GENERIC                   = 3,  // Mapped input structure
  MI_PROTOCOL_REVISON_MAX
} mi_protocol_revision_t;

typedef enum
{
  MI_MASTER_EVENT_NOP                           = 0,    // No Payload
  MI_MASTER_TEST_EVENT_NO_PAYLOAD               = 1,    // Unused
  MI_MASTER_TEST_EVENT_8B_PAYLOAD               = 2,    // Unused
  MI_MASTER_EVENT_WIRLESS_PAIRING_STARTED       = 3,    // No Payload
  MI_MASTER_EVENT_WIRLESS_PAIRING_STOPPED       = 4,    // No Payload
  MI_MASTER_EVENT_WIRLESS_PAIRING_CONNECTED     = 5,    // No Payload
  MI_MASTER_EVENT_WIRLESS_PAIRING_DISCONNECTED  = 6,    // No Payload
  MI_MASTER_EVENT_MFG_CFG_RESP                  = 7,    // Depricated
  MI_MASTER_EVENT_FIRMWARE_REV_REQ              = 8,    // No Payload
  MI_MASTER_EVENT_HAPTIC_PULSE                  = 9,    // mi_haptic_pulse_t
  MI_MASTER_EVENT_HAPTIC_PULSE_REPEAT           = 10,   // Depricated
  MI_MASTER_EVENT_HAPTIC_SONG                   = 11,   // mi_haptic_song_t
  MI_MASTER_EVENT_SHUTDOWN_WARNING              = 12,   // No Payload
  MI_MASTER_EVENT_ON_CHARGER                    = 13,   // No Payload
  MI_MASTER_EVENT_OFF_CHARGER                   = 14,   // No Payload
  MI_MASTER_EVENT_POWER_GOOD                    = 15,   // No Payload
  MI_MASTER_EVENT_POWER_NOT_GOOD                = 16,   // No Payload
  MI_MASTER_EVENT_USB_HOST                      = 17,   // No Payload
  MI_MASTER_EVENT_USB_CHARGER                   = 18,   // No Payload
  MI_MASTER_EVENT_USB_UNKNOWN                   = 19,   // No Payload
  MI_MASTER_FIRMWARE_UPDATE_START               = 20,   // No Payload
  MI_MASTER_EVENT_USB_INSERTED                  = 21,   // No Payload
  MI_MASTER_EVENT_USB_REMOVED                   = 22,   // No Payload
  MI_MASTER_EVENT_USB_ENUMERATION_COMPLETE      = 23,   // No Payload
  MI_MASTER_NUM_EVENTS
} mi_master_event_t;

typedef enum
{
  MI_SLAVE_EVENT_NOP                            = 0,
  MI_SLAVE_TEST_EVENT_NO_PAYLOAD                = 1,
  MI_SLAVE_TEST_EVENT_8B_PAYLOAD                = 2,
  MI_SLAVE_EVENT_BEGIN_WIRELESS_PAIRING         = 3,
  MI_SLAVE_EVENT_MFG_CFG_REQ                    = 4,  // Depricated
  MI_SLAVE_EVENT_FIRMWARE_REV_RESP              = 5,
  MI_SLAVE_EVENT_REQUEST_RF_DISABLE             = 6,
  MI_SLAVE_NUM_EVENTS
} mi_slave_event_t;

typedef enum
{
  MI_MASTER_DFU_JUMP_TO_BOOTLOADER   = 0,
  MI_MASTER_DFU_WRITE_PAYLOAD,
  MI_MASTER_DFU_FINISH,
  MI_MASTER_DFU_LOAD,
  MI_MASTER_DFU_NUM_CMD,
} mi_master_dfu_cmd_t;

typedef enum
{
  MI_SLAVE_DFU_NOP                  = 0,
  MI_SLAVE_DFU_READY_FOR_DATA,
  MI_SLAVE_DFU_PAYLOAD_WRITE_ACK,
  MI_SLAVE_DFU_FINISH_PASS,
  MI_SLAVE_DFU_FINISH_FAIL,
  MI_SLAVE_DFU_LOAD_PASS,
  MI_SLAVE_DFU_LOAD_FAIL,
  MI_SLAVE_DFU_ABORT,
  MI_SLAVE_DFU_NUM_CMD,
} mi_slave_dfu_cmd_t;

typedef struct __attribute__( ( packed, aligned( 1 ) ) )
{
  uint8_t                   spi_protocol_rev;
  uint8_t                   frame_id;
  uint8_t                   cmd;
  uint8_t                   length;
  uint8_t                   payload[64];
}
mi_master_dfu_frame_t;

#pragma pack(pop)

#endif
