#include "hardware/dma.h"

#ifndef TMI_h
#define TMI_h
#include <Arduino.h>
#include "mapped_input_types.h"

class TMI {
  public:
    TMI( );
    TMI( uint8_t mosi_pin, uint8_t miso_pin, uint8_t sclk_pin, uint8_t csn_pin );
    bool                    init( );
    void                    register_event_callback( tmi_event_callback_t cb );
    bool                    data_ready( void );
    bool                    send_data( void * d, uint8_t len );
    void                    set_legacy_mode( bool enable );
    void                    handle_rx_data();
    void                    send_firmware_rev( uint32_t firmware_rev );
    void                    send_event( uint8_t event_id );
    uint8_t                 get_cs_pin( void );
    void                    csn_irq( uint gpio, uint32_t event_mask );
  
  private:
    uint8_t                 _miso;
    uint8_t                 _mosi;
    uint8_t                 _sclk; 
    uint8_t                 _csn;
    uint8_t                 _rx_dma_chan;
    dma_channel_config      _rx_dma_conf;
    uint8_t                 _tx_dma_chan;
    dma_channel_config      _tx_dma_conf;
    mi_master_frame_t       _rx_buff;
    mi_slave_frame_t        _tx_buff;
    bool                    _dma_done;
    volatile bool           _slave_port_busy;
    uint8_t                 _backchannel_length;
    uint8_t                 _backchannel_buf[32];
    uint8_t                 _event_queue[16];
    uint8_t                 _event_queue_push_i;
    uint8_t                 _event_queue_pull_i;
    uint8_t                 _protocol_revision;
    tmi_event_callback_t    _event_cb;
};

#endif



