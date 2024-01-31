#include "tundra_mapped_input.h"
#include "hardware/spi.h"
#include "hardware/irq.h"
#include "hardware/dma.h"
#include "hardware/gpio.h"

TMI::TMI( )
{
  // Pinout as defined by the Tundra Labs Breakout
  _miso = 15;
  _mosi = 12;
  _sclk = 14;
  _csn = 13;
}

TMI::TMI(uint8_t mosi_pin, uint8_t miso_pin, uint8_t sclk_pin, uint8_t csn_pin)
{
  _miso = miso_pin;
  _mosi = mosi_pin;
  _sclk = sclk_pin;
  _csn = csn_pin;
}

bool TMI::init( )
{
  gpio_pull_up (_csn);
  gpio_set_input_enabled (_csn, true);

  gpio_set_function(_csn, GPIO_FUNC_SPI);//CSn
  gpio_set_function(_sclk, GPIO_FUNC_SPI);//SCK
  gpio_set_function(_miso, GPIO_FUNC_SPI);//MISO
  gpio_set_function(_mosi, GPIO_FUNC_SPI);//MOSI

  _spi_init(spi1, 8000000);
  spi_set_format(spi1, 8, SPI_CPOL_1, SPI_CPHA_1, SPI_MSB_FIRST);
  spi_set_slave(spi1, true);

  _rx_dma_chan = dma_claim_unused_channel( true );
  _rx_dma_conf = dma_channel_get_default_config( _rx_dma_chan );
  channel_config_set_transfer_data_size( &_rx_dma_conf, DMA_SIZE_8 );
  channel_config_set_read_increment( &_rx_dma_conf, false );
  channel_config_set_write_increment( &_rx_dma_conf, true );
  channel_config_set_dreq( &_rx_dma_conf, DREQ_SPI1_RX );
  dma_channel_configure( _rx_dma_chan, &_rx_dma_conf, &_rx_buff, &spi_get_hw(spi1)->dr, sizeof(_rx_buff) , false);

  _tx_dma_chan = dma_claim_unused_channel( true );
  _tx_dma_conf = dma_channel_get_default_config( _tx_dma_chan );
  channel_config_set_transfer_data_size( &_tx_dma_conf, DMA_SIZE_8 );
  channel_config_set_read_increment( &_tx_dma_conf, true );
  channel_config_set_write_increment( &_tx_dma_conf, false );
  channel_config_set_dreq( &_tx_dma_conf, DREQ_SPI1_TX );
  dma_channel_configure( _tx_dma_chan, &_tx_dma_conf, &spi_get_hw(spi1)->dr, &_tx_buff, sizeof(_tx_buff) , true);
  
  _dma_done = false;

  _tx_buff.header.spi_protocol_rev = 1;
  _tx_buff.header.report_mode  = MI_PROTOCOL_REVISON_GENERIC;

  _event_queue_push_i = 0;
  _event_queue_pull_i = 0;

  return true;
}

uint8_t TMI::get_cs_pin( void )
{
  return _csn;
}

bool TMI::data_ready( void )
{
  if ( _dma_done )
  {
    _dma_done = false;
    return true;
  }
  return false;
}

void TMI::set_legacy_mode( bool enable )
{
  if ( enable )
  {
  _tx_buff.header.report_mode  = MI_PROTOCOL_REVISON_V1_CONTROLLER;
  }
  else
  {
  _tx_buff.header.report_mode  = MI_PROTOCOL_REVISON_GENERIC;
  }
}


bool TMI::send_data( void * d, uint8_t len )
{
  memcpy( _tx_buff.data, d, len);
  _tx_buff.header.input_data_length = len;
  return true;
}

void TMI::send_event(uint8_t event_id)
{
  _event_queue[_event_queue_push_i] = event_id;
  _event_queue_push_i = (_event_queue_push_i+1) & 0x0F;
}

void TMI::send_firmware_rev( uint32_t firmware_rev )
{
  _event_queue[_event_queue_push_i] = MI_SLAVE_EVENT_FIRMWARE_REV_RESP;
  _event_queue_push_i = (_event_queue_push_i+1) & 0x0F;
  _event_queue[_event_queue_push_i] = ( firmware_rev ) && 0xFF;
  _event_queue_push_i = (_event_queue_push_i+1) & 0x0F;
  _event_queue[_event_queue_push_i] = ( firmware_rev >> 8 ) && 0xFF;
  _event_queue_push_i = (_event_queue_push_i+1) & 0x0F;
  _event_queue[_event_queue_push_i] = ( firmware_rev >> 16 ) && 0xFF;
  _event_queue_push_i = (_event_queue_push_i+1) & 0x0F;
  _event_queue[_event_queue_push_i] = ( firmware_rev >> 24 ) && 0xFF;
  _event_queue_push_i = (_event_queue_push_i+1) & 0x0F;
}

void TMI::handle_rx_data()
{
  uint8_t revision;
  uint8_t evt_opcode;
  uint8_t data_index         = 0;
  uint8_t evt_index          = 0;
  uint8_t evt_payload_length = 0;

  if(_tx_buff.header.spi_protocol_rev == 0x01)
  {
    /**
    if(_rx_buff.header.status.enter_bootloader == 1)                                                                     // Manage bootloader status
    {
      <<enter code here to jump to bootloader>>
    }
    **/
    while(data_index < _rx_buff.header.backchannel_length)                                                               // Loop past any backchannel data to set the "data_index" counter
    {
      data_index++;                                                                                                     // "data_index" is now set at the first byte of the first event
    }
    while(evt_index < _rx_buff.header.event_data_length)
    {
      if(_event_cb)
      {
        evt_opcode = _rx_buff.buffer[data_index];                                                                        // Read the event code; it's next in the Rx buffer
        data_index++;
        switch (evt_opcode)                                                                                                // Switch case to set the event payload length
        {
          case MI_MASTER_EVENT_HAPTIC_PULSE:
            evt_payload_length = 7;
            break;
          case MI_MASTER_EVENT_HAPTIC_SONG:
            evt_payload_length = 1;
            break;
        }
        if(evt_payload_length > 0)                                                                                          // Call the callback with event data
        {
          if ( _event_cb )
          {
            _event_cb(evt_opcode, evt_payload_length, (char*) &_rx_buff.buffer[data_index]);
          }
          data_index += evt_payload_length;
        }
        else
        {
          _event_cb(evt_opcode, 0, NULL);                                                                                   // Call the callback without event data
        }
      }
      else
      {
        break;
      }
      evt_index += evt_payload_length + 1;
    }
    memset(&_rx_buff, 0, sizeof(_rx_buff));                                                                             // Clear residual garbage in the Rx buffer
  }
}

void TMI::register_event_callback(tmi_event_callback_t cb)
{
  _event_cb = cb;
}

void TMI::csn_irq( uint gpio, uint32_t event_mask )
{
  if (gpio_get(_csn))
  {
    // CSn de-assert
    dma_channel_abort( _rx_dma_chan );
    dma_channel_abort( _tx_dma_chan );
    _dma_done = true;
    
    _tx_buff.header.frame_id++;
    
    dma_channel_set_write_addr( _rx_dma_chan, &_rx_buff, true);
    dma_channel_set_read_addr(  _tx_dma_chan, &_tx_buff, true);
  }  
}