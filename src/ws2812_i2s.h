// ws2812_lib.h

#ifndef __WS2812_I2S_H__
#define __WS2812_I2S_H__

#include <Arduino.h>
#include <stdint.h>
#include "ws2812_i2s.h"
#include "../include/ws2812_defs.h"
#include "../include/ws2812_gamma.h"

// include C-style header
extern "C" {
#include "../include/ws2812_dma.h"
};

// left this comment in tact....credit to Charles

// All functions below this line are Public Domain 2015 Charles Lohr.
// this code may be used by anyone in any way without restriction or limitation.

// Send out WS2812 bits with coded pulses, one nibble, then the other.
static const uint16_t bitpatterns[16] =
    {
        0b1000100010001000, 0b1000100010001110, 0b1000100011101000, 0b1000100011101110,
        0b1000111010001000, 0b1000111010001110, 0b1000111011101000, 0b1000111011101110,
        0b1110100010001000, 0b1110100010001110, 0b1110100011101000, 0b1110100011101110,
        0b1110111010001000, 0b1110111010001110, 0b1110111011101000, 0b1110111011101110,
};

typedef struct
{
  uint8_t G; // G,R,B order is determined by WS2812B
  uint8_t R;
  uint8_t B;
} Pixel_t;

template <uint16_t T>
class WS2812
{
public:
  WS2812()
  {
    initialize();
  }
  ~WS2812() {}

  void setR(uint16_t i, uint8_t v)
  {
    if (isPixel(i))
    {
      pixels[i].R = v;
    }
  }
  void setG(uint16_t i, uint8_t v)
  {
    if (isPixel(i))
    {
      pixels[i].G = v;
    }
  }
  void setB(uint16_t i, uint8_t v)
  {
    if (isPixel(i))
    {
      pixels[i].B = v;
    }
  }
  void setRGB(uint16_t i, uint8_t r, uint8_t g, uint8_t b)
  {
    if (isPixel(i))
    {
      setR(i, r);
      setG(i, g);
      setB(i, b);
    }
  }

  void show(void) // display the pixels
  {
    uint8_t *buffer;
    uint8_t pixelbyte;
    uint8_t gammabyte;
    uint16_t i, b;
    uint16_t *i2s_ptr[WS2812_DITHER_NUM];

    buffer = (uint8_t *)pixels;

    // set-up pointers into the i2s-pixel-buffers
    for (i = 0; i < WS2812_DITHER_NUM; i++)
    {
      i2s_ptr[i] = (uint16_t *)i2s_pixels_buffer[i];
    }

    // for every pixel in the input-array
    // - get the pixel value (either R,G, or B)
    // - for every dithered buffer
    //   get the gamma-corrected output value
    // - and transform into i2s nibble

    for (b = 0; b < NUM_RGB_BYTES; b++)
    {
      pixelbyte = *buffer++;

      for (i = 0; i < WS2812_DITHER_NUM; i++)
      {
        gammabyte = gamma_dither[i][pixelbyte];
        *(i2s_ptr[i]++) = bitpatterns[(gammabyte & 0x0f)];
        *(i2s_ptr[i]++) = bitpatterns[(gammabyte >> 4) & 0x0f];
      }
    }
  }

private:
  uint16_t num_leds = T;
  Pixel_t pixels[T];
  uint32_t *i2s_pixels_buffer[WS2812_DITHER_NUM];
  uint32_t i2s_zeros_buffer[NUM_I2S_ZERO_WORDS];
  sdio_queue_t i2s_zeros_queue[WS2812_DITHER_NUM];
  sdio_queue_t i2s_pixels_queue[WS2812_DITHER_NUM];

  Pixel_t *getPixel(uint16_t i)
  {
    if (isPixel(i))
    {
      return &pixels[i];
    }
    return 0;
  }

  bool isPixel(uint16_t i)
  {
    return (i < num_leds);
  }

  void initialize(void) // Init led-string / memory buffers etc.
  {
    uint8_t i;
    uint16_t j;

    num_leds = T;

    // clear zero buffer
    for (j = 0; j < NUM_I2S_ZERO_WORDS; j++)
    {
      i2s_zeros_buffer[j] = 0;
    }

    // do memory allocation for i2s buffer(s)
    for (i = 0; i < WS2812_DITHER_NUM; i++)
    {
      i2s_pixels_buffer[i] = new uint32_t[NUM_I2S_PIXEL_WORDS];

      // TODO : handle memory allocation error better
      if (i2s_pixels_buffer[i] == 0)
      {
        Serial.println("WS2812_I2S : ERROR ALLOCATING MEMORY");
        return;
      }

      for (j = 0; j < NUM_I2S_PIXEL_WORDS; j++)
      {
        i2s_pixels_buffer[i][j] = 0;
      }
    }

    // set-up DMA descriptors / 1 pair per dither-factor
    for (i = 0; i < WS2812_DITHER_NUM; i++)
    {
      i2s_pixels_queue[i].owner = 1;
      i2s_pixels_queue[i].eof = 1;
      i2s_pixels_queue[i].sub_sof = 0;
      i2s_pixels_queue[i].datalen = NUM_I2S_PIXEL_BYTES;   // Size in bytes
      i2s_pixels_queue[i].blocksize = NUM_I2S_PIXEL_BYTES; // Size in bytes
      i2s_pixels_queue[i].buf_ptr = (uint32_t)i2s_pixels_buffer[i];
      i2s_pixels_queue[i].unused = 0;
      i2s_pixels_queue[i].next_link_ptr = (uint32_t)&i2s_zeros_queue[i]; // always link to zeros-buffer

      i2s_zeros_queue[i].owner = 1;
      i2s_zeros_queue[i].eof = 1;
      i2s_zeros_queue[i].sub_sof = 0;
      i2s_zeros_queue[i].datalen = NUM_I2S_ZERO_BYTES;   // Size in bytes)
      i2s_zeros_queue[i].blocksize = NUM_I2S_ZERO_BYTES; // Size in bytes
      i2s_zeros_queue[i].buf_ptr = (uint32_t)i2s_zeros_buffer;
      i2s_zeros_queue[i].unused = 0;

      if (i == (WS2812_DITHER_NUM - 1)) // last DMA descriptor in linked list ?
      {
        // yes : link to first DMA descriptor
        i2s_zeros_queue[i].next_link_ptr = (uint32_t)&i2s_pixels_queue[0];
      }
      else
      {
        // no : link to next DMA descriptor
        i2s_zeros_queue[i].next_link_ptr = (uint32_t)&i2s_pixels_queue[i + 1];
      }
    }

    // call C based helper function
    // I did not really succeed in putting the code from the helper
    // funtion directly into this constructor...has something to do
    // with  C vs. C++
    // the i2c_writeReg_Mask macro (+ others) failed.. see ws2812_defs.h
    // may be I solve this later

    // parameter = first entry in DMA descriptor list, i.e the descriptor list
    ws2812_dma(i2s_pixels_queue);
  }
};

#endif

// end of file
