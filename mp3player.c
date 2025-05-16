/*
 *
 * MP3 Player with Ultibo API and libmad
 *
 *
 * Based on the 20_pwm_sound example, https://github.com/ultibohub/API/tree/master/samples/20_pwm_sound
 *
 *
 *
 */

#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <stdbool.h>

#include "ultibo/platform.h"
#include "ultibo/threads.h"
#include "ultibo/console.h"
#include "ultibo/pwm.h"
#include "ultibo/filesystem.h"
#include "libmad/mad.h"

#include "pwmsound.h"

struct mad_stream mad_stream;
struct mad_frame mad_frame;
struct mad_synth mad_synth;

static char bufout[80 * 1024 * 1024];
uint32_t channel_count, bit_count;
u_int32_t samplerate;
u_int32_t nmp3i = 0;

/* Rounds MAD's high-resolution samples down to 16 bits */
int16_t scale(mad_fixed_t sample)
{
  /* round */
  sample += (1L << (MAD_F_FRACBITS - 16));
  /* clip */
  if (sample >= MAD_F_ONE)
    sample = MAD_F_ONE - 1;
  else if (sample < -MAD_F_ONE)
    sample = -MAD_F_ONE;
  /* quantize */
  sample >>= (MAD_F_FRACBITS + 1 - 16);
  return (int16_t)sample;
}

static uint32_t pwmsound_clock_start(PWM_DEVICE *pwm, uint32_t frequency)
{
  uint32_t res = ERROR_INVALID_PARAMETER;
  volatile uint32_t *clock_pwmdiv = (uint32_t *)(BCM283X_CM_REGS_BASE + BCM283X_CM_PWMDIV);
  volatile uint32_t *clock_pwmctl = (uint32_t *)(BCM283X_CM_REGS_BASE + BCM283X_CM_PWMCTL);

  /* Check PWM */
  if (!pwm)
    return res;

  /* Check Frequency */
  if (frequency == 0)
    return res;

  /* Check Enabled */
  if (!bcm27xx_pwm_clock_enabled(pwm))
  {
    /* Get Divisors */
    uint32_t divisori = PWMSOUND_PWM_PLLD_CLOCK / frequency;
    uint32_t divisorr = PWMSOUND_PWM_PLLD_CLOCK % frequency;
    uint32_t divisorf = (divisorr * 4096) / PWMSOUND_PWM_PLLD_CLOCK;

    if (divisori > 4095)
      divisori = 4095;

    /* Memory Barrier */
    data_memory_barrier();

    /* Set Dividers */
    *clock_pwmdiv = BCM283X_CM_PASSWORD | (divisori << 12) | divisorf;
    /* Delay */
    microsecond_delay(10);

    /* Set Source */
    *clock_pwmctl = BCM283X_CM_PASSWORD | BCM283X_CM_CTL_SRC_PLLD;
    /* Delay */
    microsecond_delay(10);

    /* Start Clock */
    *clock_pwmctl = BCM283X_CM_PASSWORD | *clock_pwmctl | BCM283X_CM_CTL_ENAB;
    /* Delay */
    microsecond_delay(10);

    /* Memory Barrier */
    data_memory_barrier();
  }

  /* Return Success */
  res = ERROR_SUCCESS;

  return res;
}

static uint32_t pwmsound_start(PWM_DEVICE *pwm)
{
  uint32_t res = ERROR_INVALID_PARAMETER;
  BCM27XX_PWM_DEVICE *bcm27xx_pwm = (BCM27XX_PWM_DEVICE *)pwm;
  volatile BCM283X_PWM_REGISTERS *bcm283x_regs = (BCM283X_PWM_REGISTERS *)bcm27xx_pwm->address;

  /* Check PWM */
  if (!pwm)
    return res;

  /* Check Settings */
  if (pwm->range == 0)
    return res;

  if (pwm->frequency == 0)
    return res;

  res = ERROR_OPERATION_FAILED;

  /* Check GPIO */
  if (pwm->gpio == GPIO_PIN_UNKNOWN)
  {
    /* Check Channel */
    switch (bcm27xx_pwm->channel)
    {
    case 0:
      /* Set GPIO 18 */
      if (pwm_device_set_gpio(pwm, GPIO_PIN_18) != ERROR_SUCCESS)
        return res;

      break;
    case 1:
      /* Set GPIO 19 */
      if (pwm_device_set_gpio(pwm, GPIO_PIN_19) != ERROR_SUCCESS)
        return res;

      break;
    default:
      return res;
    }
  }

  /* Start Clock */
  if (pwmsound_clock_start(pwm, pwm->frequency) != ERROR_SUCCESS)
    return res;

  /* Memory Barrier */
  data_memory_barrier();

  /* Check Channel */
  switch (bcm27xx_pwm->channel)
  {
  case 0:
    /* PWM0 (PWM Channel 1) */
    /* Enable PWEN, USEF and CLRF */
    bcm283x_regs->CTL = bcm283x_regs->CTL | BCM283X_PWM_CTL_PWEN1 | BCM283X_PWM_CTL_USEF1 | BCM283X_PWM_CTL_CLRF1;

    break;
  case 1:
    /* PWM1 (PWM Channel 2) */
    /* Enable PWEN, USEF and CLRF */
    bcm283x_regs->CTL = bcm283x_regs->CTL | BCM283X_PWM_CTL_PWEN2 | BCM283X_PWM_CTL_USEF2 | BCM283X_PWM_CTL_CLRF1;

    break;
  default:
    return res;
  }

  /* Clear Status */
  bcm283x_regs->STA = (uint32_t)-1;

  /* Memory Barrier */
  data_memory_barrier();

  /* Return Success */
  res = ERROR_SUCCESS;

  return res;
}

static uint32_t pwmsound_set_frequency(PWM_DEVICE *pwm, uint32_t frequency)
{
  uint32_t res = ERROR_INVALID_PARAMETER;
  BCM27XX_PWM_DEVICE *bcm27xx_pwm = (BCM27XX_PWM_DEVICE *)pwm;

  /* Check PWM */
  if (!pwm)
    return res;

  /* Check Frequency */
  if (frequency == 0)
    return res;

  res = ERROR_OPERATION_FAILED;

  /* Check Pair */
  if (bcm27xx_pwm->pair)
  {
    /* Check Enabled */
    if (bcm27xx_pwm->pair->pwm.pwmstate == PWM_STATE_ENABLED)
      return res;
  }

  /* Stop Clock */
  if (bcm27xx_pwm_clock_stop(pwm) != ERROR_SUCCESS)
    return res;

  /* Check Enabled */
  if (pwm->pwmstate == PWM_STATE_ENABLED)
  {
    /* Start Clock */
    if (pwmsound_clock_start(pwm, frequency) != ERROR_SUCCESS)
      return res;
  }

  /* Update Scaler */
  bcm27xx_pwm->scaler = NANOSECONDS_PER_SECOND / frequency;

  /* Update Properties */
  pwm->frequency = frequency;
  pwm->properties.frequency = frequency;

  /* Check Pair */
  if (bcm27xx_pwm->pair)
  {
    /* Update Scaler */
    bcm27xx_pwm->pair->scaler = NANOSECONDS_PER_SECOND / frequency;

    /* Update Properties */
    bcm27xx_pwm->pair->pwm.frequency = frequency;
    bcm27xx_pwm->pair->pwm.properties.frequency = frequency;
  }

  /* Return Success */
  res = ERROR_SUCCESS;

  return res;
}

static uint32_t pwmsound_play_sample(PWM_DEVICE *pwm, void *data, uint32_t size, uint32_t channel_count, uint32_t bit_count)
{
  char value[256];
  uint32_t res = ERROR_INVALID_PARAMETER;
  BCM27XX_PWM_DEVICE *bcm27xx_pwm = (BCM27XX_PWM_DEVICE *)pwm;
  volatile BCM283X_PWM_REGISTERS *bcm283x_regs = (BCM283X_PWM_REGISTERS *)bcm27xx_pwm->address;

  if ((channel_count != 1) && (channel_count != 2))
    return res;

  if ((bit_count != 8) && (bit_count != 16))
    return res;

  /* Check PWM */
  if (!pwm)
    return res;

  /* Check Parameters */
  if (size == 0)
    return res;

  if ((channel_count != 1) && (channel_count != 2))
    return res;

  if ((bit_count != 8) && (bit_count != 16))
    return res;

  sprintf(value, "Playing %u bytes on %u channel(s) at %u bits per channel", (unsigned int)size, (unsigned int)channel_count, (unsigned int)bit_count);
  console_write_ln(value);

  res = ERROR_OPERATION_FAILED;

  /* Calculate Range Bits */
  uint32_t range_bits = 0;
  uint32_t count = 2;
  while (count < 16)
  {
    if (pwm->range < (1 << count))
    {
      range_bits = count - 1;

      break;
    }

    count++;
  }

  sprintf(value, "Range = %u", (unsigned int)pwm->range);
  console_write_ln(value);
  sprintf(value, "Range Bits = %u", (unsigned int)range_bits);
  console_write_ln(value);

  /* Get Sample Count */
  uint32_t samples = 0;
  if (bit_count == 8)
  {
    samples = size;

    if (channel_count == 1)
      samples *= 2;
  }
  else if (bit_count == 16)
  {
    samples = size / 2;

    if (channel_count == 1)
      samples *= 2;
  }
  if (samples == 0)
    return res;

  /* Allocate Output */
  uint32_t *output = dma_allocate_buffer(samples * sizeof(uint32_t));
  if (!output)
    return res;

  sprintf(value, "Total Samples = %u", (unsigned int)samples);
  console_write_ln(value);

  /* Convert Sound */
  uint8_t *buffer = (uint8_t *)data;
  uint32_t value1, value2;
  uint32_t current = 0;
  count = 0;
  while (count < size)
  {
    /* Get channel 1 */
    value1 = buffer[count];
    count++;
    if (bit_count > 8)
    {
      /* Get 16 bit sample */
      value1 |= buffer[count] << 8;
      count++;

      /* Convert to unsigned */
      value1 = (value1 + 0x8000) & 0xffff;
    }

    if (bit_count >= range_bits)
      value1 >>= (bit_count - range_bits);
    else
      value1 <<= (range_bits - bit_count);

    /* Get channel 2 */
    value2 = value1;
    if (channel_count == 2)
    {
      value2 = buffer[count];
      count++;
      if (bit_count > 8)
      {
        /* Get 16 bit sample */
        value2 |= buffer[count] << 8;
        count++;

        /* Convert to unsigned */
        value2 = (value2 + 0x8000) & 0xffff;
      }

      if (bit_count >= range_bits)
        value2 >>= (bit_count - range_bits);
      else
        value2 <<= (range_bits - bit_count);
    }

    /* Store Sample */
    output[current] = value1;
    output[current + 1] = value2;
    current += 2;
  }

  /* Get DMA data */
  DMA_DATA *dma_data = malloc(sizeof(DMA_DATA));
  if (!dma_data)
  {
    /* Free Output */
    dma_release_buffer(output);

    return res;
  }
  memset(dma_data, 0, sizeof(DMA_DATA));

  dma_data->source = output;
  dma_data->dest = bcm27xx_pwm->address + BCM283X_PWM_FIF1;
  dma_data->size = samples * sizeof(uint32_t);
  dma_data->flags = DMA_DATA_FLAG_DEST_NOINCREMENT | DMA_DATA_FLAG_DEST_DREQ | DMA_DATA_FLAG_LITE;
  dma_data->stridelength = 0;
  dma_data->sourcestride = 0;
  dma_data->deststride = 0;
  dma_data->next = NULL;

  /* Enable DMA */
  bcm283x_regs->DMAC = bcm283x_regs->DMAC | BCM283X_PWM_DMAC_ENAB;

  /* Perform DMA transfer */
  dma_transfer(dma_data, DMA_DIR_MEM_TO_DEV, PWMSOUND_DMA_DREQ_ID_PWM);

  /* Free DMA Data */
  free(dma_data);

  /* Free Output */
  dma_release_buffer(output);

  /* Return Success */
  res = ERROR_SUCCESS;

  return res;
}

/* Open the supplied file and read the contents into memory for use by pwmsound_play_sample()
 *
 * Note that we show the use of Ultibo filesytem functions here, you could use standard C file
 * functions such as fopen, fread and fclose instead
 */
static uint32_t read_decode_MP3file(char *filename)
{
  uint32_t res = ERROR_INVALID_PARAMETER;

  /* Check Parameters */
  if (strlen(filename) == 0)
    return res;

  /* Wait for SD Card */
  while (!DirectoryExists("C:\\"))
    thread_sleep(100);

  /* Check File */
  if (!FileExists(filename))
    return res;

  res = ERROR_OPERATION_FAILED;

  /* Open File */
  HANDLE handle = FileOpen(filename, fmOpenRead | fmShareDenyNone);
  if (handle == INVALID_HANDLE_VALUE)
    return res;

  int32_t fnsize = FileSize(handle);
  printf("\nThe file size of %s is: %d bytes\n", filename, (int)fnsize);

  /* Check Size */
  if (fnsize > (100 * 1024 * 1024))
    return res;

  /* Allocate Memory */
  void *buffer = malloc(fnsize);
  if (!buffer)
  {
    /* Close File */
    FileClose(handle);

    return res;
  }

  /* Read File */
  int32_t fnread = FileRead(handle, buffer, fnsize);
  if (fnread == fnsize)
  {
    printf("Read MP3 File OK");

    mad_stream_buffer(&mad_stream, buffer, fnsize);
    mad_stream.error = MAD_ERROR_NONE;

    static char stream[1152 * 4];
    u_int32_t nlocation;
    nmp3i = 0;

    mad_timer_t track_timer;
    mad_timer_reset(&track_timer);

    printf("\n\nDecoding the mp3 file...\n");

    // Decode frame and synthesize loop
    while (1)
    {
      // Frame
      if (mad_frame_decode(&mad_frame, &mad_stream))
      {
        if (mad_stream.error == MAD_ERROR_BUFLEN)
          break;

        if (!MAD_RECOVERABLE(mad_stream.error))
          break;
      }
      mad_timer_add(&track_timer, mad_frame.header.duration);
      mad_synth_frame(&mad_synth, &mad_frame);

      struct mad_pcm *pcm;

      pcm = &mad_synth.pcm;

      unsigned int nchannels, nsamples;
      mad_fixed_t const *left_ch, *right_ch;
      u_int32_t ix;

      nchannels = pcm->channels;
      nsamples = pcm->length;
      left_ch = pcm->samples[0];
      right_ch = pcm->samples[1];

      for (ix = 0; ix < nsamples; ix++)
      {
        signed int sample;

        sample = scale(*left_ch++);
        stream[(4 * ix)] = (sample >> 0) & 0xff;
        stream[(4 * ix) + 1] = (sample >> 8) & 0xff;
        if (nchannels == 2)
          sample = scale(*right_ch++);
        stream[(4 * ix) + 2] = (sample >> 0) & 0xff;
        stream[(4 * ix) + 3] = (sample >> 8) & 0xff;
      }

      nlocation = (1152 * 4 * nmp3i);

      nmp3i++;

      memcpy(&bufout[nlocation], stream, (size_t)1152 * 4);
    }

    channel_count = mad_synth.pcm.channels;
    samplerate = mad_synth.pcm.samplerate;
    bit_count = SOUND_BITS;

    u_int32_t lengthms = mad_timer_count(track_timer, MAD_UNITS_MILLISECONDS);
    int secondsx = (int)(lengthms / 1000) % 60;
    int minutesx = (int)((lengthms / (1000 * 60)) % 60);
    int hoursx = (int)((lengthms / (1000 * 60 * 60)) % 24);

    printf("\nDuration: %02d:%02d:%02d\n", hoursx, minutesx, secondsx);
    printf("Bitrate: %lu bps\n", mad_frame.header.bitrate);
    printf("Number channels: %lu\n", channel_count);
    printf("Sample Rate: %lu hz\n\n", samplerate);

    /* Return Success */
    res = ERROR_SUCCESS;
  }

  /* Free Memory */
  free(buffer);

  /* Close File */
  FileClose(handle);

  return res;
}

static uint32_t pwmsound_play_MP3file(PWM_DEVICE *pwm)
{
  uint32_t res = ERROR_INVALID_PARAMETER;

  /* Check PWM */
  if (!pwm)
    return res;

  res = pwmsound_play_sample(pwm, bufout, 1152 * 4 * nmp3i, channel_count, bit_count);
  return res;
}

int apimain(int argc, char **argv)
{
  char value[256];
  WINDOW_HANDLE handle;

  char pwm1_desc[DEVICE_DESC_LENGTH];
  char pwm2_desc[DEVICE_DESC_LENGTH];

  PWM_DEVICE *pwm1_device;
  PWM_DEVICE *pwm2_device;

  /* Create a console window and display a welcome message */
  handle = console_window_create(console_device_get_default(), CONSOLE_POSITION_FULL, TRUE);
  console_window_write_ln(handle, "MP3 Player with Ultibo API and libmad");
  console_window_write_ln(handle, "Make sure you have a the Raspberry Pi audio jack connected to the AUX input of an amplifier, TV or other audio device");

  // Initialize MAD structures
  mad_stream_init(&mad_stream);
  mad_frame_init(&mad_frame);
  mad_synth_init(&mad_synth);

  // Read and decode MP3 File
  uint32_t resMP3 = read_decode_MP3file("test.mp3");

  /* First locate the PWM devices
   *
   * The Raspberry Pi A/B/2B/3+/3B have two PWM channels which will normally
   * end up with the names PWM0 and PWM1 when the driver is included in an
   * application.
   *
   * The Raspberry Pi 4B/400 have four PWM channels which normally end up with
   * the names PWM2 and PWM3.
   *
   * For the purpose of this example we'll refer to them just as device 1 and
   * device 2 instead of their actual name.
   *
   * As with the PWM control example we use pwm_device_find_by_description()
   * to locate the devices and use pwm_get_description() to determine the
   * correct value to pass regardless of the board or configuration.
   *
   * We've also defined all the details that differ between models in a header
   * file named pwmsound.h so we can just refer to those definitions instead
   * of including a lot of ifdef statements in the source code.
   */

  /* Get the PWM device descriptions */
  pwm_get_description(PWMSOUND_PWM1_ID, PWMSOUND_PWM1_CH, pwm1_desc, sizeof(pwm1_desc));
  pwm_get_description(PWMSOUND_PWM2_ID, PWMSOUND_PWM2_CH, pwm2_desc, sizeof(pwm2_desc));

  /* Find the PWM devices */
  pwm1_device = pwm_device_find_by_description(pwm1_desc);
  pwm2_device = pwm_device_find_by_description(pwm2_desc);

  if (pwm1_device && pwm2_device && (resMP3 == ERROR_SUCCESS))
  {
    /* Modify PWM device functions.
     *
     * This allows us to change the behaviour of the PWM driver so we can
     * use a different clock source and enable the FIFO for audio output.
     *
     * Notice how we use the same function for both devices, the PWM device
     * instance is passed to the function so it knows which device to use
     */
    pwm1_device->devicestart = pwmsound_start;
    pwm1_device->devicesetfrequency = pwmsound_set_frequency;
    pwm2_device->devicestart = pwmsound_start;
    pwm2_device->devicesetfrequency = pwmsound_set_frequency;

    /* Setup PWM device 1 */
    /* Set the GPIO */
    pwm_device_set_gpio(pwm1_device, PWMSOUND_PWM1_GPIO);
    /* Set the range */
    pwm_device_set_range(pwm1_device, (CLOCK_RATE + (samplerate / 2)) / samplerate);
    /* And the mode to PWM_MODE_BALANCED */
    pwm_device_set_mode(pwm1_device, PWM_MODE_BALANCED);
    /* Finally set the frequency */
    pwm_device_set_frequency(pwm1_device, CLOCK_RATE);

    /* Setup PWM device 2 */
    /* Use exactly the same settings as PWM1 except the GPIO is different */
    pwm_device_set_gpio(pwm2_device, PWMSOUND_PWM2_GPIO);
    pwm_device_set_range(pwm2_device, (CLOCK_RATE + (samplerate / 2)) / samplerate);
    pwm_device_set_mode(pwm2_device, PWM_MODE_BALANCED);
    pwm_device_set_frequency(pwm2_device, CLOCK_RATE);

    sprintf(value, "Range = %u", (unsigned int)pwm1_device->range);
    console_window_write_ln(handle, value);

    /* Start the PWM devices */
    if ((pwm_device_start(pwm1_device) == ERROR_SUCCESS) && (pwm_device_start(pwm2_device) == ERROR_SUCCESS))
    {
      /* Play MP3 sound file. */

      if (pwmsound_play_MP3file(pwm1_device) != ERROR_SUCCESS)
      {
        console_window_write_ln(handle, "Error: Failed to play MP3 sound file");
      }
      else
      {
        console_window_write_ln(handle, "Finished playing MP3 sound file");
      }

      /* Stop the PWM devices */
      pwm_device_stop(pwm1_device);
      pwm_device_stop(pwm2_device);

      /* Release MAD structures */
      mad_synth_finish(&mad_synth);
      mad_frame_finish(&mad_frame);
      mad_stream_finish(&mad_stream);
    }
    else
    {
      console_window_write_ln(handle, "Error: Failed to start PWM devices");
    }
  }
  else
  {
    console_window_write_ln(handle, "Error: Failed to locate PWM devices");
  }

  /* Turn on the LED to indicate completion */
  activity_led_enable();
  activity_led_on();

  /* Halt the thread if we return */
  thread_halt(0);

  return 0;
}
