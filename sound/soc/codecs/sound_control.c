/*
 * Author: Andrei F. 19.03.2013:
 * 	Implementation fork: Code refactoring and sysfs rewrite.
 *
 * Author: andip71, 26.02.2013
 *
 * Version 1.6.0
 *
 * credits: Supercurio for ideas and partially code from his Voodoo
 * 	    sound implementation,
 *          Yank555 for great support on problem analysis,
 *          Gokhanmoral for further modifications to the original code
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#include <sound/soc.h>
#include <sound/core.h>
#include <sound/jack.h>

#include <linux/mfd/wm8994/core.h>
#include <linux/mfd/wm8994/registers.h>
#include <linux/mfd/wm8994/pdata.h>
#include <linux/mfd/wm8994/gpio.h>

#include <linux/sysfs.h>
#include <linux/sysfs_helpers.h>
#include <linux/miscdevice.h>
#include <linux/switch.h>

#include "wm8994.h"

#include "sound_control.h"

/*****************************************/
// Static variables
/*****************************************/

static int aif_drc_values[3][4] = {
	/* registers WM8994_AIF1_DRC1_1 to _4 */
	{ 152	, 2116	, 232	, 528	},  /* AIF1_DRC1_DEFAULT */
	{ 132	, 647	, 232	, 528	},  /* AIF1_DRC1_PREVENT */
	{ 152	, 2116	, 16	, 235	}   /* AIF1_DRC1_STUNING */
};

static unsigned int eq_bands_original[EQ_TYPE_MAX][5][4] = {
	{	/*   A       B       C      PG   //    Band  */
		{ 0x0FE3, 0x0403, 0	, 0x0074 }, /*   1   */
		{ 0x1F03, 0xF0F9, 0x040A, 0x03DA }, /*   2   */
		{ 0x1ED2, 0xF11A, 0x040A, 0x045D }, /*   3   */ /* Speaker */
		{ 0x0E76, 0xFCE4, 0x040A, 0x330D }, /*   4   */
		{ 0xFC8F, 0x0400, 0	, 0x323C }, /*   5   */
	},
	{
		{ 0x0FCA, 0x0400, 0	, 0x00D8 }, /*   1   */
		{ 0x1EB5, 0xF145, 0x0B75, 0x01C5 }, /*   2   */
		{ 0x1C58, 0xF373, 0x0A54, 0x0558 }, /*   3   */ /* Headphone */
		{ 0x168E, 0xF829, 0x07AD, 0x1103 }, /*   4   */
		{ 0x0564, 0x0559, 0	, 0x4000 }, /*   5   */
	}
};

// pointer to codec structure
static struct snd_soc_codec *codec;
static struct wm8994_priv *wm8994;
extern struct switch_dev android_switch;

// internal sound control variables
static int sound_control;		// sound control master switch
static int debug_level;			// debug level for logging into kernel log

static int headphone_l, headphone_r;	// headphone volume left/right

static int speaker_l, speaker_r;	// speaker volume left/right

static int eq;   			// activates headphone eq
static int eq_speaker;  		// activates speaker eq
static int speaker_boost_level = SPEAKER_BOOST_TUNED;	// boost level for speakers

// gain information for equalizer
static int eq_gains[EQ_TYPE_MAX][5] = { { EQ_GAIN_DEFAULT } };	

// frequency setup for equalizer
static unsigned int eq_bands[EQ_TYPE_MAX][5][4] = { { { 0 } } };

static int dac_direct;			// activate dac_direct for headphone eq
static int dac_oversampling;		// activate 128bit oversampling for headphone eq
static int fll_tuning;			// activate fll tuning to avoid jitter
static int stereo_expansion_gain;	// activate stereo expansion effect if greater than zero
static int mono_downmix;		// activate mono downmix
static int privacy_mode;		// activate privacy mode

static int mic_level_general;		// microphone sensivity for general recording purposes
static int mic_level_call;		// microphone sensivity for call only

static unsigned int debug_register;	// current register to show in debug register interface

// internal state variables
static bool is_call;			// is currently a call active?
static bool is_headphone;		// is headphone connected?
static bool is_socket;			// is something connected to the headphone socket?
static bool is_fmradio;			// is stock fm radio app active?
static bool is_eq;			// is an equalizer (headphone or speaker tuning) active?
static bool is_mic_controlled;		// is microphone sensivity controlled by Audio or not?
static bool is_mono_downmix;		// is mono downmix active?

static int regdump_bank;		// current bank configured for register dump
static unsigned int regcache[REGDUMP_BANKS * REGDUMP_REGISTERS + 1];	// register cache to highlight changes in dump

static int mic_level;			// internal mic level


/*****************************************/
// Internal function declarations
/*****************************************/

static unsigned int wm8994_read(struct snd_soc_codec *codec, unsigned int reg);
static int wm8994_write(struct snd_soc_codec *codec, unsigned int reg, unsigned int value);

static bool debug(int level);
static bool check_for_call(bool load_register, unsigned int val);
static bool check_for_socket(unsigned int val);
static bool check_for_headphone(void);
static bool check_for_fmradio(void);
static void handler_headphone_detection(void);

static void set_headphone(void);
static unsigned int get_headphone_l(unsigned int val);
static unsigned int get_headphone_r(unsigned int val);

static void set_speaker(void);
static unsigned int get_speaker_channel_volume(int channel, unsigned int val);

static void set_eq(void);
static void set_eq_gains(void);
static void set_eq_bands(void);
static void set_eq_satprevention(void);
static unsigned int get_eq_satprevention(int reg_index, unsigned int val);
static void set_speaker_boost(void);

static void set_dac_direct(void);
static unsigned int get_dac_direct_l(unsigned int val);
static unsigned int get_dac_direct_r(unsigned int val);

static void set_dac_oversampling(void);
static void set_fll_tuning(void);
static void set_stereo_expansion(void);
static void set_mono_downmix(void);
static unsigned int get_mono_downmix(unsigned int val);

static void set_mic_level(void);
static unsigned int get_mic_level(int reg_index, unsigned int val);

static void reset_sound_control(void);

/*****************************************/
// Sound hook functions for
// original wm8994 alsa driver
/*****************************************/

void sound_control_hook_wm8994_pcm_probe(struct snd_soc_codec *codec_pointer)
{
	// store a copy of the pointer to the codec, we need
	// that for internal calls to the audio hub
	codec = codec_pointer;

	// store pointer to codecs driver data
	wm8994 = snd_soc_codec_get_drvdata(codec);

	// Print debug info
	printk("Audio: codec pointer received\n");

	// Initialize sound control master switch finally
	sound_control = SOUND_CONTROL_DEFAULT;

	// If sound control is enabled during driver start, reset to default configuration
	if (sound_control) {
		reset_sound_control();
		printk("Audio: sound control enabled during startup\n");
	}
}


unsigned int sound_control_hook_wm8994_write(unsigned int reg, unsigned int val)
{
	unsigned int newval;

	/* Return original value if sound control is disabled */
	if (!sound_control)
		return val;

	/* If the write request of the original driver is for specific registers,
	 * change value to sound control values accordingly as new return value */
	newval = val;

	switch (reg) {
		// call detection
		case WM8994_AIF2_CONTROL_2:
			if (is_call != check_for_call(false, val)) {
				is_call = !is_call;

				if (debug(DEBUG_NORMAL))
					printk("Audio: Call detection new status %d\n", is_call);

				// switch equalizer (and all follow-up functionalities like gains, bands, satprevention etc.)
				set_eq();

				// switch mic level and mono downmix
				set_mic_level();
				set_mono_downmix();
			}

			break;

		// socket connection/disconnection detection (incl. headphone un-plug)
		// (see headphone detection below for plug-in)
		case WM1811_JACKDET_CTRL:
			if (check_for_socket(val)) {
				is_socket = true;

				if (debug(DEBUG_NORMAL))
					printk("Audio: Socket plugged-in\n");
			} else {
				is_socket = false;
				is_headphone = false;

				if (debug(DEBUG_NORMAL))
					printk("Audio: Socket un-plugged\n");

				// Handler: switch equalizer (and all connected functions),
				// mono downmix and set speaker volume (for privacy mode)
				set_eq();
				set_mono_downmix();
				set_speaker();
			}
			break;

		// left headphone volume
		case WM8994_LEFT_OUTPUT_VOLUME:
			newval = get_headphone_l(val);
			break;

		// right headphone volume
		case WM8994_RIGHT_OUTPUT_VOLUME:
			newval = get_headphone_r(val);
			break;

		case WM8994_SPEAKER_VOLUME_LEFT:
			newval = get_speaker_channel_volume(speaker_l, val);
			break;

		case WM8994_SPEAKER_VOLUME_RIGHT:
			newval = get_speaker_channel_volume(speaker_r, val);
			break;

		// dac_direct left channel
		case WM8994_OUTPUT_MIXER_1:
			newval = get_dac_direct_l(val);
			break;

		// dac_direct right channel
		case WM8994_OUTPUT_MIXER_2:
			newval = get_dac_direct_r(val);
			break;

		// mono downmix
		case WM8994_AIF1_DAC1_FILTERS_1:
			newval = get_mono_downmix(val);
			break;

		// EQ saturation prevention: dynamic range control 1_1 to 1_4
		case WM8994_AIF1_DRC1_1:
		case WM8994_AIF1_DRC1_2:
		case WM8994_AIF1_DRC1_3:
		case WM8994_AIF1_DRC1_4:
			newval = get_eq_satprevention(3 - (reg - WM8994_AIF1_DRC1_4), val);
			break;

		// Microphone: left input level
		case WM8994_LEFT_LINE_INPUT_1_2_VOLUME:
			newval = get_mic_level(1, val);
			break;

		// Microphone: right input level
		case WM8994_RIGHT_LINE_INPUT_1_2_VOLUME:
			newval = get_mic_level(2, val);
			break;
	}

	// Headphone plug-in detection
	// ( for un-plug detection see above, this is covered by checking a register)
	if (is_socket && !is_headphone)
		handler_headphone_detection();

	// FM radio detection
	// Important note: We need to absolutely make sure we do not do this detection if one of the
	// two output mixers are called in this hook (as they can potentially be modified again in the
	// set_dac_direct call). Otherwise this adds strange value overwriting effects.
	if ( is_fmradio != check_for_fmradio() &&
		(reg != WM8994_OUTPUT_MIXER_1) && 
		(reg != WM8994_OUTPUT_MIXER_2) )
	{
		is_fmradio = !is_fmradio;

		if (debug(DEBUG_NORMAL))
			printk("Audio: FM radio detection new status %d\n", is_fmradio);

		set_dac_direct();
	}

	// print debug info
	if (debug(DEBUG_VERBOSE))
		printk("Audio: write hook %d -> %d (Orig:%d), c:%d, h:%d, r:%d\n",
				reg, newval, val, is_call, is_headphone, is_fmradio);

	return newval;
}


/*****************************************/
// Internal functions copied over from
// original wm8994 alsa driver,
// enriched by some debug prints
/*****************************************/

static int wm8994_readable(struct snd_soc_codec *codec, unsigned int reg)
{
	//struct wm8994_priv *wm8994 = snd_soc_codec_get_drvdata(codec);
	struct wm8994 *control = codec->control_data;

	switch (reg) {
	case WM8994_GPIO_1:
	case WM8994_GPIO_2:
	case WM8994_GPIO_3:
	case WM8994_GPIO_4:
	case WM8994_GPIO_5:
	case WM8994_GPIO_6:
	case WM8994_GPIO_7:
	case WM8994_GPIO_8:
	case WM8994_GPIO_9:
	case WM8994_GPIO_10:
	case WM8994_GPIO_11:
	case WM8994_INTERRUPT_STATUS_1:
	case WM8994_INTERRUPT_STATUS_2:
	case WM8994_INTERRUPT_STATUS_1_MASK:
	case WM8994_INTERRUPT_STATUS_2_MASK:
	case WM8994_INTERRUPT_RAW_STATUS_2:
		return 1;

	case WM8958_DSP2_PROGRAM:
	case WM8958_DSP2_CONFIG:
	case WM8958_DSP2_EXECCONTROL:
		if (control->type == WM8958)
			return 1;
		else
			return 0;

	default:
		break;
	}

	if (reg >= WM8994_CACHE_SIZE)
		return 0;
	return wm8994_access_masks[reg].readable != 0;
}


static int wm8994_volatile(struct snd_soc_codec *codec, unsigned int reg)
{
	if (reg >= WM8994_CACHE_SIZE)
		return 1;

	switch (reg) {
	case WM8994_SOFTWARE_RESET:
	case WM8994_CHIP_REVISION:
	case WM8994_DC_SERVO_1:
	case WM8994_DC_SERVO_READBACK:
	case WM8994_RATE_STATUS:
	case WM8994_LDO_1:
	case WM8994_LDO_2:
	case WM8958_DSP2_EXECCONTROL:
	case WM8958_MIC_DETECT_3:
	case WM8994_DC_SERVO_4E:
		return 1;
	default:
		return 0;
	}
}


static int wm8994_write(struct snd_soc_codec *codec, unsigned int reg,
	unsigned int value)
{
	int ret;

	BUG_ON(reg > WM8994_MAX_REGISTER);

	if (!wm8994_volatile(codec, reg)) {
		ret = snd_soc_cache_write(codec, reg, value);
		if (ret != 0)
			dev_err(codec->dev, "Cache write to %x failed: %d",
				reg, ret);
	}

	// print debug info
	if (debug(DEBUG_VERBOSE))
		printk("Audio: write register %d -> %d\n", reg, value);

	return wm8994_reg_write(codec->control_data, reg, value);
}


static unsigned int wm8994_read(struct snd_soc_codec *codec,
				unsigned int reg)
{
	unsigned int val;
	int ret;

	BUG_ON(reg > WM8994_MAX_REGISTER);

	if (!wm8994_volatile(codec, reg) && wm8994_readable(codec, reg) &&
	    reg < codec->driver->reg_cache_size) {
		ret = snd_soc_cache_read(codec, reg, &val);
		if (ret >= 0)
		{
			// print debug info
			if (debug(DEBUG_VERBOSE))
				printk("Audio: read register from cache %d -> %d\n", reg, val);

			return val;
		}
		else
			dev_err(codec->dev, "Cache read from %x failed: %d",
				reg, ret);
	}

	val = wm8994_reg_read(codec->control_data, reg);

	// print debug info
	if (debug(DEBUG_VERBOSE))
		printk("Audio: read register %d -> %d\n", reg, val);

	return val;
}


/*****************************************/
// Internal helper functions
/*****************************************/

static bool check_for_call(bool load_register, unsigned int val)
{
#ifdef CONFIG_SND_SOC_SAMSUNG_MIDAS_WM1811
	// if a check outside the write hook should be performed, the current register
	// value needs to be loaded first
	if (load_register)
		val = wm8994_read(codec, WM8994_AIF2_CONTROL_2);

	// check via register WM8994_AIF2DACR if currently call active
	if (!(val & WM8994_AIF2DACR_SRC_MASK))
		return true;
#endif

	return false;
}


static bool check_for_socket(unsigned int val)
{
	// check via register WM1811_JACKDET if something is plugged in currently
	if (val & WM1811_JACKDET_DB_MASK)
		return false;

	return true;
}


static bool check_for_headphone(void)
{
	return (switch_get_state(&android_switch) > 0);
}


static bool check_for_fmradio(void)
{
#ifdef CONFIG_FM_RADIO
	struct snd_soc_dapm_widget *w;

	// loop through widget list to find widget for FM radio and check
	// power state of it
	list_for_each_entry(w, &codec->card->widgets, list) {
		if (w->dapm != &codec->dapm)
			continue;

		switch (w->id) {
			case snd_soc_dapm_line:
				if (w->name) {
					if(strstr(w->name,"FM In") != 0) {
						if((w->power) != 0)
							return true;
						else
							return false;
					}
				}
				break;
			case snd_soc_dapm_mic:
			case snd_soc_dapm_hp:
			case snd_soc_dapm_spk:
			case snd_soc_dapm_micbias:
			case snd_soc_dapm_dac:
			case snd_soc_dapm_adc:
			case snd_soc_dapm_pga:
			case snd_soc_dapm_out_drv:
			case snd_soc_dapm_mixer:
			case snd_soc_dapm_mixer_named_ctl:
			case snd_soc_dapm_supply:
				break;
			default:
				break;
		}
	}
#endif

	return false;
}


static void handler_headphone_detection(void)
{
	if (check_for_headphone()) {
		is_headphone = true;

		if (debug(DEBUG_NORMAL))
			printk("Audio: Headphone or headset found\n");

		// Handler: switch equalizer and mono downmix, set speaker volume (for privacy mode)
		set_eq();
		set_mono_downmix();
		set_speaker();
	}
}


static bool debug(int level)
{
	// determine whether a debug information should be printed 
	// according to currently configured debug level, or not
	if (level <= debug_level)
		return true;

	return false;
}


/*****************************************/
// Internal set/get/restore functions
/*****************************************/

// Headphone volume

static void set_headphone(void)
{
	unsigned int val;

	// get current register value, unmask volume bits, merge with sound control volume and write back
	val = wm8994_read(codec, WM8994_LEFT_OUTPUT_VOLUME);
	val = (val & ~WM8994_HPOUT1L_VOL_MASK) | headphone_l;
        wm8994_write(codec, WM8994_LEFT_OUTPUT_VOLUME, val);

	val = wm8994_read(codec, WM8994_RIGHT_OUTPUT_VOLUME);
	val = (val & ~WM8994_HPOUT1R_VOL_MASK) | headphone_r;
        wm8994_write(codec, WM8994_RIGHT_OUTPUT_VOLUME, val | WM8994_HPOUT1_VU);

	// print debug info
	if (debug(DEBUG_NORMAL))
		printk("Audio: %s %d %d\n", __func__, headphone_l, headphone_r);

}


static unsigned int get_headphone_l(unsigned int val)
{
	// return register value for left headphone volume back
        return (val & ~WM8994_HPOUT1L_VOL_MASK) | headphone_l;
}


static unsigned int get_headphone_r(unsigned int val)
{
	// return register value for right headphone volume back
        return (val & ~WM8994_HPOUT1R_VOL_MASK) | headphone_r;
}


// Speaker volume

static void set_speaker(void)
{
	unsigned int val;

	// read current register values, get corrected value and write back to audio hub
	val = wm8994_read(codec, WM8994_SPEAKER_VOLUME_LEFT);
	val = get_speaker_channel_volume(speaker_l, val);
        wm8994_write(codec, WM8994_SPEAKER_VOLUME_LEFT, val);

	val = wm8994_read(codec, WM8994_SPEAKER_VOLUME_RIGHT);
	val = get_speaker_channel_volume(speaker_r, val);
        wm8994_write(codec, WM8994_SPEAKER_VOLUME_RIGHT, val | WM8994_SPKOUT_VU);

	// print debug info
	if (debug(DEBUG_NORMAL)) {
		if (privacy_mode && is_headphone) {
			printk("Audio: %s to mute (privacy mode)\n", __func__);
		} else {
			printk("Audio: %s %d %d\n", __func__, speaker_l, speaker_r);
		}
	}
}


static unsigned int get_speaker_channel_volume(int channel, unsigned int val)
{
	// if privacy mode is on, we set value to zero, otherwise to configured speaker volume
	if (privacy_mode && is_headphone)
		return (val & ~WM8994_SPKOUTL_VOL_MASK);

	return (val & ~WM8994_SPKOUTL_VOL_MASK) | channel;
}

// Equalizer on/off

static void set_eq(void)
{
	unsigned int val;

	// Equalizer will only be switched on in fact if
	// 1. headphone eq is on, there is no call and there is headphone connected -- or --
	// 2. speaker tuning is enabled, there is no call and there is no headphone connected

	// set internal state variables
	is_eq = !is_call && ( (eq & EQ_ENABLED && is_headphone) || 
			      (eq_speaker && !is_headphone) );

	// switch equalizer based on internal status
	val = wm8994_read(codec, WM8994_AIF1_DAC1_EQ_GAINS_1);

	if (is_eq)
		val |= WM8994_AIF1DAC1_EQ_ENA_MASK;
	else
		val &= ~WM8994_AIF1DAC1_EQ_ENA_MASK;

	wm8994_write(codec, WM8994_AIF1_DAC1_EQ_GAINS_1, val);

	if (debug(DEBUG_NORMAL))
		printk("Audio: %s %s\n", __func__, is_eq ? "on":"off");

	// refresh settings for gains, bands, saturation prevention and speaker boost
	set_eq_gains();
	set_eq_bands();
	set_eq_satprevention();
	set_speaker_boost();
}


// Equalizer gains

static void set_eq_gains(void)
{
	unsigned int val, out;
	unsigned int gain1, gain2, gain3, gain4, gain5;

	// determine gain values based on equalizer mode (headphone vs. speaker tuning)
	out = (!is_headphone && eq_speaker) ? EQ_SP : EQ_HP;

	gain1 = eq_gains[out][0];
	gain2 = eq_gains[out][1];
	gain3 = eq_gains[out][2];
	gain4 = eq_gains[out][3];
	gain5 = eq_gains[out][4];

	// print debug info
	if (debug(DEBUG_NORMAL))
		printk("Audio: %s (%s) %d %d %d %d %d\n", __func__,
			out ? "headphone" : "speaker", gain1, gain2, gain3, gain4, gain5);

	// First register
	// read current value from audio hub and mask all bits apart from equalizer enabled bit,
	// add individual gains and write back to audio hub

	val = wm8994_read(codec, WM8994_AIF1_DAC1_EQ_GAINS_1);
	val &= WM8994_AIF1DAC1_EQ_ENA_MASK;

	val |=  ((gain1 + EQ_GAIN_OFFSET) << WM8994_AIF1DAC1_EQ_B1_GAIN_SHIFT) |
		((gain2 + EQ_GAIN_OFFSET) << WM8994_AIF1DAC1_EQ_B2_GAIN_SHIFT) | 
		((gain3 + EQ_GAIN_OFFSET) << WM8994_AIF1DAC1_EQ_B3_GAIN_SHIFT);

	wm8994_write(codec, WM8994_AIF1_DAC1_EQ_GAINS_1, val);

	// second register
	val =   ((gain4 + EQ_GAIN_OFFSET) << WM8994_AIF1DAC1_EQ_B4_GAIN_SHIFT) |
		((gain5 + EQ_GAIN_OFFSET) << WM8994_AIF1DAC1_EQ_B5_GAIN_SHIFT);

	wm8994_write(codec, WM8994_AIF1_DAC1_EQ_GAINS_2, val);
}


// Equalizer bands

static void set_eq_bands()
{
	// Set band frequencies either for headphone eq or for speaker tuning
	int i = WM8994_AIF1_DAC1_EQ_BAND_1_A;
	int j = 0;
	int k = 0;
	int o = (!is_headphone && eq_speaker) ? EQ_SP : EQ_HP;

	while (i <= WM8994_AIF1_DAC1_EQ_BAND_5_PG) {
		if(eq_bands[o][j][k])
			wm8994_write(codec, i++, eq_bands[o][j][k]);

		if(k++ >= 3) {
			k = 0;
			++j;
		} 
	}

	if (debug(DEBUG_NORMAL)) {
		for(i = 0; i < 5; i++) {
			printk("Audio: %s %d (%s) %d %d %d %d\n",
				__func__, i+1, o ? "headphone" : "speaker",
				eq_bands[o][i][0], eq_bands[o][i][1],
				eq_bands[o][i][2], eq_bands[o][i][3]);
		}
	}
}


// EQ saturation prevention

static void set_eq_satprevention(void)
{
	unsigned int val, i;

	/* Read current value for DRC1_i register, modify value and write back to audio hub.
	 * Iterate from WM8994_AIF1_DRC1_1 to WM8994_AIF1_DRC1_4 */

	for(i = 0; i < 5; i++) {
		val = wm8994_read(codec, WM8994_AIF1_DRC1_1 + i);
		val = get_eq_satprevention(i, val);
		wm8994_write(codec, WM8994_AIF1_DRC1_1 + i, val);
	}

	// print debug information
	if (debug(DEBUG_NORMAL)) {
		/* Output current equalizer status and saturation prevention mode */
		if (is_eq) {
			if (is_headphone && (eq & EQ_SATPREVENT)) {
				printk("Audio: %s to on (headphone)\n", __func__);
				return;
			}

			if (!is_headphone && eq_speaker) {
				printk("Audio: %s to on (speaker)\n", __func__);
				return;
			}
		}

		printk("Audio: %s to off\n", __func__);
	}
}


static unsigned int get_eq_satprevention(int reg_index, unsigned int val)
{
	if (is_eq) {
		if (is_headphone && (eq & EQ_SATPREVENT))
			return aif_drc_values[AIF1_DRC1_PREVENT][reg_index];

		if (!is_headphone && eq_speaker)
			return aif_drc_values[AIF1_DRC1_STUNING][reg_index];
	}

	return aif_drc_values[AIF1_DRC1_DEFAULT][reg_index];
}


// Speaker boost (for speaker tuning)

static void set_speaker_boost(void)
{
	unsigned int val, boostval;

	// Speaker boost gets activated only if EQ mode is for speaker tuning
	boostval = (!is_headphone && eq_speaker) ? speaker_boost_level : SPEAKER_BOOST_DEFAULT;

	val = wm8994_read(codec, WM8994_CLASSD);
	val &= ~(WM8994_SPKOUTL_BOOST_MASK | WM8994_SPKOUTR_BOOST_MASK);
	val |= (boostval << WM8994_SPKOUTL_BOOST_SHIFT);
	val |= (boostval << WM8994_SPKOUTR_BOOST_SHIFT);
	wm8994_write(codec, WM8994_CLASSD, val);

	// print debug info
	if (debug(DEBUG_NORMAL))
		printk("Audio: %s %d\n", __func__, boostval);
}


// DAC direct

static void set_dac_direct(void)
{
	unsigned int val;

	// get current values for output mixers 1 and 2 (l + r) from audio hub
	// modify the data accordingly and write back to audio hub
	val = wm8994_read(codec, WM8994_OUTPUT_MIXER_1);
	val = get_dac_direct_l(val);
	wm8994_write(codec, WM8994_OUTPUT_MIXER_1, val);

	val = wm8994_read(codec, WM8994_OUTPUT_MIXER_2);
	val = get_dac_direct_r(val);
	wm8994_write(codec, WM8994_OUTPUT_MIXER_2, val);

	// take value of the right channel as reference, check for the bypass bit
	// and print debug information
	if (debug(DEBUG_NORMAL)) {
		if (val & WM8994_DAC1R_TO_HPOUT1R)
			printk("Audio: set_dac_direct on\n");
		else
			printk("Audio: set_dac_direct off\n");
	}

}

static unsigned int get_dac_direct_l(unsigned int val)
{
	// dac direct is only enabled if fm radio is not active
	if (dac_direct && !is_fmradio) {
		// enable dac_direct: bypass for both channels, mute output mixer
		return((val & ~WM8994_DAC1L_TO_MIXOUTL) | WM8994_DAC1L_TO_HPOUT1L);
	}

	// disable dac_direct: enable bypass for both channels, mute output mixer
	return((val & ~WM8994_DAC1L_TO_HPOUT1L) | WM8994_DAC1L_TO_MIXOUTL);
}

static unsigned int get_dac_direct_r(unsigned int val)
{
	// dac direct is only enabled if fm radio is not active
	if (dac_direct && !is_fmradio) {
		// enable dac_direct: bypass for both channels, mute output mixer
		return((val & ~WM8994_DAC1R_TO_MIXOUTR) | WM8994_DAC1R_TO_HPOUT1R);
	}

	// disable dac_direct: enable bypass for both channels, mute output mixer
	return((val & ~WM8994_DAC1R_TO_HPOUT1R) | WM8994_DAC1R_TO_MIXOUTR);
}


// DAC oversampling

static void set_dac_oversampling()
{
	unsigned int val;

	// read current value of oversampling register
	val = wm8994_read(codec, WM8994_OVERSAMPLING);

	// toggle oversampling bit depending on status + print debug
	if (dac_oversampling)
		val |= WM8994_DAC_OSR128;
	else
		val &= ~WM8994_DAC_OSR128;

	if (debug(DEBUG_NORMAL))
		printk("Audio: %s %s\n", __func__, dac_oversampling ? "on":"off");

	// write value back to audio hub
	wm8994_write(codec, WM8994_OVERSAMPLING, val);
}


// FLL tuning

static void set_fll_tuning(void)
{
	unsigned int val;

	// read current value of FLL control register 4 and mask out loop gain value
	val = wm8994_read(codec, WM8994_FLL1_CONTROL_4);
	val &= ~WM8994_FLL1_LOOP_GAIN_MASK;

	// depending on whether fll tuning is on or off, modify value accordingly
	// and print debug
	val |= (fll_tuning) ? FLL_LOOP_GAIN_TUNED : FLL_LOOP_GAIN_DEFAULT;

	// write value back to audio hub
	wm8994_write(codec, WM8994_FLL1_CONTROL_4, val);

	if (debug(DEBUG_NORMAL))
		printk("Audio: %s %s\n", __func__, dac_oversampling ? "on":"off");
}


// Stereo expansion

static void set_stereo_expansion(void)
{
	unsigned int val;

	// read current value of DAC1 filter register and mask out gain value and enable bit
	val = wm8994_read(codec, WM8994_AIF1_DAC1_FILTERS_2);
	val &= ~(WM8994_AIF1DAC1_3D_GAIN_MASK);
	val &= ~(WM8994_AIF1DAC1_3D_ENA_MASK);

	// depending on whether stereo expansion is 0 (=off) or not, modify values for gain
	// and enabled bit accordingly, also print debug
	if (stereo_expansion_gain != STEREO_EXPANSION_GAIN_OFF) {
		val |= (stereo_expansion_gain << WM8994_AIF1DAC1_3D_GAIN_SHIFT) | WM8994_AIF1DAC1_3D_ENA;

		if (debug(DEBUG_NORMAL))
			printk("Audio: set_stereo_expansion set to %d\n", stereo_expansion_gain);
	} else
		if (debug(DEBUG_NORMAL))
			printk("Audio: set_stereo_expansion off\n");

	// write value back to audio hub
	wm8994_write(codec, WM8994_AIF1_DAC1_FILTERS_2, val);
}


// Mono downmix

static void set_mono_downmix(void)
{
	unsigned int val;

	if (!is_call && is_headphone) {
		if (is_mono_downmix != mono_downmix) {
			is_mono_downmix = mono_downmix;
			
			val = wm8994_read(codec, WM8994_AIF1_DAC1_FILTERS_1);

			if (is_mono_downmix)
				val &= ~WM8994_AIF1DAC1_MONO;
			else
				val |= WM8994_AIF1DAC1_MONO;
			
			wm8994_write(codec, WM8994_AIF1_DAC1_FILTERS_1, val);

			if (debug(DEBUG_NORMAL))
				printk("Audio: %s set to %s\n", __func__, is_mono_downmix ? "on":"off");
		}
	}
}


static unsigned int get_mono_downmix(unsigned int val)
{
	if (!mono_downmix)
		return val;

	if (is_mono_downmix)
		return val | WM8994_AIF1DAC1_MONO;

	return val & ~WM8994_AIF1DAC1_MONO;
}


// MIC level

static void set_mic_level(void)
{
	unsigned int val;

	// if mic is not controlled by Audio, terminate and do nothing
	if (!is_mic_controlled)
		return;

	// check if call is currently active as internal mic sensivity value
	// is dependent on this

	mic_level = (is_call) ? mic_level_call : mic_level_general;

	// set input volume for both input channels
	val = wm8994_read(codec, WM8994_LEFT_LINE_INPUT_1_2_VOLUME);
	wm8994_write(codec, WM8994_LEFT_LINE_INPUT_1_2_VOLUME, get_mic_level(1, 0));

	val = wm8994_read(codec, WM8994_RIGHT_LINE_INPUT_1_2_VOLUME);
	wm8994_write(codec, WM8994_RIGHT_LINE_INPUT_1_2_VOLUME, get_mic_level(2, 0));

	// print debug info
	if (debug(DEBUG_NORMAL))
		printk("Audio: set_mic_level %d\n", mic_level);
}


static unsigned int get_mic_level(int reg_index, unsigned int val)
{

	// check if mic is currently controlled by Audio
	// if not, the value is returned back unchanged to not impact the microphone at all
	if (!is_mic_controlled)
		return val;

	// send changed values back
	switch (reg_index) {
		case 1: //  Register WM8994_LEFT_LINE_INPUT_1_2_VOLUME
		case 2: //  Register WM8994_RIGHT_LINE_INPUT_1_2_VOLUME
			return(mic_level | WM8994_IN1_VU);
	}

	// we should never reach this point ideally, but in error case return original value
	return val;
}


// Initialization functions

static void initialize_global_variables(void)
{
	// set global variables to standard values

	headphone_l = HEADPHONE_DEFAULT;
	headphone_r = HEADPHONE_DEFAULT;

	speaker_l = SPEAKER_DEFAULT;
	speaker_r = SPEAKER_DEFAULT;

	eq_speaker = false;

	memcpy(eq_bands, eq_bands_original, sizeof(eq_bands_original));

	eq = EQ_DEFAULT;

	dac_direct = false;

	dac_oversampling = false;

	fll_tuning = false;

	stereo_expansion_gain = STEREO_EXPANSION_GAIN_OFF;

	mono_downmix = false;

	privacy_mode = false;

	mic_level_general = MICLEVEL_GENERAL;
	mic_level_call = MICLEVEL_CALL;
	mic_level = MICLEVEL_GENERAL;

	debug_register = 0;

	is_call = false;
	is_socket = false;
	is_headphone = false;
	is_fmradio = false;
	is_eq = false;
	is_mic_controlled = false;
	is_mono_downmix = false;

	// print debug info
	if (debug(DEBUG_NORMAL))
		printk("Audio: %s complete\n", __func__);
}


static void reset_sound_control(void)
{
	unsigned int val;

	// print debug info
	if (debug(DEBUG_NORMAL))
		printk("Audio: %s start\n", __func__);

	// load all default values
	initialize_global_variables();

	// set headphone volumes to defaults
	set_headphone();

	// set speaker volumes to defaults
	set_speaker();

	// reset equalizer mode
	// (this also resets gains, bands, saturation prevention and speaker boost)
	set_eq();

	// reset DAC_direct
	set_dac_direct();

	// reset DAC oversampling
	set_dac_oversampling();

	// reset FLL tuning
	set_fll_tuning();

	// reset stereo expansion
	set_stereo_expansion();

	// reset mono downmix
	set_mono_downmix();

	// reset mic level
	set_mic_level();

	// initialize jacket, headphone, call and fm radio status
	val = wm8994_read(codec, WM1811_JACKDET_CTRL);
	is_socket = check_for_socket(val);

	is_call = check_for_call(true, 0);
	handler_headphone_detection();
	is_fmradio = check_for_fmradio();

	// print debug info
	if (debug(DEBUG_NORMAL))
		printk("Audio: %s complete\n", __func__);
}



/*****************************************/
// sysfs interface functions
/*****************************************/

static ssize_t show_sound_property(struct device *dev,
				    struct device_attribute *attr, char *buf);

static ssize_t store_sound_property(struct device *dev,
				     struct device_attribute *attr,
				     const char *buf, size_t count);

static ssize_t debug_info_show(char *buf)
{
	unsigned int val, len = 0;

	// start with version info
	len += sprintf(buf + len, "Audio version: %s\n\n", SOUND_CONTROL_VERSION);

	// read values of some interesting registers and put them into a string
	val = wm8994_read(codec, WM8994_AIF2_CONTROL_2);
	len += sprintf(buf + len, "WM8994_AIF2_CONTROL_2: %d\n", val);

	val = wm8994_read(codec, WM8994_LEFT_OUTPUT_VOLUME);
	len += sprintf(buf + len, "WM8994_LEFT_OUTPUT_VOLUME: %d\n", val);

	val = wm8994_read(codec, WM8994_RIGHT_OUTPUT_VOLUME);
	len += sprintf(buf + len, "WM8994_RIGHT_OUTPUT_VOLUME: %d\n", val);

	val = wm8994_read(codec, WM8994_SPEAKER_VOLUME_LEFT);
	len += sprintf(buf + len, "WM8994_SPEAKER_VOLUME_LEFT: %d\n", val);

	val = wm8994_read(codec, WM8994_SPEAKER_VOLUME_RIGHT);
	len += sprintf(buf + len, "WM8994_SPEAKER_VOLUME_RIGHT: %d\n", val);

	val = wm8994_read(codec, WM8994_CLASSD);
	len += sprintf(buf + len, "WM8994_CLASSD: %d\n", val);

	val = wm8994_read(codec, WM8994_AIF1_DAC1_EQ_GAINS_1);
	len += sprintf(buf + len, "WM8994_AIF1_DAC1_EQ_GAINS_1: %d\n", val);

	val = wm8994_read(codec, WM8994_AIF1_DAC1_EQ_GAINS_2);
	len += sprintf(buf + len, "WM8994_AIF1_DAC1_EQ_GAINS_2: %d\n", val);

	val = wm8994_read(codec, WM8994_AIF1_DRC1_1);
	len += sprintf(buf + len, "WM8994_AIF1_DRC1_1: %d\n", val);

	val = wm8994_read(codec, WM8994_AIF1_DRC1_2);
	len += sprintf(buf + len, "WM8994_AIF1_DRC1_2: %d\n", val);

	val = wm8994_read(codec, WM8994_AIF1_DRC1_3);
	len += sprintf(buf + len, "WM8994_AIF1_DRC1_3: %d\n", val);

	val = wm8994_read(codec, WM8994_AIF1_DRC1_4);
	len += sprintf(buf + len, "WM8994_AIF1_DRC1_4: %d\n", val);

	val = wm8994_read(codec, WM8994_OUTPUT_MIXER_1);
	len += sprintf(buf + len, "WM8994_OUTPUT_MIXER_1: %d\n", val);

	val = wm8994_read(codec, WM8994_OUTPUT_MIXER_2);
	len += sprintf(buf + len, "WM8994_OUTPUT_MIXER_2: %d\n", val);

	val = wm8994_read(codec, WM8994_OVERSAMPLING);
	len += sprintf(buf + len, "WM8994_OVERSAMPLING: %d\n", val);

	val = wm8994_read(codec, WM8994_FLL1_CONTROL_4);
	len += sprintf(buf + len, "WM8994_FLL1_CONTROL_4: %d\n", val);

	val = wm8994_read(codec, WM8994_LEFT_LINE_INPUT_1_2_VOLUME);
	len += sprintf(buf + len, "WM8994_LEFT_LINE_INPUT_1_2_VOLUME: %d\n", val);

	val = wm8994_read(codec, WM8994_RIGHT_LINE_INPUT_1_2_VOLUME);
	len += sprintf(buf + len, "WM8994_RIGHT_LINE_INPUT_1_2_VOLUME: %d\n", val);

	val = wm8994_read(codec, WM8994_INPUT_MIXER_3);
	len += sprintf(buf + len, "WM8994_INPUT_MIXER_3: %d\n", val);

	val = wm8994_read(codec, WM8994_INPUT_MIXER_4);
	len += sprintf(buf + len, "WM8994_INPUT_MIXER_4: %d\n", val);

	val = wm8994_read(codec, WM8994_AIF1_DAC1_FILTERS_1);
	len += sprintf(buf + len, "WM8994_AIF1_DAC1_FILTERS_1: %d\n", val);

	val = wm8994_read(codec, WM8994_AIF1_DAC1_FILTERS_2);
	len += sprintf(buf + len, "WM8994_AIF1_DAC1_FILTERS_2: %d\n", val);

	// add the current states of call, headphone and fmradio
	len += sprintf(buf + len, "is_call:%d is_socket: %d is_headphone:%d is_fmradio:%d\n",
				is_call, is_socket, is_headphone, is_fmradio);

	// add the current states of internal headphone handling and mono downmix
	len += sprintf(buf + len, "is_eq:%d is_mono_downmix: %d\n",
				is_eq, is_mono_downmix);

	// add the current states of internal mic level, gain and control state
	len += sprintf(buf + len, "mic_level: %d is_mic_controlled: %d\n",
				mic_level, is_mic_controlled);

	return len;
}

static ssize_t debug_dump_show(char *buf)
{
	int i, val, len = 0;
	for (i = regdump_bank*REGDUMP_REGISTERS; i <= (regdump_bank+1)*REGDUMP_REGISTERS; i++) {
		val = wm8994_read(codec, i);

		if(regcache[i] != val)
			len += sprintf((buf + len), "%d: %d -> %d\n", i, regcache[i], val);
		else
			len += sprintf((buf + len), "%d: %d\n", i, val);

		regcache[i] = val;
	}
	return len;
}


static ssize_t band_show(int output, int band, char *buf)
{
	int i, len = 0;

	for(i = 0; i < 4; i++)
		len += sprintf(buf + len, "%d ", eq_bands[output][band][i]);

	return len;
}

#define SOUND_ATTR(name_)				\
{							\
	.attr = {					\
		  .name = #name_,			\
		  .mode = S_IRUGO | S_IWUSR | S_IWGRP,	\
		},					\
	.show = show_sound_property,			\
	.store = store_sound_property,			\
}

static struct device_attribute sound_control_attrs[] = {
	SOUND_ATTR(switch_master),
	SOUND_ATTR(switch_eq_headphone),
	SOUND_ATTR(switch_eq_speaker),
	SOUND_ATTR(switch_dac_direct),
	SOUND_ATTR(switch_oversampling),
	SOUND_ATTR(switch_fll_tuning),
	SOUND_ATTR(switch_mono_downmix),
	SOUND_ATTR(switch_privacy_mode),

	SOUND_ATTR(debug_level),
	SOUND_ATTR(debug_info),
	SOUND_ATTR(debug_reg),
	SOUND_ATTR(debug_dump),

	SOUND_ATTR(headphone_left),
	SOUND_ATTR(headphone_right),

	SOUND_ATTR(speaker_left),
	SOUND_ATTR(speaker_right),

	SOUND_ATTR(speaker_boost_level),

	SOUND_ATTR(stereo_expansion),

	SOUND_ATTR(mic_level_general),
	SOUND_ATTR(mic_level_call),

	SOUND_ATTR(eq_hp_gain_1),
	SOUND_ATTR(eq_hp_gain_2),
	SOUND_ATTR(eq_hp_gain_3),
	SOUND_ATTR(eq_hp_gain_4),
	SOUND_ATTR(eq_hp_gain_5),

	SOUND_ATTR(eq_hp_band_1),
	SOUND_ATTR(eq_hp_band_2),
	SOUND_ATTR(eq_hp_band_3),
	SOUND_ATTR(eq_hp_band_4),
	SOUND_ATTR(eq_hp_band_5),

	SOUND_ATTR(eq_sp_gain_1),
	SOUND_ATTR(eq_sp_gain_2),
	SOUND_ATTR(eq_sp_gain_3),
	SOUND_ATTR(eq_sp_gain_4),
	SOUND_ATTR(eq_sp_gain_5),

	SOUND_ATTR(eq_sp_band_1),
	SOUND_ATTR(eq_sp_band_2),
	SOUND_ATTR(eq_sp_band_3),
	SOUND_ATTR(eq_sp_band_4),
	SOUND_ATTR(eq_sp_band_5),
};

enum {
	SWITCH_MASTER = 0,
	SWITCH_EQ_HEADPHONE,
	SWITCH_EQ_SPEAKER,
	SWITCH_DAC_DIRECT,
	SWITCH_OVERSAMPLING,
	SWITCH_FLL_TUNING,
	SWITCH_MONO_DOWNMIX,
	SWITCH_PRIVACY_MODE,
	
	DEBUG_LEVEL, DEBUG_INFO, DEBUG_REG, DEBUG_DUMP,

	HEADPHONE_LEFT, HEADPHONE_RIGHT,

	SPEAKER_LEFT, SPEAKER_RIGHT,

	SPEAKER_BOOST_LEVEL,

	STEREO_EXPANSION,

	MIC_LEVEL_GENERAL, MIC_LEVEL_CALL,

	EQ_HP_GAIN_1, EQ_HP_GAIN_2, EQ_HP_GAIN_3, EQ_HP_GAIN_4, EQ_HP_GAIN_5,
	EQ_HP_BAND_1, EQ_HP_BAND_2, EQ_HP_BAND_3, EQ_HP_BAND_4, EQ_HP_BAND_5,

	EQ_SP_GAIN_1, EQ_SP_GAIN_2, EQ_SP_GAIN_3, EQ_SP_GAIN_4, EQ_SP_GAIN_5,
	EQ_SP_BAND_1, EQ_SP_BAND_2, EQ_SP_BAND_3, EQ_SP_BAND_4, EQ_SP_BAND_5,
};

static ssize_t show_sound_property(struct device *dev,
				    struct device_attribute *attr, char *buf)
{
	const ptrdiff_t offset = attr - sound_control_attrs;
	
	switch (offset) {
		case SWITCH_MASTER:
			return sprintf(buf, "%d", sound_control);
		case SWITCH_EQ_HEADPHONE:
			return sprintf(buf, "%d", eq);
		case SWITCH_EQ_SPEAKER:
			return sprintf(buf, "%d", eq_speaker);
		case SWITCH_DAC_DIRECT:
			return sprintf(buf, "%d", dac_direct);
		case SWITCH_OVERSAMPLING:
			return sprintf(buf, "%d", dac_oversampling);
		case SWITCH_FLL_TUNING:
			return sprintf(buf, "%d", fll_tuning);
		case SWITCH_MONO_DOWNMIX:
			return sprintf(buf, "%d", mono_downmix);
		case SWITCH_PRIVACY_MODE:
			return sprintf(buf, "%d", privacy_mode);

		case DEBUG_LEVEL:
			return sprintf(buf, "%d", debug_level);

		case DEBUG_INFO:
			return debug_info_show(buf);

		case DEBUG_REG:
			return sprintf(buf, "%d -> %d", debug_register, 
					wm8994_read(codec, debug_register));

		case DEBUG_DUMP:
			return debug_dump_show(buf);

		case HEADPHONE_LEFT:
			return sprintf(buf, "%d", headphone_l);
		case HEADPHONE_RIGHT:
			return sprintf(buf, "%d", headphone_r);

		case SPEAKER_LEFT:
			return sprintf(buf, "%d", speaker_l);
		case SPEAKER_RIGHT:
			return sprintf(buf, "%d", speaker_r);

		case SPEAKER_BOOST_LEVEL:
			return sprintf(buf, "%d", speaker_boost_level);


		case STEREO_EXPANSION:
			return sprintf(buf, "%d", stereo_expansion_gain);

		case MIC_LEVEL_GENERAL:
			return sprintf(buf, "%d", mic_level_general);
		case MIC_LEVEL_CALL:
			return sprintf(buf, "%d", mic_level_call);

		case EQ_HP_GAIN_1:
		case EQ_HP_GAIN_2:
		case EQ_HP_GAIN_3:
		case EQ_HP_GAIN_4:
		case EQ_HP_GAIN_5:
			return sprintf(buf, "%d", eq_gains[EQ_HP][4 - (EQ_HP_GAIN_5 - offset)]);

		case EQ_HP_BAND_1:
		case EQ_HP_BAND_2:
		case EQ_HP_BAND_3:
		case EQ_HP_BAND_4:
		case EQ_HP_BAND_5:
			return band_show(EQ_HP, (4 - (EQ_HP_BAND_5 - offset)), buf);

		case EQ_SP_GAIN_1:
		case EQ_SP_GAIN_2:
		case EQ_SP_GAIN_3:
		case EQ_SP_GAIN_4:
		case EQ_SP_GAIN_5:
			return sprintf(buf, "%d", eq_gains[EQ_SP][4 - (EQ_SP_GAIN_5 - offset)]);

		case EQ_SP_BAND_1:
		case EQ_SP_BAND_2:
		case EQ_SP_BAND_3:
		case EQ_SP_BAND_4:
		case EQ_SP_BAND_5:
			return band_show(EQ_SP, (4 - (EQ_SP_BAND_5 - offset)), buf);

	}

	return -EINVAL;
}

static ssize_t store_sound_property(struct device *dev,
				     struct device_attribute *attr,
				     const char *buf, size_t count)
{
	const ptrdiff_t offset = attr - sound_control_attrs;
	int val, t;
	int in[5] = { 0 };

	if((t = read_into((int*)&in, 5, buf, count)) < 1)
		return -EINVAL;

	if(t == 1) {

	val = in[0];

	switch (offset) {
		case SWITCH_MASTER:
			if(!!val != sound_control)
			sound_control = !!val;
			reset_sound_control();
			break;

		case SWITCH_EQ_HEADPHONE:
			eq = val & (EQ_ENABLED | EQ_SATPREVENT);
			set_eq();
			break;

		case SWITCH_EQ_SPEAKER:
			eq_speaker = !!val;
			set_eq();
			break;

		case SWITCH_DAC_DIRECT:
			dac_direct = !!val;
			set_dac_direct();
			break;

		case SWITCH_OVERSAMPLING:
			dac_oversampling = !!val;
			set_dac_oversampling();
			break;

		case SWITCH_FLL_TUNING:
			fll_tuning = !!val;
			set_fll_tuning();
			break;

		case SWITCH_MONO_DOWNMIX:
			if(!!val != mono_downmix)
			mono_downmix = !!val;
			set_mono_downmix();
			break;

		case SWITCH_PRIVACY_MODE:
			privacy_mode = !!val;
			set_speaker();
			break;

		case DEBUG_LEVEL:
			debug_level = !!val;
			break;

		case DEBUG_DUMP:
			if ((val >= 0) && (val < REGDUMP_BANKS))
				regdump_bank = val;
			break;

		case HEADPHONE_LEFT:
		case HEADPHONE_RIGHT:
			sanitize_min_max(val, HEADPHONE_MIN, HEADPHONE_MAX);

			if(offset == HEADPHONE_LEFT)
				headphone_l = val;
			else
				headphone_r = val;

			set_headphone();
			break;

		case SPEAKER_LEFT:
		case SPEAKER_RIGHT:
			sanitize_min_max(val, SPEAKER_MIN, SPEAKER_MAX);

			if(offset == SPEAKER_LEFT)
				speaker_l = val;
			else
				speaker_r = val;

			set_speaker();
			break;

		case SPEAKER_BOOST_LEVEL:
			sanitize_min_max(val, 0, WM8994_SPKOUTR_BOOST_MASK);
			speaker_boost_level = val;
			set_speaker_boost();
			break;

		case STEREO_EXPANSION:
			sanitize_min_max(val, STEREO_EXPANSION_GAIN_OFF, STEREO_EXPANSION_GAIN_MAX);
			stereo_expansion_gain = val;
			set_stereo_expansion();
			break;

		
		case MIC_LEVEL_GENERAL:
		case MIC_LEVEL_CALL:
			sanitize_min_max(val, MICLEVEL_MIN, MICLEVEL_MAX);

			if(offset == MIC_LEVEL_GENERAL)
				mic_level_general = val;
			else
				mic_level_call = val;
			
			is_mic_controlled = !((mic_level_general == MICLEVEL_GENERAL)
						&& (mic_level_call == MICLEVEL_CALL));

			set_mic_level();
			break;

		case EQ_HP_GAIN_1:
		case EQ_HP_GAIN_2:
		case EQ_HP_GAIN_3:
		case EQ_HP_GAIN_4:
		case EQ_HP_GAIN_5:
			sanitize_min_max(val, EQ_GAIN_MIN, EQ_GAIN_MAX);
			eq_gains[EQ_HP][4 - (EQ_HP_GAIN_5 - offset)] = val;

			set_eq_gains();
			break;

		case EQ_SP_GAIN_1:
		case EQ_SP_GAIN_2:
		case EQ_SP_GAIN_3:
		case EQ_SP_GAIN_4:
		case EQ_SP_GAIN_5:
			sanitize_min_max(val, EQ_GAIN_MIN, EQ_GAIN_MAX);
			eq_gains[EQ_SP][4 - (EQ_SP_GAIN_5 - offset)] = val;

			set_eq_gains();
			break;

		default: return -EINVAL;
	}

	}

	if(t == 3)
	switch (offset) {
		case DEBUG_REG:
			if(in[1] == DEBUG_REGISTER_KEY)
				wm8994_write(codec, in[0], in[2]);
			break;

		default: return -EINVAL;
	}

	if(t == 4)
	switch (offset) {
		case EQ_HP_BAND_1:
		case EQ_HP_BAND_2:
		case EQ_HP_BAND_3:
		case EQ_HP_BAND_4:
		case EQ_HP_BAND_5:
			for(t = 0; t < 4; t++)
				eq_bands[EQ_HP][4 - (EQ_HP_BAND_5 - offset)][t] = in[t];

			set_eq_bands();
			break;

		case EQ_SP_BAND_1:
		case EQ_SP_BAND_2:
		case EQ_SP_BAND_3:
		case EQ_SP_BAND_4:
		case EQ_SP_BAND_5:
			for(t = 0; t < 4; t++)
				eq_bands[EQ_SP][4 - (EQ_SP_BAND_5 - offset)][t] = in[t];

			set_eq_bands();
			break;

		default: return -EINVAL;
	}


	return count;
}

// define control device
static struct miscdevice sound_dev = {
	.minor = MISC_DYNAMIC_MINOR,
	.name = "wolfson_control",
};

/*****************************************/
// Driver init and exit functions
/*****************************************/

static int sound_control_init(void)
{
	int i;

	// register sound control control device
	misc_register(&sound_dev);

	for (i = 0; i < ARRAY_SIZE(sound_control_attrs); i++) {
		if (sysfs_create_file(&sound_dev.this_device->kobj, &sound_control_attrs[i].attr))
			goto create_failed;
	}

	// Initialize sound control master switch with false per default (will be set to correct
	// default value when we receive the codec pointer later - avoids startup boot loop)
	sound_control = false;

	// initialize global variables and default debug level
	initialize_global_variables();

	// One-time only initialisations
	debug_level = DEBUG_DEFAULT;
	regdump_bank = 0;

	// Print debug info
	printk("Audio: engine version %s started\n", SOUND_CONTROL_VERSION);

	return 0;

create_failed:
	pr_info("%s: sound control interface creation failed\n", __func__);

	while (i--)
		sysfs_remove_file(&sound_dev.this_device->kobj, &sound_control_attrs[i].attr);

	return 0;
}


static void sound_control_exit(void)
{
	int i;

	// remove sound control control device
	for (i = 0; i < ARRAY_SIZE(sound_control_attrs); i++) {
		sysfs_remove_file(&sound_dev.this_device->kobj, &sound_control_attrs[i].attr);
	}

	// Print debug info
	printk("Audio: engine stopped\n");
}

/* define driver entry points */

module_init(sound_control_init);
module_exit(sound_control_exit);
