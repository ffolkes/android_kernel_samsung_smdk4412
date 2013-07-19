/*
 * Copyright (C) 2010-2012 ARM Limited. All rights reserved.
 *
 * This program is free software and is provided to you under the terms of the GNU General Public License version 2
 * as published by the Free Software Foundation, and any use by you of this program is subject to the terms of such GNU licence.
 *
 * A copy of the licence is included with the program, and can also be obtained from Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301, USA.
 */

/**
 * @file mali_platform_dvfs.c
 * Platform specific Mali driver dvfs functions
 */

#include "mali_kernel_common.h"
#include "mali_osk.h"
#include "mali_platform.h"

#include <linux/clk.h>
#include <linux/err.h>
#include <linux/regulator/consumer.h>
#include <linux/regulator/driver.h>

#include <linux/sysdev.h>
#include <linux/platform_device.h>
#include <linux/sysfs_helpers.h>

#include <../../../../../input/keyboard/cypress/cypress-touchkey.h>

#include <asm/io.h>

#include "mali_device_pause_resume.h"
#include <linux/workqueue.h>

#define MAX_MALI_DVFS_STEPS 5
#define MALI_DVFS_WATING 10 // msec

#ifdef CONFIG_CPU_FREQ
#include <mach/asv.h>
#define EXYNOS4_ASV_ENABLED
#endif

#include <plat/cpu.h>

static int bMaliDvfsRun=0;

static _mali_osk_atomic_t bottomlock_status;
int bottom_lock_step = 0;

typedef struct mali_dvfs_tableTag{
	unsigned int clock;
	unsigned int freq;
	unsigned int vol;
}mali_dvfs_table;

typedef struct mali_dvfs_statusTag{
	unsigned int currentStep;
	mali_dvfs_table * pCurrentDvfs;

}mali_dvfs_currentstatus;

typedef struct mali_dvfs_thresholdTag{
	unsigned int downthreshold;
	unsigned int upthreshold;
}mali_dvfs_threshold_table;

typedef struct mali_dvfs_staycount{
	unsigned int staycount;
}mali_dvfs_staycount_table;

typedef struct mali_dvfs_stepTag{
	int clk;
	int vol;
}mali_dvfs_step;

mali_dvfs_step step[MALI_DVFS_STEPS]={
	/*step 0 clk*/ {160,   875000},
#if (MALI_DVFS_STEPS > 1)
	/*step 1 clk*/ {266,   900000},
#if (MALI_DVFS_STEPS > 2)
	/*step 2 clk*/ {350,   950000},
#if (MALI_DVFS_STEPS > 3)
	/*step 3 clk*/ {440,  1025000},
#if (MALI_DVFS_STEPS > 4)
	/*step 4 clk*/ {533,  1150000}
#endif
#endif
#endif
#endif
};

mali_dvfs_staycount_table mali_dvfs_staycount[MALI_DVFS_STEPS]={
	/*step 0*/{0},
#if (MALI_DVFS_STEPS > 1)
	/*step 1*/{0},
#if (MALI_DVFS_STEPS > 2)
	/*step 2*/{0},
#if (MALI_DVFS_STEPS > 3)
	/*step 3*/{0},
#if (MALI_DVFS_STEPS > 4)
	/*step 4*/{0}
#endif
#endif
#endif
#endif
};

/* dvfs information */
// L0 = 533Mhz, 1.075V
// L1 = 440Mhz, 1.025V
// L2 = 350Mhz, 0.95V
// L3 = 266Mhz, 0.90V
// L4 = 160Mhz, 0.875V

int step0_clk = 160;
int step0_vol = 875000;
#if (MALI_DVFS_STEPS > 1)
int step1_clk = 266;
int step1_vol = 900000;
int step0_up = 70;
int step1_down = 62;
#if (MALI_DVFS_STEPS > 2)
int step2_clk = 350;
int step2_vol = 950000;
int step1_up = 90;
int step2_down = 85;
#if (MALI_DVFS_STEPS > 3)
int step3_clk = 440;
int step3_vol = 1025000;
int step2_up = 90;
int step3_down = 85;
#if (MALI_DVFS_STEPS > 4)
int step4_clk = 533;
int step4_vol = 1150000;
int step3_up = 90;
int step4_down = 95;
#endif
#endif
#endif
#endif

mali_dvfs_table mali_dvfs_all[MAX_MALI_DVFS_STEPS]={
	{160   ,1000000   ,  875000},
	{266   ,1000000   ,  900000},
	{350   ,1000000   ,  950000},
	{440   ,1000000   , 1025000},
	{533   ,1000000   , 1075000} };

mali_dvfs_table mali_dvfs[MALI_DVFS_STEPS]={
	{160   ,1000000   , 875000},
#if (MALI_DVFS_STEPS > 1)
	{266   ,1000000   , 900000},
#if (MALI_DVFS_STEPS > 2)
	{350   ,1000000   , 950000},
#if (MALI_DVFS_STEPS > 3)
	{440   ,1000000   ,1025000},
#if (MALI_DVFS_STEPS > 4)
	{533   ,1000000   ,1150000}
#endif
#endif
#endif
#endif
};

mali_dvfs_threshold_table mali_dvfs_threshold[MALI_DVFS_STEPS]={
	{0   , 70},
#if (MALI_DVFS_STEPS > 1)
	{62  , 90},
#if (MALI_DVFS_STEPS > 2)
	{85  , 90},
#if (MALI_DVFS_STEPS > 3)
	{85  ,90},
#if (MALI_DVFS_STEPS > 4)
	{95  ,100}
#endif
#endif
#endif
#endif
};

#ifdef EXYNOS4_ASV_ENABLED
#define ASV_LEVEL     12	/* ASV0, 1, 11 is reserved */
#define ASV_LEVEL_PRIME     13	/* ASV0, 1, 12 is reserved */

static unsigned int asv_3d_volt_9_table_1ghz_type[MALI_DVFS_STEPS-1][ASV_LEVEL] = {
	{  975000,  950000,  950000,  950000,  925000,  925000,  925000,  900000,  900000,  900000,  900000,  875000},  /* L3(160Mhz) */
#if (MALI_DVFS_STEPS > 1)
	{ 1000000,  975000,  975000,  975000,  950000,  950000,  950000,  900000,  900000,  900000,  900000,  875000},  /* L2(266Mhz) */
#if (MALI_DVFS_STEPS > 2)
	{ 1075000, 1050000, 1050000, 1050000, 1000000, 1000000, 1000000,  975000,  975000,  975000,  975000,  925000},  /* L1(350Mhz) */
#if (MALI_DVFS_STEPS > 3)
	{ 1125000, 1100000, 1100000, 1100000, 1075000, 1075000, 1075000, 1025000, 1025000, 1025000, 1025000,  975000},  /* L0(440Mhz) */
#endif
#endif
#endif
};

static unsigned int asv_3d_volt_9_table[MALI_DVFS_STEPS-1][ASV_LEVEL] = {
	{  950000,  925000,  900000,  900000,  875000,  875000,  875000,  875000,  850000,  850000,  850000,  850000},  /* L3(160Mhz) */
#if (MALI_DVFS_STEPS > 1)
	{  975000,  950000,  925000,  925000,  925000,  900000,  900000,  875000,  875000,  875000,  875000,  850000},  /* L2(266Mhz) */
#if (MALI_DVFS_STEPS > 2)
	{ 1050000, 1025000, 1000000, 1000000,  975000,  950000,  950000,  950000,  925000,  925000,  925000,  900000},  /* L1(350Mhz) */
#if (MALI_DVFS_STEPS > 3)
	{ 1100000, 1075000, 1050000, 1050000, 1050000, 1025000, 1025000, 1000000, 1000000, 1000000,  975000,  950000},  /* L0(440Mhz) */
#endif
#endif
#endif
};

static unsigned int asv_3d_volt_9_table_for_prime[MALI_DVFS_STEPS][ASV_LEVEL_PRIME] = {
	{  950000,  937500,  925000,  912500,  900000,  887500,  875000,  862500,  875000,  862500,  850000,  850000},  /* L4(160Mhz) */
#if (MALI_DVFS_STEPS > 1)
	{  975000,  962500,  950000,  937500,  925000,  912500,  900000,  887500,  900000,  887500,  875000,  850000},	/* L3(266Mhz) */
#if (MALI_DVFS_STEPS > 2)
	{ 1025000, 1012500, 1000000,  987500,  975000,  962500,  950000,  937500,  950000,  937500,  925000,  850000},	/* L2(350Mhz) */
#if (MALI_DVFS_STEPS > 3)
	{ 1087500, 1075000, 1062500, 1050000, 1037500, 1025000, 1012500, 1000000, 1012500, 1000000,  987500,  850000},	/* L1(440Mhz) */
#if (MALI_DVFS_STEPS > 4)
	{ 1150000, 1137500, 1125000, 1112500, 1100000, 1087500, 1075000, 1062500, 1087500, 1075000, 1062500, 850000},	/* L0(600Mhz) */
#endif
#endif
#endif
#endif
};

static unsigned int asv_3d_volt_9_table_for_prime_orig[MALI_DVFS_STEPS][ASV_LEVEL_PRIME] = {
	{  950000,  937500,  925000,  912500,  900000,  887500,  875000,  862500,  875000,  862500,  850000,  850000},  /* L4(160Mhz) */
#if (MALI_DVFS_STEPS > 1)
	{  975000,  962500,  950000,  937500,  925000,  912500,  900000,  887500,  900000,  887500,  875000,  862500},	/* L3(266Mhz) */
#if (MALI_DVFS_STEPS > 2)
	{ 1025000, 1012500, 1000000,  987500,  975000,  962500,  950000,  937500,  950000,  937500,  925000,  912500},	/* L2(350Mhz) */
#if (MALI_DVFS_STEPS > 3)
	{ 1087500, 1075000, 1062500, 1050000, 1037500, 1025000, 1012500, 1000000, 1012500, 1000000,  987500,  975000},	/* L1(440Mhz) */
#if (MALI_DVFS_STEPS > 4)
	{ 1150000, 1137500, 1125000, 1112500, 1100000, 1087500, 1075000, 1062500, 1087500, 1075000, 1062500, 1050000},	/* L0(600Mhz) */
#endif
#endif
#endif
#endif
};
#endif /* ASV_LEVEL */

/*dvfs status*/
mali_dvfs_currentstatus maliDvfsStatus;
int mali_dvfs_control=0;

static u32 mali_dvfs_utilization = 255;

static void mali_dvfs_work_handler(struct work_struct *w);

static struct workqueue_struct *mali_dvfs_wq = 0;
extern mali_io_address clk_register_map;
extern _mali_osk_lock_t *mali_dvfs_lock;

static u64 time_in_state[MALI_DVFS_STEPS];
static struct timeval last_switch;
static int time_enable = 0;

int mali_runtime_resumed = -1;

static DECLARE_WORK(mali_dvfs_work, mali_dvfs_work_handler);

static unsigned int manual_asv_level = 0;
static int mali_step_lock = -1;
static int mali_step_limit = -1;

static void do_time_slice(int level)
{
	u64 delta;
	struct timeval now;	
	
	do_gettimeofday(&now);
	
	delta = (now.tv_sec - last_switch.tv_sec) * USEC_PER_SEC +
		    (now.tv_usec - last_switch.tv_usec);

	time_in_state[level] += delta;

	last_switch = now;
}

/* lock/unlock CPU freq by Mali */
#include <linux/types.h>
#include <mach/cpufreq.h>

atomic_t mali_cpufreq_lock;

int cpufreq_lock_by_mali(unsigned int freq)
{
#ifdef CONFIG_EXYNOS4_CPUFREQ
/* #if defined(CONFIG_CPU_FREQ) && defined(CONFIG_ARCH_EXYNOS4) */
	unsigned int level;

	if (atomic_read(&mali_cpufreq_lock) == 0) {
		if (exynos_cpufreq_get_level(freq * 1000, &level)) {
			printk(KERN_ERR
				"Mali: failed to get cpufreq level for %dMHz",
				freq);
			return -EINVAL;
		}

		if (exynos_cpufreq_lock(DVFS_LOCK_ID_G3D, level)) {
			printk(KERN_ERR
				"Mali: failed to cpufreq lock for L%d", level);
			return -EINVAL;
		}

		atomic_set(&mali_cpufreq_lock, 1);
		printk(KERN_DEBUG "Mali: cpufreq locked on <%d>%dMHz\n", level,
									freq);
	}
#endif
	return 0;
}

void cpufreq_unlock_by_mali(void)
{
#ifdef CONFIG_EXYNOS4_CPUFREQ
/* #if defined(CONFIG_CPU_FREQ) && defined(CONFIG_ARCH_EXYNOS4) */
	if (atomic_read(&mali_cpufreq_lock) == 1) {
		exynos_cpufreq_lock_free(DVFS_LOCK_ID_G3D);
		atomic_set(&mali_cpufreq_lock, 0);
		printk(KERN_DEBUG "Mali: cpufreq locked off\n");
	}
#endif
}

static unsigned int get_mali_dvfs_status(void)
{
	return maliDvfsStatus.currentStep;
}
#if MALI_PMM_RUNTIME_JOB_CONTROL_ON
int get_mali_dvfs_control_status(void)
{
	return mali_dvfs_control;
}

mali_bool set_mali_dvfs_current_step(unsigned int step)
{
	_mali_osk_lock_wait(mali_dvfs_lock, _MALI_OSK_LOCKMODE_RW);
	if(!!time_enable)
		do_time_slice(maliDvfsStatus.currentStep);
	maliDvfsStatus.currentStep = step % MAX_MALI_DVFS_STEPS;
	if (step >= MAX_MALI_DVFS_STEPS)
		mali_runtime_resumed = maliDvfsStatus.currentStep;
	_mali_osk_lock_signal(mali_dvfs_lock, _MALI_OSK_LOCKMODE_RW);
	return MALI_TRUE;
}
#endif
static mali_bool set_mali_dvfs_status(u32 step,mali_bool boostup)
{
	u32 validatedStep=step;
	int err;
	
	if (mali_step_lock > -1 && step != mali_step_lock) {
		step = mali_step_lock;
	}

#ifdef CONFIG_REGULATOR
	if (mali_regulator_get_usecount() == 0) {
		MALI_DEBUG_PRINT(1, ("regulator use_count is 0 \n"));
		return MALI_FALSE;
	}
#endif

	if (boostup) {
#ifdef CONFIG_REGULATOR
		/*change the voltage*/
		mali_regulator_set_voltage(mali_dvfs[step].vol, mali_dvfs[step].vol);
#endif
		/*change the clock*/
		mali_clk_set_rate(mali_dvfs[step].clock, mali_dvfs[step].freq);
	} else {
		/*change the clock*/
		mali_clk_set_rate(mali_dvfs[step].clock, mali_dvfs[step].freq);
#ifdef CONFIG_REGULATOR
		/*change the voltage*/
		mali_regulator_set_voltage(mali_dvfs[step].vol, mali_dvfs[step].vol);
#endif
	}

#ifdef EXYNOS4_ASV_ENABLED
#ifndef CONFIG_ABB_CONTROL
	if (samsung_rev() < EXYNOS4412_REV_2_0) {
		if (mali_dvfs[step].clock == 160)
			exynos4x12_set_abb_member(ABB_G3D, ABB_MODE_100V);
		else
			exynos4x12_set_abb_member(ABB_G3D, ABB_MODE_130V);
	}
#else
	abb_target(ABB_G3D, mali_dvfs[step].clock*1000);
#endif
#endif


	set_mali_dvfs_current_step(validatedStep);
	/*for future use*/
	maliDvfsStatus.pCurrentDvfs = &mali_dvfs[validatedStep];

#if CPUFREQ_LOCK_DURING_440
	/* lock/unlock CPU freq by Mali */
	if (mali_dvfs[step].clock == 440)
		err = cpufreq_lock_by_mali(1200);
	else
		cpufreq_unlock_by_mali();
#endif

	return MALI_TRUE;
}

static void mali_platform_wating(u32 msec)
{
	/*sample wating
	change this in the future with proper check routine.
	*/
	unsigned int read_val;
	while(1) {
		read_val = _mali_osk_mem_ioread32(clk_register_map, 0x00);
		if ((read_val & 0x8000)==0x0000) break;
			_mali_osk_time_ubusydelay(100); // 1000 -> 100 : 20101218
		}
		/* _mali_osk_time_ubusydelay(msec*1000);*/
}

static mali_bool change_mali_dvfs_status(u32 step, mali_bool boostup )
{

	MALI_DEBUG_PRINT(1, ("> change_mali_dvfs_status: %d, %d \n",step, boostup));

	if (!set_mali_dvfs_status(step, boostup)) {
		MALI_DEBUG_PRINT(1, ("error on set_mali_dvfs_status: %d, %d \n",step, boostup));
		return MALI_FALSE;
	}

	/*wait until clock and voltage is stablized*/
	mali_platform_wating(MALI_DVFS_WATING); /*msec*/

	return MALI_TRUE;
}

#ifdef EXYNOS4_ASV_ENABLED
extern unsigned int exynos_result_of_asv;

static mali_bool mali_dvfs_table_update(void)
{
	unsigned int i;
	unsigned int step_num = MALI_DVFS_STEPS;
	unsigned int asv_level;

	if(samsung_rev() < EXYNOS4412_REV_2_0)
		step_num = MALI_DVFS_STEPS - 1;
	
	if (manual_asv_level > 0) {
		asv_level = manual_asv_level;
	} else {
		asv_level = exynos_result_of_asv;
	}
	
	printk("mali: using manual_asv_level: %d\n", asv_level);

	if(soc_is_exynos4412()) {
		if (exynos_armclk_max == 1000000) {
			MALI_PRINT(("::C::exynos_result_of_asv : %d\n", asv_level));
			for (i = 0; i < step_num; i++) {
				mali_dvfs[i].vol = asv_3d_volt_9_table_1ghz_type[i][asv_level];
				MALI_PRINT(("mali_dvfs[%d].vol = %d \n", i, mali_dvfs[i].vol));
			}
		} else if(((is_special_flag() >> G3D_LOCK_FLAG) & 0x1) && (samsung_rev() >= EXYNOS4412_REV_2_0)) {
			MALI_PRINT(("::L::exynos_result_of_asv : %d\n", asv_level));
			for (i = 0; i < step_num; i++) {
				mali_dvfs[i].vol = asv_3d_volt_9_table_for_prime[i][asv_level] + 25000;
				MALI_PRINT(("mali_dvfs[%d].vol = %d \n ", i, mali_dvfs[i].vol));
			}
		} else if (samsung_rev() >= EXYNOS4412_REV_2_0) {
			MALI_PRINT(("::P::exynos_result_of_asv : %d\n", asv_level));
			for (i = 0; i < step_num; i++) {
				mali_dvfs[i].vol = asv_3d_volt_9_table_for_prime[i][asv_level];
				MALI_PRINT(("mali_dvfs[%d].vol = %d \n", i, mali_dvfs[i].vol));
			}
		} else {
			MALI_PRINT(("::Q::exynos_result_of_asv : %d\n", asv_level));
			for (i = 0; i < step_num; i++) {
				mali_dvfs[i].vol = asv_3d_volt_9_table[i][asv_level];
				MALI_PRINT(("mali_dvfs[%d].vol = %d \n", i, mali_dvfs[i].vol));
			}
		}
	}

	return MALI_TRUE;
}
#endif

static unsigned int decideNextStatus(unsigned int utilization)
{
	static unsigned int level = 0; // 0:stay, 1:up
	static int mali_dvfs_clk = 0;
	unsigned int mali_dvfs_steps = MALI_DVFS_STEPS;
	
	if (mali_step_limit > -1) {
		mali_dvfs_steps = mali_step_limit;
	}

	if (mali_runtime_resumed >= 0) {
		level = mali_runtime_resumed;
		mali_runtime_resumed = -1;
	}

	if (mali_dvfs_threshold[maliDvfsStatus.currentStep].upthreshold
			<= mali_dvfs_threshold[maliDvfsStatus.currentStep].downthreshold) {
		/*MALI_PRINT(("upthreadshold is smaller than downthreshold: %d < %d\n",
				mali_dvfs_threshold[maliDvfsStatus.currentStep].upthreshold,
				mali_dvfs_threshold[maliDvfsStatus.currentStep].downthreshold));*/
		return level;
	}

	if (!mali_dvfs_control && level == maliDvfsStatus.currentStep) {
		if (utilization > (int)(255 * mali_dvfs_threshold[maliDvfsStatus.currentStep].upthreshold / 100) &&
				level < mali_dvfs_steps - 1) {
			level++;
			if ((samsung_rev() < EXYNOS4412_REV_2_0) && (maliDvfsStatus.currentStep == 3)) {
				level=get_mali_dvfs_status();
			}
		}
		if (utilization < (int)(255 * mali_dvfs_threshold[maliDvfsStatus.currentStep].downthreshold / 100) &&
				level > 0) {
			level--;
		}
	} else if (mali_dvfs_control == 999) {
		int i = 0;
		for (i = 0; i < mali_dvfs_steps; i++) {
			step[i].clk = mali_dvfs_all[i].clock;
		}
#ifdef EXYNOS4_ASV_ENABLED
		mali_dvfs_table_update();
#endif
		i = 0;
		for (i = 0; i < mali_dvfs_steps; i++) {
			mali_dvfs[i].clock = step[i].clk;
		}
		mali_dvfs_control = 0;
		level = 0;

		step0_clk = step[0].clk;
		change_dvfs_tableset(step0_clk, 0);
#if (MALI_DVFS_STEPS > 1)
		step1_clk = step[1].clk;
		change_dvfs_tableset(step1_clk, 1);
#if (MALI_DVFS_STEPS > 2)
		step2_clk = step[2].clk;
		change_dvfs_tableset(step2_clk, 2);
#if (MALI_DVFS_STEPS > 3)
		step3_clk = step[3].clk;
		change_dvfs_tableset(step3_clk, 3);
#if (MALI_DVFS_STEPS > 4)
		step4_clk = step[4].clk;
		change_dvfs_tableset(step4_clk, 4);
#endif
#endif
#endif
#endif
	} else if (mali_dvfs_control != mali_dvfs_clk && mali_dvfs_control != 999) {
		if (mali_dvfs_control < mali_dvfs_all[1].clock && mali_dvfs_control > 0) {
			int i = 0;
			for (i = 0; i < mali_dvfs_steps; i++) {
				step[i].clk = mali_dvfs_all[0].clock;
			}
			maliDvfsStatus.currentStep = 0;
		} else if (mali_dvfs_control < mali_dvfs_all[2].clock && mali_dvfs_control >= mali_dvfs_all[1].clock) {
			int i = 0;
			for (i = 0; i < mali_dvfs_steps; i++) {
				step[i].clk = mali_dvfs_all[1].clock;
			}
			maliDvfsStatus.currentStep = 1;
		} else if (mali_dvfs_control < mali_dvfs_all[3].clock && mali_dvfs_control >= mali_dvfs_all[2].clock) {
			int i = 0;
			for (i = 0; i < mali_dvfs_steps; i++) {
				step[i].clk = mali_dvfs_all[2].clock;
			}
			maliDvfsStatus.currentStep = 2;
		} else if (mali_dvfs_control < mali_dvfs_all[4].clock && mali_dvfs_control >= mali_dvfs_all[3].clock) {
			int i = 0;
			for (i = 0; i < mali_dvfs_steps; i++) {
				step[i].clk  = mali_dvfs_all[3].clock;
			}
			maliDvfsStatus.currentStep = 3;
		} else {
			int i = 0;
			for (i = 0; i < mali_dvfs_steps; i++) {
				step[i].clk  = mali_dvfs_all[4].clock;
			}
			maliDvfsStatus.currentStep = 4;
		}
		step0_clk = step[0].clk;
		change_dvfs_tableset(step0_clk, 0);
#if (MALI_DVFS_STEPS > 1)
		step1_clk = step[1].clk;
		change_dvfs_tableset(step1_clk, 1);
#if (MALI_DVFS_STEPS > 2)
		step2_clk = step[2].clk;
		change_dvfs_tableset(step2_clk, 2);
#if (MALI_DVFS_STEPS > 3)
		step3_clk = step[3].clk;
		change_dvfs_tableset(step3_clk, 3);
#if (MALI_DVFS_STEPS > 4)
		step4_clk = step[4].clk;
		change_dvfs_tableset(step4_clk, 4);
#endif
#endif
#endif
#endif
		level = maliDvfsStatus.currentStep;
	}

	mali_dvfs_clk = mali_dvfs_control;

	if (_mali_osk_atomic_read(&bottomlock_status) > 0) {
		if (level < bottom_lock_step)
			level = bottom_lock_step;
	}

	return level;
}

static mali_bool mali_dvfs_status(u32 utilization)
{
	unsigned int nextStatus = 0;
	unsigned int curStatus = 0;
	mali_bool boostup = MALI_FALSE;
	static int stay_count = 0;
#ifdef EXYNOS4_ASV_ENABLED
	static mali_bool asv_applied = MALI_FALSE;
#endif

	MALI_DEBUG_PRINT(1, ("> mali_dvfs_status: %d \n",utilization));
#ifdef EXYNOS4_ASV_ENABLED
	if (asv_applied == MALI_FALSE) {
		mali_dvfs_table_update();
		change_mali_dvfs_status(1, 0);
		asv_applied = MALI_TRUE;

		return MALI_TRUE;
	}
#endif

	/*decide next step*/
	curStatus = get_mali_dvfs_status();
	nextStatus = decideNextStatus(utilization);
	
	if (mali_step_lock > -1 && nextStatus != mali_step_lock) {
		nextStatus = mali_step_lock;
	}

	MALI_DEBUG_PRINT(1, ("= curStatus %d, nextStatus %d, maliDvfsStatus.currentStep %d \n", curStatus, nextStatus, maliDvfsStatus.currentStep));

	/*if next status is same with current status, don't change anything*/
	if ((curStatus != nextStatus && stay_count == 0)) {
		/*check if boost up or not*/
		if (nextStatus > maliDvfsStatus.currentStep) boostup = 1;

		/*change mali dvfs status*/
		if (!change_mali_dvfs_status(nextStatus,boostup)) {
			MALI_DEBUG_PRINT(1, ("error on change_mali_dvfs_status \n"));
			return MALI_FALSE;
		}
		stay_count = mali_dvfs_staycount[maliDvfsStatus.currentStep].staycount;
	} else {
		if (stay_count > 0)
			stay_count--;
	}

	return MALI_TRUE;
}



int mali_dvfs_is_running(void)
{
	return bMaliDvfsRun;

}



void mali_dvfs_late_resume(void)
{
	// set the init clock as low when resume
	set_mali_dvfs_status(0,0);
}


static void mali_dvfs_work_handler(struct work_struct *w)
{
	int change_clk = 0;
	int change_step = 0;
	bMaliDvfsRun=1;

	/* dvfs table change when clock was changed */
	if (step0_clk != mali_dvfs[0].clock) {
		MALI_PRINT(("::: step0_clk change to %d Mhz\n", step0_clk));
		change_clk = step0_clk;
		change_step = 0;
		step0_clk = change_dvfs_tableset(change_clk, change_step);
	}
#if (MALI_DVFS_STEPS > 1)
	if (step1_clk != mali_dvfs[1].clock) {
		MALI_PRINT(("::: step1_clk change to %d Mhz\n", step1_clk));
		change_clk = step1_clk;
		change_step = 1;
		step1_clk = change_dvfs_tableset(change_clk, change_step);
	}
	if (step0_up != mali_dvfs_threshold[0].upthreshold) {
		MALI_PRINT(("::: step0_up change to %d %\n", step0_up));
		mali_dvfs_threshold[0].upthreshold = step0_up;
	}
	if (step1_down != mali_dvfs_threshold[1].downthreshold) {
		MALI_PRINT((":::step1_down change to %d %\n", step1_down));
		mali_dvfs_threshold[1].downthreshold = step1_down;
	}
#if (MALI_DVFS_STEPS > 2)
	if (step2_clk != mali_dvfs[2].clock) {
		MALI_PRINT(("::: step2_clk change to %d Mhz\n", step2_clk));
		change_clk = step2_clk;
		change_step = 2;
		step2_clk = change_dvfs_tableset(change_clk, change_step);
	}
	if (step1_up != mali_dvfs_threshold[1].upthreshold) {
		MALI_PRINT((":::step1_up change to %d %\n", step1_up));
		mali_dvfs_threshold[1].upthreshold = step1_up;
	}
	if (step2_down != mali_dvfs_threshold[2].downthreshold) {
		MALI_PRINT((":::step2_down change to %d %\n", step2_down));
		mali_dvfs_threshold[2].downthreshold = step2_down;
	}
#if (MALI_DVFS_STEPS > 3)
	if (step3_clk != mali_dvfs[3].clock) {
		MALI_PRINT(("::: step3_clk change to %d Mhz\n", step3_clk));
		change_clk = step3_clk;
		change_step = 3;
		step3_clk = change_dvfs_tableset(change_clk, change_step);
	}
	if (step2_up != mali_dvfs_threshold[2].upthreshold) {
		MALI_PRINT((":::step2_up change to %d %\n", step2_up));
		mali_dvfs_threshold[2].upthreshold = step2_up;
	}
	if (step3_down != mali_dvfs_threshold[3].downthreshold) {
		MALI_PRINT((":::step3_down change to %d %\n", step3_down));
		mali_dvfs_threshold[3].downthreshold = step3_down;
	}
#if (MALI_DVFS_STEPS > 4)
	if (step4_clk != mali_dvfs[4].clock) {
		MALI_PRINT(("::: step4_clk change to %d Mhz\n", step4_clk));
		change_clk = step4_clk;
		change_step = 4;
		step4_clk = change_dvfs_tableset(change_clk, change_step);
	}
	if (step3_up != mali_dvfs_threshold[3].upthreshold) {
		MALI_PRINT((":::step3_up change to %d %\n", step3_up));
		mali_dvfs_threshold[3].upthreshold = step3_up;
	}
	if (step4_down != mali_dvfs_threshold[4].downthreshold) {
		MALI_PRINT((":::step4_down change to %d %\n", step4_down));
		mali_dvfs_threshold[4].downthreshold = step4_down;
	}
#endif
#endif
#endif
#endif

#ifdef DEBUG
	mali_dvfs[0].vol = step0_vol;
	mali_dvfs[1].vol = step1_vol;
	mali_dvfs[2].vol = step2_vol;
	mali_dvfs[3].vol = step3_vol;
	mali_dvfs[4].vol = step4_vol;
#endif
	MALI_DEBUG_PRINT(3, ("=== mali_dvfs_work_handler\n"));

	if (!mali_dvfs_status(mali_dvfs_utilization))
		MALI_DEBUG_PRINT(1,( "error on mali dvfs status in mali_dvfs_work_handler"));

	bMaliDvfsRun=0;
}

/*static ssize_t lvl160_upthreshold_show(struct sysdev_class * cls, 
							 struct sysdev_class_attribute *attr, char *buf)
{
	return sprintf(buf, "%d\n", mali_dvfs_threshold[0].upthreshold);
}

static ssize_t lvl160_upthreshold_store(struct sysdev_class * cls, struct sysdev_class_attribute *attr,
							  const char *buf, size_t count) 
{
	unsigned int ret, input;
	
	ret = sscanf(buf, "%d", &input);
	
	if (ret && (input >= 0 && input <= 100)) {
		mali_dvfs_threshold[0].upthreshold = input;
	} else {
		return -EINVAL;
	}
	
	return count;	
}

static ssize_t current_freq_show(struct sysdev_class * cls, 
								 struct sysdev_class_attribute *attr, char *buf)
{
	return sprintf(buf, "%d\n", mali_dvfs[maliDvfsStatus.currentStep].clock);
}

static ssize_t max_freq_show(struct sysdev_class * cls, 
							 struct sysdev_class_attribute *attr, char *buf)
{
	return sprintf(buf, "%d\n", 1);
}

static ssize_t max_freq_store(struct sysdev_class * cls, struct sysdev_class_attribute *attr,
							  const char *buf, size_t count) 
{

	return count;	
}

struct sysdev_class gpu_m400_sysclass = {
	.name	= "gpu",
};

static SYSDEV_CLASS_ATTR(max_freq, S_IRUGO | S_IWUGO, max_freq_show, max_freq_store);
static SYSDEV_CLASS_ATTR(lvl160_upthreshold, S_IRUGO | S_IWUGO, lvl160_upthreshold_show, lvl160_upthreshold_store);
static SYSDEV_CLASS_ATTR(current_freq, S_IRUGO, current_freq_show, NULL);

static struct sysdev_class_attribute *gpu_attributes[] = {
	&attr_max_freq,
	&attr_lvl160_upthreshold,
	&attr_current_freq,
};*/

mali_bool init_mali_dvfs_status(int step)
{
	int i = 0;
	
	/*init sysinterfaces
	sysdev_class_register(&gpu_m400_sysclass);
	for (i = 0;  i < ARRAY_SIZE(gpu_attributes); i++) {
		sysdev_class_create_file(&gpu_m400_sysclass, gpu_attributes[i]);
	}*/
	
	/*default status
	add here with the right function to get initilization value.
	*/
	if (!mali_dvfs_wq)
		mali_dvfs_wq = create_singlethread_workqueue("mali_dvfs");

	_mali_osk_atomic_init(&bottomlock_status, 0);

	/*add a error handling here*/
	set_mali_dvfs_current_step(step);

	return MALI_TRUE;
}

void deinit_mali_dvfs_status(void)
{
	if (mali_dvfs_wq)
		destroy_workqueue(mali_dvfs_wq);

	_mali_osk_atomic_term(&bottomlock_status);

	mali_dvfs_wq = NULL;
}

mali_bool mali_dvfs_handler(u32 utilization)
{
	mali_dvfs_utilization = utilization;
	queue_work_on(0, mali_dvfs_wq,&mali_dvfs_work);

	/*add error handle here*/
	return MALI_TRUE;
}

int change_dvfs_tableset(int change_clk, int change_step)
{
	int err;

	if (change_clk < mali_dvfs_all[1].clock) {
		mali_dvfs[change_step].clock = mali_dvfs_all[0].clock;
	} else if (change_clk < mali_dvfs_all[2].clock && change_clk >= mali_dvfs_all[1].clock) {
		mali_dvfs[change_step].clock = mali_dvfs_all[1].clock;
	} else if (change_clk < mali_dvfs_all[3].clock && change_clk >= mali_dvfs_all[2].clock) {
		mali_dvfs[change_step].clock = mali_dvfs_all[2].clock;
	} else if (change_clk < mali_dvfs_all[4].clock && change_clk >= mali_dvfs_all[3].clock) {
		mali_dvfs[change_step].clock = mali_dvfs_all[3].clock;
	} else {
		mali_dvfs[change_step].clock = mali_dvfs_all[4].clock;
	}

	MALI_PRINT((":::mali dvfs step %d clock and voltage = %d Mhz, %d V\n",change_step, mali_dvfs[change_step].clock, mali_dvfs[change_step].vol));

	if (maliDvfsStatus.currentStep == change_step) {
#ifdef CONFIG_REGULATOR
		/*change the voltage*/
		mali_regulator_set_voltage(mali_dvfs[change_step].vol, mali_dvfs[change_step].vol);
#endif
		/*change the clock*/
		mali_clk_set_rate(mali_dvfs[change_step].clock, mali_dvfs[change_step].freq);

#if CPUFREQ_LOCK_DURING_440
		/* lock/unlock CPU freq by Mali */
		if (mali_dvfs[change_step].clock == 440)
			err = cpufreq_lock_by_mali(1200);
		else
			cpufreq_unlock_by_mali();
#endif
	}

	return mali_dvfs[change_step].clock;
}

void mali_default_step_set(int step, mali_bool boostup)
{
	mali_clk_set_rate(mali_dvfs[step].clock, mali_dvfs[step].freq);

	if (maliDvfsStatus.currentStep == 1)
		set_mali_dvfs_status(step, boostup);
}

int mali_dvfs_bottom_lock_push(int lock_step)
{
	int prev_status = _mali_osk_atomic_read(&bottomlock_status);

	if (prev_status < 0) {
		MALI_PRINT(("gpu bottom lock status is not valid for push\n"));
		return -1;
	}
	if (bottom_lock_step < lock_step) {
		bottom_lock_step = lock_step;
		if (get_mali_dvfs_status() < lock_step) {
			mali_regulator_set_voltage(mali_dvfs[lock_step].vol, mali_dvfs[lock_step].vol);
			mali_clk_set_rate(mali_dvfs[lock_step].clock, mali_dvfs[lock_step].freq);
			set_mali_dvfs_current_step(lock_step);
		}
	}
	return _mali_osk_atomic_inc_return(&bottomlock_status);
}

int mali_dvfs_bottom_lock_pop(void)
{
	int prev_status = _mali_osk_atomic_read(&bottomlock_status);
	if (prev_status <= 0) {
		MALI_PRINT(("gpu bottom lock status is not valid for pop\n"));
		return -1;
	} else if (prev_status == 1) {
		bottom_lock_step = 0;
		MALI_PRINT(("gpu bottom lock release\n"));
	}

	return _mali_osk_atomic_dec_return(&bottomlock_status);
}

int mali_dvfs_get_vol(int step)
{
	step = step % MAX_MALI_DVFS_STEPS;
	MALI_DEBUG_ASSERT(step<MAX_MALI_DVFS_STEPS);

	return mali_dvfs[step].vol;
}

#if MALI_VOLTAGE_LOCK
int mali_vol_get_from_table(int vol)
{
	int i;
	for (i = 0; i < MALI_DVFS_STEPS; i++) {
		if (mali_dvfs[i].vol >= vol)
			return mali_dvfs[i].vol;
	}
	MALI_PRINT(("Failed to get voltage from mali_dvfs table, maximum voltage is %d uV\n", mali_dvfs[MALI_DVFS_STEPS-1].vol));
	return 0;
}
#endif

void set_mali_asv(unsigned int level) {
	
	if (level > 0 && level < 11) {
		manual_asv_level = level;
	} else {
		manual_asv_level = 0;
	}
	
	//mali_dvfs_table_update();
	
	printk("mali: manual_asv_level: %d\n", manual_asv_level);
}
EXPORT_SYMBOL(set_mali_asv);

unsigned int get_mali_asv(void) {
	
	return mali_dvfs[maliDvfsStatus.currentStep].vol;
	
}
EXPORT_SYMBOL(get_mali_asv);

unsigned int get_mali_cur_freq(void) {
	
	return mali_dvfs[maliDvfsStatus.currentStep].clock;
	
}
EXPORT_SYMBOL(get_mali_cur_freq);

void set_mali_up_threshold_by_step(unsigned int level, unsigned int threshold) {
	
	//unsigned int prev_step = maliDvfsStatus.currentStep;
	level--;
	
	mali_dvfs_threshold[level].upthreshold = threshold;
	
	// there is no step 4 up, as that's already the highest.
	if (level == 0) {
		step0_up = threshold;
	} else if (level == 1) {
		step1_up = threshold;
	} else if (level == 2) {
		step2_up = threshold;
	} else if (level == 3) {
		step3_up = threshold;
	}
	
	/*if (prev_step != 1) {
		// cycle things. make sure we cycle to a different step than current.
		change_mali_dvfs_status(1, 0);
		change_mali_dvfs_status(prev_step, 1);
	} else {
		change_mali_dvfs_status(2, 1);
		change_mali_dvfs_status(prev_step, 1);
	}*/
	
	printk("mali: set_mali_up_threshold_by_step() step: %d, up_threshold: %d\n", level, threshold);
}
EXPORT_SYMBOL(set_mali_up_threshold_by_step);

void set_mali_down_threshold_by_step(unsigned int level, unsigned int threshold) {
	
	//unsigned int prev_step = maliDvfsStatus.currentStep;
	level--;
	
	mali_dvfs_threshold[level].downthreshold = threshold;
	
	// there is no step 0 down, as that's already the lowest.
	if (level == 1) {
		step1_down = threshold;
	} else if (level == 2) {
		step2_down = threshold;
	} else if (level == 3) {
		step3_down = threshold;
	} else if (level == 4) {
		step4_down = threshold;
	}
	
	/*if (prev_step != 1) {
		// cycle things. make sure we cycle to a different step than current.
		change_mali_dvfs_status(1, 0);
		change_mali_dvfs_status(prev_step, 1);
	} else {
		change_mali_dvfs_status(2, 1);
		change_mali_dvfs_status(prev_step, 1);
	}*/
	
	printk("mali: set_mali_down_threshold_by_step() step: %d, down_threshold: %d\n", level, threshold);
}
EXPORT_SYMBOL(set_mali_down_threshold_by_step);

void set_mali_step_freq(unsigned int level, unsigned int freq) {
	
	unsigned int prev_step = maliDvfsStatus.currentStep;
	level--;
	
	step[level].clk = freq;
	mali_dvfs_all[level].clock = freq;
	mali_dvfs[level].clock = freq;
	
	if (level == 0) {
		step0_clk = freq;
	} else if (level == 1) {
		step1_clk = freq;
	} else if (level == 2) {
		step2_clk = freq;
	} else if (level == 3) {
		step3_clk = freq;
	} else if (level == 4) {
		step4_clk = freq;
	}
	
	if (prev_step != 1) {
		// cycle things. make sure we cycle to a different step than current.
		change_mali_dvfs_status(1, 0);
		change_mali_dvfs_status(prev_step, 1);
	} else {
		change_mali_dvfs_status(2, 1);
		change_mali_dvfs_status(prev_step, 1);
	}
	
	printk("mali: set_mali_step_freq() step: %d, freq: %d\n", level, freq);
}
EXPORT_SYMBOL(set_mali_step_freq);

void set_mali_step_volt(unsigned int level, unsigned int volt) {
	
	unsigned int prev_step = maliDvfsStatus.currentStep;
	level--;
	
	if (volt == 0) {
		// enable asv default for this step.
		asv_3d_volt_9_table_for_prime[level][exynos_result_of_asv] = asv_3d_volt_9_table_for_prime_orig[level][exynos_result_of_asv];
		mali_dvfs_table_update();
		printk("mali: set_mali_step_volt() setting to asv default. step: %d, volt: %d\n", level, volt);
	} else {
	
		step[level].vol = volt;
		mali_dvfs_all[level].vol = volt;
		mali_dvfs[level].vol = volt;
	
		if (level == 0) {
			step0_vol = volt;
		} else if (level == 1) {
			step1_vol = volt;
		} else if (level == 2) {
			step2_vol = volt;
		} else if (level == 3) {
			step3_vol = volt;
		} else if (level == 4) {
			step4_vol = volt;
		}
	
	}
	
	if (prev_step != 1) {
		// cycle things. make sure we cycle to a different step than current.
		change_mali_dvfs_status(1, 0);
		change_mali_dvfs_status(prev_step, 1);
	} else {
		change_mali_dvfs_status(2, 1);
		change_mali_dvfs_status(prev_step, 1);
	}
	
	printk("mali: set_mali_step_volt() step: %d, volt: %d\n", level, volt);
}
EXPORT_SYMBOL(set_mali_step_volt);

unsigned int get_mali_freq_by_step(unsigned int level) {
	
	level--;
	
	printk("mali: get_mali_freq_by_step() step: %d, freq: %d\n", level, mali_dvfs[level].clock);
	return mali_dvfs[level].clock;
}
EXPORT_SYMBOL(get_mali_freq_by_step);

unsigned int get_mali_volt_by_step(unsigned int level) {
	
	level--;
	
	printk("mali: get_mali_volt_by_step() step: %d, volt: %d\n", level, mali_dvfs[level].vol);
	return mali_dvfs[level].vol;
}
EXPORT_SYMBOL(get_mali_volt_by_step);

unsigned int get_mali_up_threshold_by_step(unsigned int level) {
	
	level--;
	
	printk("mali: get_mali_up_threshold_by_step() step: %d, up_threshold: %d\n", level, mali_dvfs_threshold[level].upthreshold);
	return mali_dvfs_threshold[level].upthreshold;
}
EXPORT_SYMBOL(get_mali_up_threshold_by_step);

unsigned int get_mali_down_threshold_by_step(unsigned int level) {
	
	level--;
	
	printk("mali: get_mali_down_threshold_by_step() step: %d, down_threshold: %d\n", level, mali_dvfs_threshold[level].downthreshold);
	return mali_dvfs_threshold[level].downthreshold;
}
EXPORT_SYMBOL(get_mali_down_threshold_by_step);

unsigned int get_mali_step_lock(void) {

	if (mali_step_lock == -1)
		return 0;
	else
		return mali_step_lock + 1;
}
EXPORT_SYMBOL(get_mali_step_lock);

void set_mali_step_lock(unsigned int level) {
	
	level--;
	
	if (step > 0) {
		mali_step_lock = level;
	} else {
		mali_step_lock = -1;
	}
	
	printk("mali: freq lock set to: %d\n", level);
}
EXPORT_SYMBOL(set_mali_step_lock);

unsigned int get_mali_step_limit(void) {

	if (mali_step_limit == -1)
		return 0;
	else
		return mali_step_limit;
}
EXPORT_SYMBOL(get_mali_step_limit);

void set_mali_step_limit(unsigned int level) {
	
	// since this limit takes the place of MALI_DVFS_STEPS,
	// we don't need to adjust for 0 with level--;
	
	if (level > 0) {
		mali_step_limit = level;
	} else {
		mali_step_limit = -1;
	}
	
	printk("mali: freq limit set to step %d and below\n", level);
}
EXPORT_SYMBOL(set_mali_step_limit);

u64 get_mali_time_in_state(unsigned int level) {

	level--;

	return time_in_state[level];
}
EXPORT_SYMBOL(get_mali_time_in_state);

void set_mali_time_in_state(unsigned int mode) {

	int i;
	
	switch(mode) {
		case 1:
			time_enable = 1;
			break;
		case 0:
			time_enable = 0;
			break;
		default: 
			for (i = 0; i < MALI_DVFS_STEPS; i++)
				time_in_state[i] = 0;
	}
}
EXPORT_SYMBOL(set_mali_time_in_state);

unsigned int get_mali_utilization_timeout(void) {

	return mali_gpu_utilization_timeout;
}
EXPORT_SYMBOL(get_mali_utilization_timeout);

void set_mali_utilization_timeout(unsigned int ms) {

	if (mali_gpu_utilization_timeout < 25)
		mali_gpu_utilization_timeout = 25;
		
	if (mali_gpu_utilization_timeout > 5000)
		mali_gpu_utilization_timeout = 5000;
		
	mali_gpu_utilization_timeout = ms;
}
EXPORT_SYMBOL(set_mali_utilization_timeout);
