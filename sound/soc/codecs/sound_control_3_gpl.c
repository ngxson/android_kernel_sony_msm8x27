/*
 * Author: Paul Reioux aka Faux123 <reioux@gmail.com>
 *
 * WCD93xx sound control module
 * Copyright 2013 Paul Reioux
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

#include <linux/module.h>
#include <linux/kobject.h>
#include <linux/sysfs.h>
#include <linux/kallsyms.h>
#include <linux/mfd/wcd9xxx/wcd9304_registers.h>

#define SOUND_CONTROL_MAJOR_VERSION	3
#define SOUND_CONTROL_MINOR_VERSION	0

extern struct snd_soc_codec *fauxsound_codec_ptr;

unsigned int sitar_read(struct snd_soc_codec *codec, unsigned int reg);
int sitar_write(struct snd_soc_codec *codec, unsigned int reg,
		unsigned int value);

static ssize_t mic_gain_show(struct kobject *kobj,
		struct kobj_attribute *attr, char *buf)
{
	return sprintf(buf, "%u",
		sitar_read(fauxsound_codec_ptr,
			SITAR_A_CDC_TX1_VOL_CTL_GAIN));
}

static ssize_t mic_gain_store(struct kobject *kobj,
		struct kobj_attribute *attr, const char *buf, size_t count)
{
	unsigned int lval;

	sscanf(buf, "%u", &lval);
		sitar_write(fauxsound_codec_ptr,
			SITAR_A_CDC_TX1_VOL_CTL_GAIN, lval);
		sitar_write(fauxsound_codec_ptr,
			SITAR_A_CDC_TX2_VOL_CTL_GAIN, lval);
	return count;

}

static ssize_t speaker_gain_show(struct kobject *kobj,
		struct kobj_attribute *attr, char *buf)
{
        return sprintf(buf, "%u",
			sitar_read(fauxsound_codec_ptr,
				SITAR_A_CDC_RX1_VOL_CTL_B2_CTL));

}

static ssize_t speaker_gain_store(struct kobject *kobj,
		struct kobj_attribute *attr, const char *buf, size_t count)
{
	unsigned int lval;

	sscanf(buf, "%u", &lval);
		sitar_write(fauxsound_codec_ptr,
			SITAR_A_CDC_RX1_VOL_CTL_B2_CTL, lval);
	return count;
}

static ssize_t headphone_gain_show(struct kobject *kobj,
		struct kobj_attribute *attr, char *buf)
{
	return sprintf(buf, "%u",
			sitar_read(fauxsound_codec_ptr,
				SITAR_A_CDC_RX2_VOL_CTL_B2_CTL));
}

static ssize_t headphone_gain_store(struct kobject *kobj,
		struct kobj_attribute *attr, const char *buf, size_t count)
{
	unsigned int lval;

	sscanf(buf, "%u", &lval);
		sitar_write(fauxsound_codec_ptr,
			SITAR_A_CDC_RX2_VOL_CTL_B2_CTL, lval);
		sitar_write(fauxsound_codec_ptr,
			SITAR_A_CDC_RX3_VOL_CTL_B2_CTL, lval);
	return count;
}

static ssize_t sound_control_version_show(struct kobject *kobj, struct kobj_attribute *attr, char *buf)
{
	return sprintf(buf, "version: %u.%u\n",
			SOUND_CONTROL_MAJOR_VERSION,
			SOUND_CONTROL_MINOR_VERSION);
}

static struct kobj_attribute mic_gain_attribute =
	__ATTR(gpl_mic_gain,
		0666,
		mic_gain_show,
		mic_gain_store);

static struct kobj_attribute speaker_gain_attribute =
	__ATTR(gpl_speaker_gain,
		0666,
		speaker_gain_show,
		speaker_gain_store);

static struct kobj_attribute headphone_gain_attribute =
	__ATTR(gpl_headphone_gain,
		0666,
		headphone_gain_show,
		headphone_gain_store);

static struct kobj_attribute sound_control_version_attribute =
	__ATTR(gpl_sound_control_version,
		0444,
		sound_control_version_show, NULL);

static struct attribute *sound_control_attrs[] =
	{
		&mic_gain_attribute.attr,
		&speaker_gain_attribute.attr,
		&headphone_gain_attribute.attr,
		&sound_control_version_attribute.attr,
		NULL,
	};

static struct attribute_group sound_control_attr_group =
	{
		.attrs = sound_control_attrs,
	};

static struct kobject *sound_control_kobj;

static int sound_control_init(void)
{
	int sysfs_result;

	sound_control_kobj =
		kobject_create_and_add("sound_control", kernel_kobj);

	if (!sound_control_kobj) {
		pr_err("%s sound_control_kobj create failed!\n",
			__FUNCTION__);
		return -ENOMEM;
        }

	sysfs_result = sysfs_create_group(sound_control_kobj,
			&sound_control_attr_group);

	if (sysfs_result) {
		pr_info("%s sysfs create failed!\n", __FUNCTION__);
		kobject_put(sound_control_kobj);
	}
	return sysfs_result;
}

static void sound_control_exit(void)
{
	if (sound_control_kobj != NULL)
		kobject_put(sound_control_kobj);
}

module_init(sound_control_init);
module_exit(sound_control_exit);
MODULE_LICENSE("GPL and additional rights");
MODULE_AUTHOR("Paul Reioux <reioux@gmail.com>");
MODULE_DESCRIPTION("Sound Control Module 3.x");
