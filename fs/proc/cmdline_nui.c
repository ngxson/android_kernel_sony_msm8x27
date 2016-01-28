#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/proc_fs.h>
#include <linux/string.h>
#include <linux/vmalloc.h>
#include <linux/init.h>
#include <asm/uaccess.h>

#define MAX_LEN       1024
int read_info( char *page, char **start, off_t off,int count, int *eof, void *data );
ssize_t write_info( struct file *filp, const char __user *buff,unsigned long len, void *data );

static struct proc_dir_entry *proc_entry;
static char *info;
static int write_index;
static int read_index;

static int __init proc_cmdline_nui_init(void)
{
    int ret = 0;
    info = (char *)vmalloc( MAX_LEN );
    memset( info, 0, MAX_LEN );
    strcpy (info, saved_command_line);
    proc_entry = create_proc_entry( "cmdline", 0666, NULL );

    if (proc_entry == NULL)
    {
        ret = -1;
        vfree(info);
        printk(KERN_INFO "ngxson: cmdline could not be created\n");
    }
    else
    {
        write_index = 0;
        read_index = 0;
        proc_entry->read_proc = read_info;
        proc_entry->write_proc = write_info;
        printk(KERN_INFO "ngxson: cmdline created = %s\n", info);
    }

    return ret;
}

ssize_t write_info( struct file *filp, const char __user *buff, unsigned long len, void *data )
{
    int capacity = (MAX_LEN-write_index)+1;
    if (len > capacity)
    {
        printk(KERN_INFO "ngxson: No space to write in cmdline!\n");
        return -1;
    }
    if (copy_from_user( &info[write_index], buff, len ))
    {
        return -2;
    }

    write_index += len;
    info[write_index-1] = 0;
    return len;
}

int read_info( char *page, char **start, off_t off, int count, int *eof, void *data )
{
    int len;
    if (off > 0)
    {
        *eof = 1;
        return 0;
    }

    if (read_index >= write_index)
    read_index = 0;

    len = sprintf(page, "%s\n", &info[read_index]);
    read_index += len;
    return len;
}

module_init(proc_cmdline_nui_init);
