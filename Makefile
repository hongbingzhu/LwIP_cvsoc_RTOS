# ===============================================================
# 1. Global Variables
# ===============================================================
#CROSS_COMPILE := arm-altera-eabi-
#CC = $(CROSS_COMPILE)gcc 
#AR = $(CROSS_COMPILE)ar
#OBJCOPY = $(CROSS_COMPILE)objcopy
CC = armcc
AS = armasm
LD = armlink
#ALT_LIB := P:/altera/14.1/embedded/ip/altera/hps/altera_hps/hwlib
ALT_LIB := hwlib

HOSTOS := $(shell uname -o 2>/dev/null | tr [:upper:] [:lower:])
ifeq ($(HOSTOS),cygwin)
TOPDIR := $(shell cygpath -m $(shell pwd))
else
TOPDIR := $(shell pwd)
endif
export CC TOPDIR

PRJ_NAME := ucos_lwip

# ===============================================================
# 2. CFLAGS
# ===============================================================
# -nostdinc: Do not search the standard system directories
# for header files. Only the directories you have specified
# with -I options(and the directory of the current file, if
# appropriate) are searched.
#
# data-sections: Place each function or data item into its own
# section in the output file. this could cause ld script
# bugs(*(.bss) would be *(.bss*)). 
#
# -isystem $(TOPDIR): variables used in a rule could not contain
#  any colon in make-3.82 on cygwin. so we use relative path.
#
# -std=gnu99 -fgnu89-inline: not only support gnu c99
#  extensions(like asm), but also support c89 inline functions.
# ===============================================================
CFLAGS := --cpu=Cortex-A9 --no_unaligned_access -O0 -g --md --c99 --depend_format=unix_escaped
ASFLAGS := --cpu=Cortex-A9 --no_unaligned_access -g --depend_format=unix_escaped
LDFLAGS := --cpu=Cortex-A9 --entry=Start --scatter=""BSP/ARM_Compiler/scatter.scat"" --info=sizes
#CFLAGS := -g -O0 -mfpu=neon -march=armv7-a -mtune=cortex-a9 -mcpu=cortex-a9 -std=c99 \
#	-mno-unaligned-access -ffunction-sections -fdata-sections -Werror -Wstrict-prototypes
#ASFLAGS := -mfpu=neon -mcpu=cortex-a9
#LDFLAGS := -Wl,--gc-sections -Xlinker -Map -Xlinker $(PRJ_NAME).map -T $(LINKER_SCRIPT)

# ===============================================================
# 3. Objects and Includes
# ===============================================================

# pulic includes, for configure file
PUB_INCS := APP/cfg
ASM_OBJS :=

# uC/OS objects
OS_DIR := OS
OS_VER := uCOS-II

# uC/OS-II or uC/OS-III objects, $(OS_VER)/Source needby uC/OS-II
OS_INCS := uC-Common uC-Common/Auth uC-Common/KAL 
OS_INCS += uC-CPU uC-CPU/ARM-Cortex-A/RealView uC-LIB
OS_INCS += $(OS_VER) $(OS_VER)/Source $(OS_VER)/Ports/ARM-Cortex-A/Generic/RealView
OS_INCS := $(addprefix $(OS_DIR)/,$(OS_INCS)) $(PUB_INCS)

OS_SRCS := $(wildcard $(OS_DIR)/$(OS_VER)/Source/os_*.c)
OS_SRCS += $(OS_DIR)/$(OS_VER)/Ports/ARM-Cortex-A/Generic/RealView/os_cpu_c.c
ASM_SRCS += $(OS_DIR)/$(OS_VER)/Ports/ARM-Cortex-A/Generic/RealView/os_cpu_a_vfp-d32.s
OS_SRCS += $(addprefix $(OS_DIR)/uC-LIB/,lib_ascii.c lib_math.c lib_mem.c lib_str.c)
OS_SRCS += $(OS_DIR)/uC-Common/KAL/$(OS_VER)/kal.c
OS_SRCS += $(OS_DIR)/uC-CPU/Cache/ARM/armv7_generic_l1_l2c310_l2/cpu_cache_armv7_generic_l1_l2c310_l2.c
ASM_SRCS += $(OS_DIR)/uC-CPU/Cache/ARM/armv7_generic_l1_l2c310_l2/RealView/cpu_cache_armv7_generic_l1_l2c310_l2_a.s
ASM_SRCS += $(OS_DIR)/uC-CPU/ARM-Cortex-A/RealView/cpu_a.s
OS_SRCS += $(OS_DIR)/uC-CPU/cpu_core.c
OS_OBJS := $(OS_SRCS:.c=.o)

# lwip objects
LWIP_DIR := lwip-1.4.1

LWIP_INCS := $(LWIP_DIR)/src/include $(LWIP_DIR)/src/include/ipv4 $(PUB_INCS) \
	$(LWIP_DIR)/ports/CycloneV $(LWIP_DIR)/ports/CycloneV/Standalone \
	$(ALT_LIB)/include $(ALT_LIB)/include/socal

LWIP_CORE_OBJS := def.o dhcp.o init.o mem.o memp.o netif.o pbuf.o raw.o stats.o sys.o \
	tcp.o tcp_in.o tcp_out.o timers.o udp.o
LWIP_CORE_OBJS := $(addprefix $(LWIP_DIR)/src/core/,$(LWIP_CORE_OBJS))

LWIP_IPV4_OBJS := autoip.o icmp.o igmp.o inet.o inet_chksum.o ip.o ip_addr.o ip_frag.o
LWIP_IPV4_OBJS := $(addprefix $(LWIP_DIR)/src/core/ipv4/,$(LWIP_IPV4_OBJS))

LWIP_API_OBJS := api_lib.o api_msg.o err.o netbuf.o netdb.o netifapi.o sockets.o tcpip.o
LWIP_API_OBJS := $(addprefix $(LWIP_DIR)/src/api/,$(LWIP_API_OBJS))

LWIP_PORT_OBJS := $(LWIP_DIR)/src/netif/etharp.o

LWIP_OBJS := $(LWIP_CORE_OBJS) $(LWIP_IPV4_OBJS) $(LWIP_API_OBJS) $(LWIP_PORT_OBJS)

# application objects
APP_INCS := APP BSP BSP/OS $(OS_INCS) $(LWIP_INCS)

ALT_SRCS := $(addprefix APP/,alt_16550_uart.c alt_clock_manager.c alt_generalpurpose_io.c)
APP_SRCS := $(addprefix BSP/,OS/bsp_os.c bsp_int.c bsp.c cpu_bsp.c)
ASM_SRCS += $(addprefix BSP/ARM_Compiler/,armv7a_tthelp.s bsp_cache.s startup.s)
APP_SRCS += $(addprefix APP/,app.c alt_eth_dma.c alt_ethernet.c \
	fs.c httpd_cgi_ssi.c httpd.c httpserver.c NetAddr.c)
APP_SRCS += $(ALT_SRCS) $(LWIP_DIR)/ports/CycloneV/Standalone/ethernetif.c
APP_OBJS := $(APP_SRCS:.c=.o)
APP_MIN_OBJS := $(addprefix BSP/,OS/bsp_os.o bsp_int.o bsp.o cpu_bsp.o) APP/app.o

ASM_OBJS := $(ASM_SRCS:.s=.o)

# all objects
OBJS := $(ASM_OBJS) $(OS_OBJS) $(LWIP_OBJS) $(APP_OBJS)
#OBJS := $(ASM_OBJS) $(OS_OBJS) $(APP_MIN_OBJS)

# ===============================================================
# 4. Rules
# ===============================================================
.PHONY: all clean distclean test to/timestamp.h

all: $(PRJ_NAME).axf 

$(PRJ_NAME).axf: $(OBJS)
	@echo Linking objs to $@
	@$(LD) $(LDFLAGS) -o $@ $^
#	@$(OBJCOPY) -I elf32-little -O binary $@ $(PRJ_NAME).bin
#	mkimage.exe -A arm -T firmware -C none -O openbsd -a 0x0020a000 -e 0x0020a000 -d $(PRJ_NAME).bin coreboard.dat
#	CPackageFW_A+.exe
 
src/timestamp.h:
	@echo Generate time stamp file
	@date +'#define COMPILE_DATE "%m %d %C%y"' > $@
	@date +'#define COMPILE_TIME "%T"' >> $@

%.o: %.s
	@echo Compiling assembly source $<
	@$(AS) $(ASFLAGS) -o $@ $<

$(OS_OBJS): %.o: %.c
	@echo Compiling $(OS_VER) source $<
	@$(CC) $(OS_INCS:%=-I%) $(CFLAGS) --depend=$(subst .o,.d,$@) -c -o $@ $<

$(LWIP_OBJS): %.o: %.c
	@echo Compiling lwip source $<
	@$(CC) $(LWIP_INCS:%=-I%) -IAPP $(CFLAGS) --depend=$(subst .o,.d,$@) -c -o $@ $<
	
$(APP_OBJS): %.o: %.c
	@echo Compiling application source $<
	@$(CC) $(APP_INCS:%=-I%) $(CFLAGS) --depend=$(subst .o,.d,$@) -c -o $@ $<

$(ALT_SRCS): %.c:
	cp $(ALT_LIB)/src/hwmgr/$(notdir $@) $@

deps := $(OBJS:.o=.d)

test:
	@echo $(deps)

clean:
	@rm -f $(PRJ_NAME).axf $(OBJS)

distclean:
	@rm -f $(PRJ_NAME).axf $(OBJS) $(deps)

# ===============================================================
# 5. Depends
# ===============================================================
-include $(deps)

