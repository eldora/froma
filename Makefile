NAME = FROMA

CC = arm-none-eabi-gcc
LD = arm-none-eabi-gcc
OBJCOPY = arm-none-eabi-objcopy

DEFINES = 

LIBS = 

SOURCE_DIR = .
BUILD_DIR = build

C_OPTS =	-I. \
			-I"$(SOURCE_DIR)/source/include" \
			-I"$(SOURCE_DIR)/common/include" \
			-I"$(SOURCE_DIR)/kernel/include" \
			-I"$(SOURCE_DIR)/kernel/app" \
			-I"$(SOURCE_DIR)/kernel/board" \
			-I"$(SOURCE_DIR)/kernel/sync" \
			-std=c99 \
			-O3 \
			-Wall \
			-fmessage-length=0 \
			-mcpu=cortex-a9 \
			-g3 \
			-gdwarf-2

C_FILES =	source/croutine.c \
			source/list.c \
			source/queue.c \
			source/tasks.c \
			source/timers.c \
			kernel/main.c \
			kernel/app/apps.c \
			kernel/app/shell.c \
			kernel/board/port.c \
			kernel/mm/heap_2.c \
			kernel/sync/spinlock.c \
			kernel/uart/omap4_uart.c \
			kernel/uart/pl031_rtc.c \
			kernel/uart/printf-stdarg.c \
			kernel/uart/sp804_timer.c 

S_FILES =	kernel/startup.S

C_OBJS = $(C_FILES:%.c=$(BUILD_DIR)/%.o)

S_OBJS = $(S_FILES:%.S=$(BUILD_DIR)/%.o)

ALL_CFLAGS = $(C_OPTS) $(DEFINES) $(CFLAGS)
ALL_LDFLAGS_BASE =	$(LD_FLAGS) \
					-nostartfiles \
					-mcpu=cortex-a9 \
					-g3 \
					-gdwarf-2

ALL_LDFLAGS =	$(ALL_LDFLAGS_BASE) \
				-Wl,-T,FROMA.ld,-Map,FROMA.map

AUTODEPENDENCY_CFLAGS=-MMD -MF$(@:.o=.d) -MT$@




.SUFFIXES: .o .c .bin

all: $(NAME).uimg 

clean:
	rm -rf $(BUILD_DIR) $(NAME).elf $(NAME).bin $(NAME).uimg $(NAME).map

upload: $(NAME).bin 
	expect \
	-c 'set timeout -1' \
	-c 'spawn cu -l /dev/ttyUSB0 -s 115200' \
	-c 'expect "autoboot:"' \
	-c 'sleep 0.1' \
	-c 'send "\r"' \
	-c 'expect "Panda #"' \
	-c 'sleep 0.1' \
	-c 'send "loady 0x80000000\r"' \
	-c 'sleep 0.1'
	sz --ymodem $(NAME).bin </dev/ttyUSB0 >/dev/ttyUSB0
	expect \
	-c 'set timeout -1' \
	-c 'spawn cu -l /dev/ttyUSB0 -s 115200' \
	-c 'send "\r"' \
	-c 'expect "Panda #"' \
	-c 'sleep 0.1' \
	-c 'send "go 0x80000000\r"' \
	-c 'expect "!!!!"'
	-c 'sleep 1.0'

$(NAME).uimg: $(NAME).bin
	mkimage -A arm -O linux -T kernel -C none -a 0x80000000 -e 0x80000000 \
	-d $< -n FROMA $@

%.bin: %.elf
	$(OBJCOPY) -O binary $< $@

$(NAME).elf: $(C_OBJS) $(S_OBJS)
	$(LD) $(ALL_LDFLAGS) -o $@ $^ $(LIBS)

$(BUILD_DIR)/%.o: $(SOURCE_DIR)/%.c
	@mkdir -p $(dir $@)
	$(CC) $(ALL_CFLAGS) $(AUTODEPENDENCY_CFLAGS) -c $< -o $@

$(BUILD_DIR)/%.o: $(SOURCE_DIR)/%.S
	@mkdir -p $(dir $@)
	$(CC) $(ALL_CFLAGS) $(AUTODEPENDENCY_CFLAGS) -c $< -o $@

-include $(C_OBJS:.o=.d)

