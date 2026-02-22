DIR_LIS		 = ./lis2dh12-pid
DIR_SRC      = ./app
DIR_BIN		 = ./bin
DIR_DUOS	 = ./duos

EPD = NULL
OBJ_C = $(wildcard  ${DIR_LIS}/*.c ${DIR_SRC}/*.c ${DIR_DUOS}/*.c)

OBJ_O = $(patsubst %.c,${DIR_BIN}/%.o,$(notdir ${OBJ_C}))
DUO_DEV_C = $(wildcard $(DIR_BIN)/dev_hardware_SPI.o $(DIR_BIN)/DUO_sysfs_gpio.o $(DIR_BIN)/DEV_Config.o)

#enable printf debug
#DEBUG = -D DEBUG

.PHONY : DUO clean

DUO: DUO_LIS 

TARGET = motion_detection
CC = musl-gcc
MSG = -ffunction-sections -fdata-sections -Wall
CFLAGS += $(MSG) -D $(EPD)

#optimized for debug debug
#debug_cflags=-g -O -ggdb3 -ggdb -g3 -ggdb2 -g1 -g0
#optimized for release
debug_cflags=-O3 -DNDEBUG

arch_cflags=-mcpu=c906fdv -mcmodel=medany
arch_ldflags=-D_LARGEFILE_SOURCE -D_LARGEFILE64_SOURCE -D_FILE_OFFSET_BITS=64
sys_lib=libs/system/musl_riscv64
sys_inc=include/system
CFLAGS+=${debug_cflags} -I${sys_inc}

LDFLAGS=${arch_ldflags} -L${sys_lib}
LDFLAGS += -Wl,-rpath,libs/system/musl_riscv64 -lwiringx


DUO_LIS:${OBJ_O}
	echo $(@)
	$(CC) $(CFLAGS) -D DUO $(OBJ_O) $(DUO_DEV_C) -o $(TARGET) $(DEBUG) $(LDFLAGS)

$(shell mkdir -p $(DIR_BIN))

${DIR_BIN}/%.o:$(DIR_SRC)/%.c
	$(CC) $(CFLAGS) -c	$< -o $@ -I $(DIR_LIS) -I $(DIR_DUOS) $(DEBUG) $(LDFLAGS)
	
${DIR_BIN}/%.o:$(DIR_LIS)/%.c
	$(CC) $(CFLAGS) -c	$< -o $@ $(DEBUG) $(LDFLAGS)

${DIR_BIN}/%.o:$(DIR_DUOS)/%.c
	$(CC) $(CFLAGS) -c	$< -o $@ -I $(DIR_SRC) $(DEBUG) -static
	
clean :
	rm $(DIR_BIN)/*.* 
	rm $(TARGET) 