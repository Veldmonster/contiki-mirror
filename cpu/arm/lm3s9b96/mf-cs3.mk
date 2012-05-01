### This makefile is intended for use with
### CodeSourcery Personal for Stellaris (paid version)
LM3S9B96 += syscalls_cs3.c \
			clock.c delay.c eeprom.c flash.c rs232.c \
			leds-arch.c watchdog.c memory-nv.c mtarch.c rtimer-arch.c uip-ipchksum.c
### TARGETLIBS are platform-specific routines in the contiki library path
TARGETLIBS = random.c leds.c
UIPDRIVERS = me.c me_tabs.c slip.c crc16.c
CONTIKI_TARGET_SOURCEFILES += $(LM3S9B96) $(SENSORS) \
								$(SYSAPPS) \
								$(TARGETLIBS) \
								$(UIPDRIVERS) \
								$(STELLARISWARE_DRIVERS)
CONTIKI_SOURCEFILES += $(CONTIKI_TARGET_SOURCEFILES)
			
PROJECT_OBJECTFILES += ${addprefix $(OBJECTDIR)/,$(CONTIKI_TARGET_MAIN:.c=.o)}

### Compiler definitions
CC       = arm-stellaris-eabi-gcc
LD       = arm-stellaris-eabi-ld
AS       = arm-stellaris-eabi-as
AR       = arm-stellaris-eabi-ar
NM       = arm-stellaris-eabi-nm
OBJCOPY  = arm-stellaris-eabi-objcopy
STRIP    = arm-stellaris-eabi-strip
SIZE	 = arm-stellaris-eabi-size

ARCH_FLAGS= -mcpu=cortex-m3 -mthumb

CFLAGSNO = -I. \
			-I$(CONTIKI)/core \
			-I$(CONTIKI)/platform/$(TARGET) \
			-I$(STELLARISWARE_ROOT) \
			$(CONTIKI_PLAT_DEFS) \
			-Wall $(ARCH_FLAGS) -g3 -std=c99
			
CFLAGS += 	$(CFLAGSNO) -O$(OPTI)

# Suppress the following warnings
# It seems avr libc suppress these by default because,
# when compiling with WinAVR I get no warnings.
# If someone can shed some light, please let me know.
CFLAGS += 	-fno-strict-aliasing \
			-Wno-overflow \
			-Wno-uninitialized \
			-Wno-unused-variable \
			-Wno-unused-but-set-variable \
			-Wno-char-subscripts \
			-Wno-unused-function \
			-Wno-pointer-sign

LDFLAGS +=  $(ARCH_FLAGS) -lm \
			-Wl,--gc-sections -Xlinker -Map=contiki-$(TARGET).map \
			-T $(CONTIKI_CPU)/lm3s9b96-cs3.ld
 
LDLIBS += -L$(STELLARISWARE_ROOT)/driverlib/gcc-cm3 -ldriver-cm3
LDLIBS += -L$(STELLARISWARE_ROOT)/grlib/gcc-cm3 -lgr-cm3
                               
### Compilation rules

CUSTOM_RULE_C_TO_OBJECTDIR_O=yes
$(OBJECTDIR)/%.o: %.c
	$(CC) $(CFLAGS) -c $< -o $@

#CUSTOM_RULE_C_TO_O=yes
#%.o: %.c
#	$(CC) $(CFLAGS) -c $< -o $@

#%.ko: %.o
#	$(STRIP) -K _init -K _fini --strip-unneeded -g -x $< -o $@

CUSTOM_RULE_LINK=yes
%.elf: %.co $(PROJECT_OBJECTFILES) $(PROJECT_LIBRARIES) contiki-$(TARGET).a syscalls_cs3.o
	$(CC) $(CFLAGS) $(LDFLAGS) ${filter-out %.a,$^} ${filter %.a,$^} $(LDLIBS) -o $@
#Allow top-level makefile to always show size even when build is up to date
ifndef NOMAKEFILESIZE
	$(SIZE) -A $@
endif

#CUSTOM_RULE_LINK=yes
#%.elf:	%.co $(PROJECT_OBJECTFILES) contiki-$(TARGET).a syscalls_cs3.o
#	$(CC) $(CFLAGS) $(LDFLAGS) -o $@ $(filter-out %.a,$^) $(filter %.a,$^) $(LDLIBS)
#Allow top-level makefile to always show size even when build is up to date
#ifndef NOMAKEFILESIZE
#	$(SIZE) -A $@
#endif

%.bin: %.elf
	$(OBJCOPY) $^ $@ -O binary

%.ihex: %.elf
	$(OBJCOPY) $^ $@ -O ihex

# Add a namelist to the kernel
#%.out: %.co $(PROJECT_OBJECTFILES) contiki-$(TARGET).a
#	$(CC) $(LDFLAGS) $(CFLAGS) -o $@ $^ $(LIBC)

#%.eep: %.out
#	-$(OBJCOPY) -j .eeprom --set-section-flags=.eeprom="alloc,load" \
#	--change-section-lma .eeprom=0 -O ihex $^ $@

#%-stripped.o: %.c
#	$(CC) $(CFLAGS) -c $< -o $@
#	$(STRIP) --strip-unneeded -g -x $@

#CUSTOM_RULE_C_TO_CO=yes
#%.co: %.c
#	$(CC) $(CFLAGS) -DAUTOSTART_ENABLE -c $< -o $@

#%-stripped.o: %.o
#	$(STRIP) --strip-unneeded -g -x -o $@ $<

#CUSTOM_RULE_S_TO_OBJECTDIR_O=yes
#%.o: ${CONTIKI_TARGET}/loader/%.S
#	$(AS) -o $(notdir $(<:.S=.o)) $<

#%.srec: %.$(TARGET)
#	$(OBJCOPY) -O srec $< $@

#symbols.c:
#	cp ${CONTIKI}/tools/empty-symbols.c symbols.c
#	cp ${CONTIKI}/tools/empty-symbols.h symbols.h
