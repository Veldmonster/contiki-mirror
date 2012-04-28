### This makefile is intended for use with
### Yagarto
LM3S9B96 = startup_gcc.c syscalls_gcc.c \
			clock.c delay.c eeprom.c flash.c rs232.c leds-arch.c \
			watchdog.c memory-nv.c mtarch.c rtimer-arch.c
### TARGETLIBS are platform-specific routines in the contiki library path
TARGETLIBS = random.c leds.c
CONTIKI_TARGET_SOURCEFILES += $(LM3S9B96) $(SENSORS) \
								$(SYSAPPS) \
								$(TARGETLIBS) \
								$(STELLARISWARE_DRIVERS)
CONTIKI_SOURCEFILES += $(CONTIKI_TARGET_SOURCEFILES)

### Compiler definitions
CC       = arm-none-eabi-gcc
LD       = arm-none-eabi-ld
AS       = arm-none-eabi-as
AR       = arm-none-eabi-ar
NM       = arm-none-eabi-nm
OBJCOPY  = arm-none-eabi-objcopy
STRIP    = arm-none-eabi-strip
SIZE	 = arm-none-eabi-size

ARCH_FLAGS= -mcpu=cortex-m3 -mthumb

# -std=gnu89 is avr libc default.  I am trying to do the same thing on arm.
CFLAGSNO = -I. \
			-I$(CONTIKI)/core \
			-I$(CONTIKI)/platform/$(TARGET) \
			-I$(STELLARISWARE_ROOT) \
			$(CONTIKI_PLAT_DEFS) \
			-Wall $(ARCH_FLAGS) -g3 -std=gnu89
			
CFLAGS += 	$(CFLAGSNO) -O$(OPTI)

# Suppress the following warnings
# It seems avr libc suppress these by default because,
# when compiling with WinAVR I get no warnings.
# If someone can shed some light, please let me know.
CFLAGS += 	-fno-strict-aliasing \
			-Wno-overflow \
			-Wno-uninitialized \
			-Wno-unused-variable \
			-Wno-unused-but-set-variable

LDFLAGS +=  $(ARCH_FLAGS) -lc -lm -lgcc \
			-Wl,--gc-sections -Xlinker -Map=contiki-$(TARGET).map
 
LDLIBS += -L$(STELLARISWARE_ROOT)/driverlib/gcc-cm3 -ldriver-cm3
LDLIBS += -L$(STELLARISWARE_ROOT)/grlib/gcc-cm3 -lgr-cm3
                               
### Compilation rules

#CUSTOM_RULE_C_TO_OBJECTDIR_O=yes
#CUSTOM_RULE_C_TO_O=yes
$(OBJECTDIR)/%.o: %.c
	$(CC) $(CFLAGS) -c $< -o $@

#CUSTOM_RULE_C_TO_OBJECTDIR_O=yes
%.o: %.c
	$(CC) $(CFLAGS) -c $< -o $@

%.ko: %.o
	$(STRIP) -K _init -K _fini --strip-unneeded -g -x $< -o $@

CUSTOM_RULE_LINK=yes
%.elf:	%.co $(PROJECT_OBJECTFILES) contiki-$(TARGET).a symbols.o
	$(CC) $(CFLAGS) $(LDFLAGS) -o $@ $(filter-out %.a,$^) $(filter %.a,$^) $(LDLIBS)
#Allow top-level makefile to always show size even when build is up to date
ifndef NOMAKEFILESIZE
	$(SIZE) -A $@
endif

%.bin: %.elf
	$(OBJCOPY) $^ -O binary $@

%.ihex: %.elf
	$(OBJCOPY) $^ -O ihex $@

# Add a namelist to the kernel
%.out: %.co $(PROJECT_OBJECTFILES) contiki-$(TARGET).a
	cp ${CONTIKI}/tools/empty-symbols.c symbols.c
	cp ${CONTIKI}/tools/empty-symbols.h symbols.h
	$(CC) $(LDFLAGS) $(CFLAGS) -o $@ $^ $(LIBC) symbols.c

%.eep: %.out
	-$(OBJCOPY) -j .eeprom --set-section-flags=.eeprom="alloc,load" \
	--change-section-lma .eeprom=0 -O ihex $^ $@

%-stripped.o: %.c
	$(CC) $(CFLAGS) -c $< -o $@
	$(STRIP) --strip-unneeded -g -x $@

#CUSTOM_RULE_C_TO_CO=yes
%.co: %.c
	cp ${CONTIKI}/tools/empty-symbols.c symbols.c
	cp ${CONTIKI}/tools/empty-symbols.h symbols.h
	$(CC) $(CFLAGS) -DAUTOSTART_ENABLE -c $< -o $@

%-stripped.o: %.o
	$(STRIP) --strip-unneeded -g -x -o $@ $<

#CUSTOM_RULE_S_TO_OBJECTDIR_O=yes
%.o: ${CONTIKI_TARGET}/loader/%.S
	$(AS) -o $(notdir $(<:.S=.o)) $<

%.srec: %.$(TARGET)
	$(OBJCOPY) -O srec $< $@

symbols.c:
	cp ${CONTIKI}/tools/empty-symbols.c symbols.c
	cp ${CONTIKI}/tools/empty-symbols.h symbols.h
