###################################################################
# Output filename:
PROJECT		= btl18f2520

# Architecture:
ARCH		= pic16

# CPU type:
CPU			= 18f2520

# Source file:
SOURCE		= loader.asm

# Flags:
CFLAGS		= -C


PROJECT_PVR 	= 1

###################################################################

AS			= gpasm
GPUTILS_DIR=/usr/share/gputils

###################################################################
all: $(PROJECT).hex

###################################################################
$(PROJECT).hex: $(SOURCE)
	@echo "*** Compiling $(SOURCE)..."
	@$(AS) $(CFLAGS) -p$(CPU) -o $(PROJECT).hex $(SOURCE)
	@echo ";PIC$(CPU)" >> $(PROJECT).hex
	
	
###################################################################
clean:
	@$(RM) *.hex *.cof *.dasm *.lst *.o *.cod *.map
	@echo "*** Project directory cleaned."
	
.PHONY: clean all
