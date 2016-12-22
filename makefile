# Project name
PROJ_NAME = stir

# Source files to add to the build
C_FILES = \
main.c 

# Assembly files to add to the buid
ASM_FILES = 

# Project folders
INC_PATH = \
inc

SRC_PATH = src
OBJ_PATH = obj
OUT_PATH = out
RES_PATH = res

# Placeholders for executables
CC = avr-gcc
LD = avr-gcc
AS = avr-as
OD = avr-objdump
CP = avr-objcopy
SZ = avr-size
 
# Build flags
CC_FLAGS = $(addprefix -I./, $(INC_PATH)) -Wall -Wextra -Wno-main -std=gnu99 -pedantic -O3 -c -g \
							-mmcu=attiny85 -DF_CPU=16000000u
LD_FLAGS = -mmcu=attiny85 -DF_CPU=16000000u
AS_FLAGS = 
OD_FLAGS = -S
CP_FLAGS = -O ihex 
SZ_FLAGS = 
				 
# Make a list with all required objects
C_OBJ_LIST = $(C_FILES:.c=.o)
ASM_OBJ_LIST = $(ASM_FILES:.s=.o)

# Add paths to objects
C_OBJ = $(patsubst %, $(OBJ_PATH)/%, $(C_OBJ_LIST))
ASM_OBJ = $(patsubst %, $(OBJ_PATH)/%, $(ASM_OBJ_LIST))

# Generic make all o's from c files
$(OBJ_PATH)/%.o: $(SRC_PATH)/%.c
	@echo -n Compiling $<...
	$(CC) $(CC_FLAGS) -c $< -o $@
	@echo [Done]

# Generic make all o's from asm files
$(OBJ_PATH)/%.o: $(SRC_PATH)/%.s
	@echo -n Assembling $<...
	@$(AS) $(AS_FLAGS) -o $@ $<	
	@echo [Done]

# Do all operations including flashing
all: build flash

# Do only a rebuild
build: clean rebuild size


# Build obj files and link
rebuild: make_c make_asm link disasm make_bin

# Invoke linker
link:
	@echo -n Linking files...
	@$(LD) $(LD_FLAGS) -o $(OUT_PATH)/$(PROJ_NAME).elf $(C_OBJ) $(ASM_OBJ)
	@echo [Done]

# Invoke compiler	
make_c: $(C_OBJ)
		
# Invoke assembler
make_asm: $(ASM_OBJ)

# Generate binary
make_bin:
	@echo -n Making binary...
	@$(CP) $(CP_FLAGS) $(OUT_PATH)/$(PROJ_NAME).elf $(OUT_PATH)/$(PROJ_NAME).hex
	@echo [Done]

# Run disassembler on the elf file
disasm:	
	@echo -n Disassembling source...
	@$(OD) $(OD_FLAGS) $(OUT_PATH)/$(PROJ_NAME).elf > $(OUT_PATH)/$(PROJ_NAME).lst
	@echo [Done]

# Write flash contents to target
flash:
	@echo -n Flashing target...
	#@avrdude -p t85 -P COM6 -c avr910 -B 10 -b 1200 -e -U flash:w:$(OUT_PATH)/$(PROJ_NAME).hex -v
	@echo [Done] 	
	
# Print size
size:
	@echo -n Computing size...
	@$(SZ) $(SZ_FLAGS) $(OUT_PATH)/$(PROJ_NAME).elf
	@echo [Done] 	

# Clean existing obj files
clean:
	@echo -n Cleaning existing files...
	@rm -f $(OBJ_PATH)/*
	@rm -f $(OUT_PATH)/*
	@echo [Done]
	