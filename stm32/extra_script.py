Import("env")

# The stm32cube build script constructs LINKFLAGS independently of
# build_flags, so -mfpu/-mfloat-abi set there never reaches the final
# link step even though it reaches every compile step. Without this,
# ld fails with "uses VFP register arguments, firmware.elf does not"
# because the FreeRTOS ARM_CM4F port (and every HAL object compiled
# hard-float) disagrees with the link step's implicit soft-float ABI.
env.Append(
    LINKFLAGS=["-mfpu=fpv4-sp-d16", "-mfloat-abi=hard"]
)
