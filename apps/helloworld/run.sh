set -x
rm Release/bm/helloworld.elf*
EBBRT_SYSROOT=~/sysroot/native CMAKE_PREFIX_PATH=~/sysroot/hosted make -j Release
scp Release/bm/helloworld.elf32 headnode:/root/tftpbootlink/ebbrt.elf32
