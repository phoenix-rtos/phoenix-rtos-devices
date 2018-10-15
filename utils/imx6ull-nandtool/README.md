This tool allows to write raw, bootable image to NAND flash memory connected to i.MX 6ULL SoC.

Nandtool usage:

    -i (path) - file path (requires -s option)
    -r (path) - just like -i option but raw
    -s (number) - start flashing from page (requires -i option)
    -c - search for bad blocks from factory and print summary
    -h - print this message
    -t (number) - run test #no
    -e (start:end) - erase blocks form start to end
    -f (fw1) (fw2) (rootfs) - set flash for internal booting
