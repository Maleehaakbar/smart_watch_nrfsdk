
1. in dts, for flash partition , logical addresses are used , 
bootloader from address zero, slot 0 from 0xc000 , slot 1 from 0x82000
the physical addresses are calculated using flash_map APIs

2. slot 0 and 1 must have same size

3. BY transferring secondary slot to external flash, active image executes 
  from interna flash and have more memory space.

  as with 1MB of flash , we haveapproximately 48kb for mcu boot , 498 kb for both slot0,1
  with external flash, we have 64jb for mcu boot , and 928 kb for bith slots


  warnings todos 
  CMake Warning at CMakeLists.txt:612 (message):
  Unable to determine erase size of slot1 partition, cannot calculate minimum
  sector usage