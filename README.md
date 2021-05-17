- known issue


```
[step] to compile dss project  

"../dss_mrr_linker.cmd", line 92: warning #10068-D: no matching section

[how to sol.]
Disable it
```

```
[step] to compile dss project  

warning #10370-D: Possible codesize or performance degradation. Section ".text:SOC_init:libsoc_xwr18xx.ae674<soc.oe674>" has calls to rts routines, but rts is placed out of range from call site at 0x20003010, or in a different section. To optimize codesize, either 1) place rts closer to call site, or 2) place rts in same section, or 3) compile with --disable_push_pop.
warning #10370-D: Possible codesize or performance degradation. Section ".text:SOC_init:libsoc_xwr18xx.ae674<soc.oe674>" has calls to rts routines, but rts is placed out of range from call site at 0x20002ee0, or in a different section. To optimize codesize, either 1) place rts closer to call site, or 2) place rts in same section, or 3) compile with --disable_push_pop.

[how to sol.]
https://e2e.ti.com/support/sensors-group/sensors/f/sensors-forum/638908/awr1642boost-compiling-warming-for-the-mmwave-sdk-1-1-0-2

https://e2e.ti.com/support/tools/code-composer-studio-group/ccs/f/code-composer-studio-forum/431202/meaning-of---disable_push_pop-option-for-66x-dsp-compiler

```

```
[C674X_0] Heap L1 : size 16384 (0x4000), free 6144 (0x1800)
Heap L3 : size 1048576 (0x100000), free 0 (0x0)
Heap L1 : size 16384 (0x4000), free 1000 (0x3e8)
Heap L3 : size 1048576 (0x100000), free 262144 (0x40000)
Heap L2 : size 102400 (0x19000), free 21488 (0x53f0)
Heap L2 : size 102400 (0x19000), free 14432 (0x3860)
[Cortex_R4_0] Debug: Launched the Initialization Task
Debug: Initialized the mmWave module
Debug: Synchronized the mmWave module
Error: MMWDemoMSS mmWave open configuration failed [Error code -204406306]
Error: Please ensure that the XXXCfg CLI command is invoked before starting the sensor

Error code 3119 -> 3100 + 0019
#define MMWAVE_ERRNO_BASE                    (-3100)
/**
 * @brief   Error Code: mmWave link BSS calibration configuration failed
 */
#define MMWAVE_ECALCFG                  (MMWAVE_ERRNO_BASE-19)
```