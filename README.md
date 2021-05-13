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

