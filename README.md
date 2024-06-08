# PD-PPS VariablePowerSupply
詳しくは[USB PD PPSを使用した小型可変電源「mini PD-PPS VariablePowerSupply」](https://unagidojyou.com/2024/06-08/mini_pd-pps_variablepowersupply/)  <p>
Draw any(3.3~21) voltage you want using USB PD PPS.  
* PPS mode : Can be set 3.3~21V in 0.1V increments
* Fix mode : If PPS is not supported, Fix mode(5,9,12,15,20V) can be used
* Compact : only 1.5x4.5cm
* Voltmeter
* Ammeter
* Output ON/OFF

# hardware
Under construction

# software
## Compile
The GNU Compiler Collection for RISC-V on Linux.
## Write
1. Temporarily place 0 Ω on R1 and R2.  

2. Use [WCHISPTool](https://www.wch.cn/downloads/WCHISPTool_Setup_exe.html) on Windows.  
Need to set  
RST multiplexing is an extenal pin reset [Disable mul-func,RST is usedforlOfunction]

3. Upload Hex file.

4. Calibration is required after flashing.

# License
The software in this repository is based on wagiminator's  
* Development-Boards  
[wagiminator/Development-Boards/CH32X035F7P6_DevBoard](https://github.com/wagiminator/Development-Boards/tree/main/CH32X035F7P6_DevBoard)  
* MCU Templates  
[wagiminator/MCU-Templates/CH32X035](https://github.com/wagiminator/MCU-Templates/tree/main/CH32X035)  

![license.png](https://i.creativecommons.org/l/by-sa/3.0/88x31.png)

This work is licensed under Creative Commons Attribution-ShareAlike 3.0 Unported License. 
(http://creativecommons.org/licenses/by-sa/3.0/)
