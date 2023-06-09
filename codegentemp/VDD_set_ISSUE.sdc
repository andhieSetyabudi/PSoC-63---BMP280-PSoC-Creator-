# THIS FILE IS AUTOMATICALLY GENERATED
# Project: C:\Users\DwiSetyabudi\Documents\PSoC Creator\User_VDD\VDD_set_ISSUE.cydsn\VDD_set_ISSUE.cyprj
# Date: Sat, 03 Jun 2023 06:13:36 GMT
#set_units -time ns
create_clock -name {CyILO} -period 31250 -waveform {0 15625} [list [get_pins {ClockBlock/ilo}]]
create_clock -name {CyClk_LF} -period 31250 -waveform {0 15625} [list [get_pins {ClockBlock/lfclk}]]
create_clock -name {CyClk_HF0} -period 125 -waveform {0 62.5} [list [get_pins {ClockBlock/hfclk_0}]]
create_clock -name {CyClk_Fast} -period 125 -waveform {0 62.5} [list [get_pins {ClockBlock/fastclk}]]
create_clock -name {CyClk_Peri} -period 125 -waveform {0 62.5} [list [get_pins {ClockBlock/periclk}]]
create_generated_clock -name {CyClk_Slow} -source [get_pins {ClockBlock/periclk}] -edges {1 2 3} [list [get_pins {ClockBlock/slowclk}]]
create_generated_clock -name {UART_SCBCLK} -source [get_pins {ClockBlock/periclk}] -edges {1 7 13} [list [get_pins {ClockBlock/ff_div_5}]]
create_generated_clock -name {I2C_SCBCLK} -source [get_pins {ClockBlock/periclk}] -edges {1 5 11} [list [get_pins {ClockBlock/ff_div_6}]]
create_clock -name {CyPeriClk_App} -period 125 -waveform {0 62.5} [list [get_pins {ClockBlock/periclk_App}]]
create_clock -name {CyFLL} -period 20 -waveform {0 10} [list [get_pins {ClockBlock/fll}]]
create_clock -name {CyIMO} -period 125 -waveform {0 62.5} [list [get_pins {ClockBlock/imo}]]


# Component constraints for C:\Users\DwiSetyabudi\Documents\PSoC Creator\User_VDD\VDD_set_ISSUE.cydsn\TopDesign\TopDesign.cysch
# Project: C:\Users\DwiSetyabudi\Documents\PSoC Creator\User_VDD\VDD_set_ISSUE.cydsn\VDD_set_ISSUE.cyprj
# Date: Sat, 03 Jun 2023 06:13:16 GMT
