
�
�No debug cores found in the current design.
Before running the implement_debug_core command, either use the Set Up Debug wizard (GUI mode)
or use the create_debug_core and connect_debug_core Tcl commands to insert debug cores into the design.
154*	chipscopeZ16-241h px� 
Q
Command: %s
53*	vivadotcl2 
place_design2default:defaultZ4-113h px� 
�
@Attempting to get a license for feature '%s' and/or device '%s'
308*common2"
Implementation2default:default2
xc7a100t2default:defaultZ17-347h px� 
�
0Got license for feature '%s' and/or device '%s'
310*common2"
Implementation2default:default2
xc7a100t2default:defaultZ17-349h px� 
P
Running DRC with %s threads
24*drc2
82default:defaultZ23-27h px� 
V
DRC finished with %s
79*	vivadotcl2
0 Errors2default:defaultZ4-198h px� 
e
BPlease refer to the DRC report (report_drc) for more information.
80*	vivadotclZ4-199h px� 
p
,Running DRC as a precondition to command %s
22*	vivadotcl2 
place_design2default:defaultZ4-22h px� 
P
Running DRC with %s threads
24*drc2
82default:defaultZ23-27h px� 
V
DRC finished with %s
79*	vivadotcl2
0 Errors2default:defaultZ4-198h px� 
e
BPlease refer to the DRC report (report_drc) for more information.
80*	vivadotclZ4-199h px� 
U

Starting %s Task
103*constraints2
Placer2default:defaultZ18-103h px� 
}
BMultithreading enabled for place_design using a maximum of %s CPUs12*	placeflow2
82default:defaultZ30-611h px� 
v

Phase %s%s
101*constraints2
1 2default:default2)
Placer Initialization2default:defaultZ18-101h px� 
�
r%sTime (s): cpu = %s ; elapsed = %s . Memory (MB): peak = %s ; gain = %s ; free physical = %s ; free virtual = %s
480*common2.
Netlist sorting complete. 2default:default2
00:00:002default:default2
00:00:002default:default2
1856.0742default:default2
0.0002default:default2
9342default:default2
125502default:defaultZ17-722h px� 
�
r%sTime (s): cpu = %s ; elapsed = %s . Memory (MB): peak = %s ; gain = %s ; free physical = %s ; free virtual = %s
480*common2.
Netlist sorting complete. 2default:default2
00:00:002default:default2
00:00:002default:default2
1856.0742default:default2
0.0002default:default2
9342default:default2
125502default:defaultZ17-722h px� 
�

Phase %s%s
101*constraints2
1.1 2default:default2F
2IO Placement/ Clock Placement/ Build Placer Device2default:defaultZ18-101h px� 
�

Phase %s%s
101*constraints2
1.1.1 2default:default22
ParallelPlaceIOClockAndInitTop2default:defaultZ18-101h px� 
v

Phase %s%s
101*constraints2
1.1.1.1 2default:default2#
Pre-Place Cells2default:defaultZ18-101h px� 
H
3Phase 1.1.1.1 Pre-Place Cells | Checksum: 8e694001
*commonh px� 
�

%s
*constraints2�
�Time (s): cpu = 00:00:00.02 ; elapsed = 00:00:00.02 . Memory (MB): peak = 1856.074 ; gain = 0.000 ; free physical = 934 ; free virtual = 125502default:defaulth px� 
�

Phase %s%s
101*constraints2
1.1.1.2 2default:default2/
Constructing HAPIClkRuleMgr2default:defaultZ18-101h px� 
T
?Phase 1.1.1.2 Constructing HAPIClkRuleMgr | Checksum: 8e694001
*commonh px� 
�

%s
*constraints2�
�Time (s): cpu = 00:00:00.03 ; elapsed = 00:00:00.04 . Memory (MB): peak = 1856.074 ; gain = 0.000 ; free physical = 934 ; free virtual = 125502default:defaulth px� 
}

Phase %s%s
101*constraints2
1.1.1.5 2default:default2*
IOLockPlacementChecker2default:defaultZ18-101h px� 


Phase %s%s
101*constraints2
1.1.1.6 2default:default2,
IOBufferPlacementChecker2default:defaultZ18-101h px� 
q

Phase %s%s
101*constraints2
1.1.1.3 2default:default2

DSPChecker2default:defaultZ18-101h px� 
�

Phase %s%s
101*constraints2
1.1.1.4 2default:default2/
ClockRegionPlacementChecker2default:defaultZ18-101h px� 
C
.Phase 1.1.1.3 DSPChecker | Checksum: 8e694001
*commonh px� 
�

%s
*constraints2�
�Time (s): cpu = 00:00:00.26 ; elapsed = 00:00:00.11 . Memory (MB): peak = 1872.082 ; gain = 16.008 ; free physical = 933 ; free virtual = 125502default:defaulth px� 
y

Phase %s%s
101*constraints2
1.1.1.7 2default:default2&
V7IOVoltageChecker2default:defaultZ18-101h px� 
K
6Phase 1.1.1.7 V7IOVoltageChecker | Checksum: 8e694001
*commonh px� 
�

%s
*constraints2�
�Time (s): cpu = 00:00:00.26 ; elapsed = 00:00:00.11 . Memory (MB): peak = 1872.082 ; gain = 16.008 ; free physical = 933 ; free virtual = 125502default:defaulth px� 
�

Phase %s%s
101*constraints2
1.1.1.8 2default:default2-
OverlappingPBlocksChecker2default:defaultZ18-101h px� 
Q
<Phase 1.1.1.6 IOBufferPlacementChecker | Checksum: 8e694001
*commonh px� 
�

%s
*constraints2�
�Time (s): cpu = 00:00:00.27 ; elapsed = 00:00:00.11 . Memory (MB): peak = 1872.082 ; gain = 16.008 ; free physical = 933 ; free virtual = 125502default:defaulth px� 
v

Phase %s%s
101*constraints2
1.1.1.9 2default:default2#
DisallowedInsts2default:defaultZ18-101h px� 
R
=Phase 1.1.1.8 OverlappingPBlocksChecker | Checksum: 8e694001
*commonh px� 
�

%s
*constraints2�
�Time (s): cpu = 00:00:00.27 ; elapsed = 00:00:00.11 . Memory (MB): peak = 1872.082 ; gain = 16.008 ; free physical = 933 ; free virtual = 125502default:defaulth px� 
O
:Phase 1.1.1.5 IOLockPlacementChecker | Checksum: 8e694001
*commonh px� 
�

%s
*constraints2�
�Time (s): cpu = 00:00:00.27 ; elapsed = 00:00:00.11 . Memory (MB): peak = 1872.082 ; gain = 16.008 ; free physical = 933 ; free virtual = 125502default:defaulth px� 
�

Phase %s%s
101*constraints2
	1.1.1.10 2default:default25
!CheckerForMandatoryPrePlacedCells2default:defaultZ18-101h px� 
T
?Phase 1.1.1.4 ClockRegionPlacementChecker | Checksum: 8e694001
*commonh px� 
�

%s
*constraints2�
�Time (s): cpu = 00:00:00.27 ; elapsed = 00:00:00.11 . Memory (MB): peak = 1872.082 ; gain = 16.008 ; free physical = 933 ; free virtual = 125502default:defaulth px� 
�

Phase %s%s
101*constraints2
	1.1.1.11 2default:default24
 CheckerForUnsupportedConstraints2default:defaultZ18-101h px� 
H
3Phase 1.1.1.9 DisallowedInsts | Checksum: 8e694001
*commonh px� 
�

%s
*constraints2�
�Time (s): cpu = 00:00:00.28 ; elapsed = 00:00:00.11 . Memory (MB): peak = 1872.082 ; gain = 16.008 ; free physical = 933 ; free virtual = 125502default:defaulth px� 
}

Phase %s%s
101*constraints2
	1.1.1.12 2default:default2)
Laguna PBlock Checker2default:defaultZ18-101h px� 
[
FPhase 1.1.1.10 CheckerForMandatoryPrePlacedCells | Checksum: 8e694001
*commonh px� 
�

%s
*constraints2�
�Time (s): cpu = 00:00:00.28 ; elapsed = 00:00:00.11 . Memory (MB): peak = 1872.082 ; gain = 16.008 ; free physical = 933 ; free virtual = 125502default:defaulth px� 
�

Phase %s%s
101*constraints2
	1.1.1.13 2default:default24
 CascadeElementConstraintsChecker2default:defaultZ18-101h px� 
Z
EPhase 1.1.1.11 CheckerForUnsupportedConstraints | Checksum: 8e694001
*commonh px� 
�

%s
*constraints2�
�Time (s): cpu = 00:00:00.28 ; elapsed = 00:00:00.11 . Memory (MB): peak = 1872.082 ; gain = 16.008 ; free physical = 933 ; free virtual = 125502default:defaulth px� 
�

Phase %s%s
101*constraints2
	1.1.1.14 2default:default25
!ShapesExcludeCompatibilityChecker2default:defaultZ18-101h px� 
O
:Phase 1.1.1.12 Laguna PBlock Checker | Checksum: 8e694001
*commonh px� 
�

%s
*constraints2�
�Time (s): cpu = 00:00:00.28 ; elapsed = 00:00:00.12 . Memory (MB): peak = 1872.082 ; gain = 16.008 ; free physical = 933 ; free virtual = 125502default:defaulth px� 
�

Phase %s%s
101*constraints2
	1.1.1.15 2default:default21
ShapePlacementValidityChecker2default:defaultZ18-101h px� 
Z
EPhase 1.1.1.13 CascadeElementConstraintsChecker | Checksum: 8e694001
*commonh px� 
�

%s
*constraints2�
�Time (s): cpu = 00:00:00.28 ; elapsed = 00:00:00.12 . Memory (MB): peak = 1872.082 ; gain = 16.008 ; free physical = 933 ; free virtual = 125502default:defaulth px� 
z

Phase %s%s
101*constraints2
	1.1.1.16 2default:default2&
HdioRelatedChecker2default:defaultZ18-101h px� 
[
FPhase 1.1.1.14 ShapesExcludeCompatibilityChecker | Checksum: 8e694001
*commonh px� 
�

%s
*constraints2�
�Time (s): cpu = 00:00:00.28 ; elapsed = 00:00:00.12 . Memory (MB): peak = 1872.082 ; gain = 16.008 ; free physical = 933 ; free virtual = 125502default:defaulth px� 
�

Phase %s%s
101*constraints2
	1.1.1.17 2default:default2-
IOStdCompatabilityChecker2default:defaultZ18-101h px� 
S
>Phase 1.1.1.17 IOStdCompatabilityChecker | Checksum: 8e694001
*commonh px� 
�

%s
*constraints2�
�Time (s): cpu = 00:00:00.29 ; elapsed = 00:00:00.12 . Memory (MB): peak = 1872.082 ; gain = 16.008 ; free physical = 933 ; free virtual = 125502default:defaulth px� 
L
7Phase 1.1.1.16 HdioRelatedChecker | Checksum: 8e694001
*commonh px� 
�

%s
*constraints2�
�Time (s): cpu = 00:00:00.29 ; elapsed = 00:00:00.12 . Memory (MB): peak = 1872.082 ; gain = 16.008 ; free physical = 933 ; free virtual = 125502default:defaulth px� 
W
BPhase 1.1.1.15 ShapePlacementValidityChecker | Checksum: 8e694001
*commonh px� 
�

%s
*constraints2�
�Time (s): cpu = 00:00:00.35 ; elapsed = 00:00:00.14 . Memory (MB): peak = 1872.082 ; gain = 16.008 ; free physical = 933 ; free virtual = 125502default:defaulth px� 
�
�Found overlapping PBlocks. The use of overlapping PBlocks is not recommended as it may lead to legalization errors or unplaced instances.%s708*place2
 2default:defaultZ30-879h px� 
�
bAn IO Bus %s with more than one IO standard is found. Components associated with this bus are: %s
12*place2
SW2default:default2�
�
	<MSGMETA::BEGIN::IO_PORT>SW[15]<MSGMETA::END> of IOStandard LVCMOS33
	<MSGMETA::BEGIN::IO_PORT>SW[14]<MSGMETA::END> of IOStandard LVCMOS33
	<MSGMETA::BEGIN::IO_PORT>SW[13]<MSGMETA::END> of IOStandard LVCMOS33
	<MSGMETA::BEGIN::IO_PORT>SW[12]<MSGMETA::END> of IOStandard LVCMOS33
	<MSGMETA::BEGIN::IO_PORT>SW[11]<MSGMETA::END> of IOStandard LVCMOS33
	<MSGMETA::BEGIN::IO_PORT>SW[10]<MSGMETA::END> of IOStandard LVCMOS33
	<MSGMETA::BEGIN::IO_PORT>SW[9]<MSGMETA::END> of IOStandard LVCMOS18
	<MSGMETA::BEGIN::IO_PORT>SW[8]<MSGMETA::END> of IOStandard LVCMOS18
	<MSGMETA::BEGIN::IO_PORT>SW[7]<MSGMETA::END> of IOStandard LVCMOS33
	<MSGMETA::BEGIN::IO_PORT>SW[6]<MSGMETA::END> of IOStandard LVCMOS33
	<MSGMETA::BEGIN::IO_PORT>SW[5]<MSGMETA::END> of IOStandard LVCMOS33
	<MSGMETA::BEGIN::IO_PORT>SW[4]<MSGMETA::END> of IOStandard LVCMOS33
	<MSGMETA::BEGIN::IO_PORT>SW[3]<MSGMETA::END> of IOStandard LVCMOS33
	<MSGMETA::BEGIN::IO_PORT>SW[2]<MSGMETA::END> of IOStandard LVCMOS33
	<MSGMETA::BEGIN::IO_PORT>SW[1]<MSGMETA::END> of IOStandard LVCMOS33
	<MSGMETA::BEGIN::IO_PORT>SW[0]<MSGMETA::END> of IOStandard LVCMOS33")
SW[15]2
	: of IOStandard LVCMOS33
	"%
SW[14]: of IOStandard LVCMOS33
	"%
SW[13]: of IOStandard LVCMOS33
	"%
SW[12]: of IOStandard LVCMOS33
	"%
SW[11]: of IOStandard LVCMOS33
	"%
SW[10]: of IOStandard LVCMOS33
	"$
SW[9]: of IOStandard LVCMOS18
	"$
SW[8]: of IOStandard LVCMOS18
	"$
SW[7]: of IOStandard LVCMOS33
	"$
SW[6]: of IOStandard LVCMOS33
	"$
SW[5]: of IOStandard LVCMOS33
	"$
SW[4]: of IOStandard LVCMOS33
	"$
SW[3]: of IOStandard LVCMOS33
	"$
SW[2]: of IOStandard LVCMOS33
	"$
SW[1]: of IOStandard LVCMOS33
	""
SW[0]: of IOStandard LVCMOS332default:default8Z30-12h px� 
E
%Done setting XDC timing constraints.
35*timingZ38-35h px� 
{

Phase %s%s
101*constraints2
	1.1.1.18 2default:default2'
IO and Clk Clean Up2default:defaultZ18-101h px� 
�

Phase %s%s
101*constraints2
1.1.1.18.1 2default:default2/
Constructing HAPIClkRuleMgr2default:defaultZ18-101h px� 
W
BPhase 1.1.1.18.1 Constructing HAPIClkRuleMgr | Checksum: 8e694001
*commonh px� 
�

%s
*constraints2�
�Time (s): cpu = 00:00:00.76 ; elapsed = 00:00:00.41 . Memory (MB): peak = 1872.082 ; gain = 16.008 ; free physical = 933 ; free virtual = 125502default:defaulth px� 
M
8Phase 1.1.1.18 IO and Clk Clean Up | Checksum: 8e694001
*commonh px� 
�

%s
*constraints2�
�Time (s): cpu = 00:00:00.76 ; elapsed = 00:00:00.42 . Memory (MB): peak = 1872.082 ; gain = 16.008 ; free physical = 933 ; free virtual = 125502default:defaulth px� 
�

Phase %s%s
101*constraints2
	1.1.1.19 2default:default2>
*Implementation Feasibility check On IDelay2default:defaultZ18-101h px� 
d
OPhase 1.1.1.19 Implementation Feasibility check On IDelay | Checksum: 8e694001
*commonh px� 
�

%s
*constraints2�
�Time (s): cpu = 00:00:00.76 ; elapsed = 00:00:00.42 . Memory (MB): peak = 1872.082 ; gain = 16.008 ; free physical = 933 ; free virtual = 125502default:defaulth px� 
{

Phase %s%s
101*constraints2
	1.1.1.20 2default:default2'
Commit IO Placement2default:defaultZ18-101h px� 
M
8Phase 1.1.1.20 Commit IO Placement | Checksum: 6a36ea1f
*commonh px� 
�

%s
*constraints2�
�Time (s): cpu = 00:00:00.76 ; elapsed = 00:00:00.42 . Memory (MB): peak = 1872.082 ; gain = 16.008 ; free physical = 933 ; free virtual = 125502default:defaulth px� 
U
@Phase 1.1.1 ParallelPlaceIOClockAndInitTop | Checksum: 6a36ea1f
*commonh px� 
�

%s
*constraints2�
�Time (s): cpu = 00:00:00.77 ; elapsed = 00:00:00.42 . Memory (MB): peak = 1872.082 ; gain = 16.008 ; free physical = 933 ; free virtual = 125502default:defaulth px� 
g
RPhase 1.1 IO Placement/ Clock Placement/ Build Placer Device | Checksum: e4c15ca8
*commonh px� 
�

%s
*constraints2�
�Time (s): cpu = 00:00:00.77 ; elapsed = 00:00:00.42 . Memory (MB): peak = 1872.082 ; gain = 16.008 ; free physical = 933 ; free virtual = 125502default:defaulth px� 
}

Phase %s%s
101*constraints2
1.2 2default:default2.
Build Placer Netlist Model2default:defaultZ18-101h px� 
v

Phase %s%s
101*constraints2
1.2.1 2default:default2%
Place Init Design2default:defaultZ18-101h px� 
r

Phase %s%s
101*constraints2
1.2.1.1 2default:default2
Make Others2default:defaultZ18-101h px� 
E
0Phase 1.2.1.1 Make Others | Checksum: 18e65b535
*commonh px� 
�

%s
*constraints2�
�Time (s): cpu = 00:00:00.84 ; elapsed = 00:00:00.46 . Memory (MB): peak = 1872.082 ; gain = 16.008 ; free physical = 933 ; free virtual = 125502default:defaulth px� 
~

Phase %s%s
101*constraints2
1.2.1.2 2default:default2+
Init Lut Pin Assignment2default:defaultZ18-101h px� 
Q
<Phase 1.2.1.2 Init Lut Pin Assignment | Checksum: 18e65b535
*commonh px� 
�

%s
*constraints2�
�Time (s): cpu = 00:00:00.88 ; elapsed = 00:00:00.46 . Memory (MB): peak = 1872.082 ; gain = 16.008 ; free physical = 933 ; free virtual = 125502default:defaulth px� 
I
4Phase 1.2.1 Place Init Design | Checksum: 18222b227
*commonh px� 
�

%s
*constraints2�
�Time (s): cpu = 00:00:01 ; elapsed = 00:00:00.56 . Memory (MB): peak = 1886.734 ; gain = 30.660 ; free physical = 925 ; free virtual = 125422default:defaulth px� 
P
;Phase 1.2 Build Placer Netlist Model | Checksum: 18222b227
*commonh px� 
�

%s
*constraints2�
�Time (s): cpu = 00:00:01 ; elapsed = 00:00:00.56 . Memory (MB): peak = 1886.734 ; gain = 30.660 ; free physical = 925 ; free virtual = 125422default:defaulth px� 
z

Phase %s%s
101*constraints2
1.3 2default:default2+
Constrain Clocks/Macros2default:defaultZ18-101h px� 
M
8Phase 1.3 Constrain Clocks/Macros | Checksum: 18222b227
*commonh px� 
�

%s
*constraints2�
�Time (s): cpu = 00:00:01 ; elapsed = 00:00:00.56 . Memory (MB): peak = 1886.734 ; gain = 30.660 ; free physical = 925 ; free virtual = 125422default:defaulth px� 
I
4Phase 1 Placer Initialization | Checksum: 18222b227
*commonh px� 
�

%s
*constraints2�
�Time (s): cpu = 00:00:01 ; elapsed = 00:00:00.56 . Memory (MB): peak = 1886.734 ; gain = 30.660 ; free physical = 925 ; free virtual = 125422default:defaulth px� 
q

Phase %s%s
101*constraints2
2 2default:default2$
Global Placement2default:defaultZ18-101h px� 
D
/Phase 2 Global Placement | Checksum: 1a9d50191
*commonh px� 
�

%s
*constraints2�
�Time (s): cpu = 00:00:07 ; elapsed = 00:00:01 . Memory (MB): peak = 1942.762 ; gain = 86.688 ; free physical = 914 ; free virtual = 125312default:defaulth px� 
q

Phase %s%s
101*constraints2
3 2default:default2$
Detail Placement2default:defaultZ18-101h px� 
}

Phase %s%s
101*constraints2
3.1 2default:default2.
Commit Multi Column Macros2default:defaultZ18-101h px� 
P
;Phase 3.1 Commit Multi Column Macros | Checksum: 1a9d50191
*commonh px� 
�

%s
*constraints2�
�Time (s): cpu = 00:00:07 ; elapsed = 00:00:01 . Memory (MB): peak = 1942.762 ; gain = 86.688 ; free physical = 914 ; free virtual = 125312default:defaulth px� 


Phase %s%s
101*constraints2
3.2 2default:default20
Commit Most Macros & LUTRAMs2default:defaultZ18-101h px� 
R
=Phase 3.2 Commit Most Macros & LUTRAMs | Checksum: 23804822f
*commonh px� 
�

%s
*constraints2�
�Time (s): cpu = 00:00:07 ; elapsed = 00:00:02 . Memory (MB): peak = 1942.762 ; gain = 86.688 ; free physical = 915 ; free virtual = 125322default:defaulth px� 
y

Phase %s%s
101*constraints2
3.3 2default:default2*
Area Swap Optimization2default:defaultZ18-101h px� 
L
7Phase 3.3 Area Swap Optimization | Checksum: 185e8040c
*commonh px� 
�

%s
*constraints2�
�Time (s): cpu = 00:00:07 ; elapsed = 00:00:02 . Memory (MB): peak = 1942.762 ; gain = 86.688 ; free physical = 915 ; free virtual = 125322default:defaulth px� 
x

Phase %s%s
101*constraints2
3.4 2default:default2)
updateClock Trees: DP2default:defaultZ18-101h px� 
K
6Phase 3.4 updateClock Trees: DP | Checksum: 185e8040c
*commonh px� 
�

%s
*constraints2�
�Time (s): cpu = 00:00:07 ; elapsed = 00:00:02 . Memory (MB): peak = 1942.762 ; gain = 86.688 ; free physical = 915 ; free virtual = 125322default:defaulth px� 
x

Phase %s%s
101*constraints2
3.5 2default:default2)
Timing Path Optimizer2default:defaultZ18-101h px� 
K
6Phase 3.5 Timing Path Optimizer | Checksum: 1d0681acb
*commonh px� 
�

%s
*constraints2�
�Time (s): cpu = 00:00:07 ; elapsed = 00:00:02 . Memory (MB): peak = 1942.762 ; gain = 86.688 ; free physical = 915 ; free virtual = 125322default:defaulth px� 
t

Phase %s%s
101*constraints2
3.6 2default:default2%
Fast Optimization2default:defaultZ18-101h px� 
G
2Phase 3.6 Fast Optimization | Checksum: 1d0681acb
*commonh px� 
�

%s
*constraints2�
�Time (s): cpu = 00:00:07 ; elapsed = 00:00:02 . Memory (MB): peak = 1942.762 ; gain = 86.688 ; free physical = 915 ; free virtual = 125322default:defaulth px� 


Phase %s%s
101*constraints2
3.7 2default:default20
Small Shape Detail Placement2default:defaultZ18-101h px� 
R
=Phase 3.7 Small Shape Detail Placement | Checksum: 221003c60
*commonh px� 
�

%s
*constraints2�
�Time (s): cpu = 00:00:08 ; elapsed = 00:00:02 . Memory (MB): peak = 1942.762 ; gain = 86.688 ; free physical = 911 ; free virtual = 125282default:defaulth px� 
u

Phase %s%s
101*constraints2
3.8 2default:default2&
Re-assign LUT pins2default:defaultZ18-101h px� 
H
3Phase 3.8 Re-assign LUT pins | Checksum: 1cc7d78cb
*commonh px� 
�

%s
*constraints2�
�Time (s): cpu = 00:00:08 ; elapsed = 00:00:02 . Memory (MB): peak = 1942.762 ; gain = 86.688 ; free physical = 911 ; free virtual = 125282default:defaulth px� 
�

Phase %s%s
101*constraints2
3.9 2default:default22
Pipeline Register Optimization2default:defaultZ18-101h px� 
T
?Phase 3.9 Pipeline Register Optimization | Checksum: 1cc7d78cb
*commonh px� 
�

%s
*constraints2�
�Time (s): cpu = 00:00:08 ; elapsed = 00:00:02 . Memory (MB): peak = 1942.762 ; gain = 86.688 ; free physical = 911 ; free virtual = 125282default:defaulth px� 
u

Phase %s%s
101*constraints2
3.10 2default:default2%
Fast Optimization2default:defaultZ18-101h px� 
H
3Phase 3.10 Fast Optimization | Checksum: 1cc7d78cb
*commonh px� 
�

%s
*constraints2�
�Time (s): cpu = 00:00:08 ; elapsed = 00:00:02 . Memory (MB): peak = 1942.762 ; gain = 86.688 ; free physical = 911 ; free virtual = 125282default:defaulth px� 
D
/Phase 3 Detail Placement | Checksum: 1cc7d78cb
*commonh px� 
�

%s
*constraints2�
�Time (s): cpu = 00:00:08 ; elapsed = 00:00:02 . Memory (MB): peak = 1942.762 ; gain = 86.688 ; free physical = 911 ; free virtual = 125282default:defaulth px� 
�

Phase %s%s
101*constraints2
4 2default:default2<
(Post Placement Optimization and Clean-Up2default:defaultZ18-101h px� 
{

Phase %s%s
101*constraints2
4.1 2default:default2,
Post Commit Optimization2default:defaultZ18-101h px� 
E
%Done setting XDC timing constraints.
35*timingZ38-35h px� 
}

Phase %s%s
101*constraints2
4.1.1 2default:default2,
updateClock Trees: PCOPT2default:defaultZ18-101h px� 
P
;Phase 4.1.1 updateClock Trees: PCOPT | Checksum: 1c6ae3fd3
*commonh px� 
�

%s
*constraints2�
�Time (s): cpu = 00:00:08 ; elapsed = 00:00:02 . Memory (MB): peak = 1942.762 ; gain = 86.688 ; free physical = 909 ; free virtual = 125262default:defaulth px� 
�

Phase %s%s
101*constraints2
4.1.2 2default:default2/
Post Placement Optimization2default:defaultZ18-101h px� 
�
hPost Placement Timing Summary WNS=%s. For the most accurate timing information please run report_timing.610*place2
8.8972default:defaultZ30-746h px� 
S
>Phase 4.1.2 Post Placement Optimization | Checksum: 1b73b2f41
*commonh px� 
�

%s
*constraints2�
�Time (s): cpu = 00:00:08 ; elapsed = 00:00:02 . Memory (MB): peak = 1942.762 ; gain = 86.688 ; free physical = 909 ; free virtual = 125262default:defaulth px� 
N
9Phase 4.1 Post Commit Optimization | Checksum: 1b73b2f41
*commonh px� 
�

%s
*constraints2�
�Time (s): cpu = 00:00:08 ; elapsed = 00:00:02 . Memory (MB): peak = 1942.762 ; gain = 86.688 ; free physical = 909 ; free virtual = 125262default:defaulth px� 
�

Phase %s%s
101*constraints2
4.2 2default:default25
!Sweep Clock Roots: Post-Placement2default:defaultZ18-101h px� 
W
BPhase 4.2 Sweep Clock Roots: Post-Placement | Checksum: 1b73b2f41
*commonh px� 
�

%s
*constraints2�
�Time (s): cpu = 00:00:08 ; elapsed = 00:00:02 . Memory (MB): peak = 1942.762 ; gain = 86.688 ; free physical = 909 ; free virtual = 125262default:defaulth px� 
�

Phase %s%s
101*constraints2
4.3 2default:default27
#Uram Pipeline Register Optimization2default:defaultZ18-101h px� 
Y
DPhase 4.3 Uram Pipeline Register Optimization | Checksum: 1b73b2f41
*commonh px� 
�

%s
*constraints2�
�Time (s): cpu = 00:00:08 ; elapsed = 00:00:02 . Memory (MB): peak = 1942.762 ; gain = 86.688 ; free physical = 909 ; free virtual = 125262default:defaulth px� 
y

Phase %s%s
101*constraints2
4.4 2default:default2*
Post Placement Cleanup2default:defaultZ18-101h px� 
L
7Phase 4.4 Post Placement Cleanup | Checksum: 1b73b2f41
*commonh px� 
�

%s
*constraints2�
�Time (s): cpu = 00:00:08 ; elapsed = 00:00:02 . Memory (MB): peak = 1942.762 ; gain = 86.688 ; free physical = 909 ; free virtual = 125262default:defaulth px� 
s

Phase %s%s
101*constraints2
4.5 2default:default2$
Placer Reporting2default:defaultZ18-101h px� 
F
1Phase 4.5 Placer Reporting | Checksum: 1b73b2f41
*commonh px� 
�

%s
*constraints2�
�Time (s): cpu = 00:00:08 ; elapsed = 00:00:02 . Memory (MB): peak = 1942.762 ; gain = 86.688 ; free physical = 909 ; free virtual = 125262default:defaulth px� 
z

Phase %s%s
101*constraints2
4.6 2default:default2+
Final Placement Cleanup2default:defaultZ18-101h px� 
M
8Phase 4.6 Final Placement Cleanup | Checksum: 13dadbf49
*commonh px� 
�

%s
*constraints2�
�Time (s): cpu = 00:00:08 ; elapsed = 00:00:02 . Memory (MB): peak = 1942.762 ; gain = 86.688 ; free physical = 909 ; free virtual = 125262default:defaulth px� 
\
GPhase 4 Post Placement Optimization and Clean-Up | Checksum: 13dadbf49
*commonh px� 
�

%s
*constraints2�
�Time (s): cpu = 00:00:08 ; elapsed = 00:00:02 . Memory (MB): peak = 1942.762 ; gain = 86.688 ; free physical = 909 ; free virtual = 125262default:defaulth px� 
=
(Ending Placer Task | Checksum: b2fef81f
*commonh px� 
�

%s
*constraints2�
�Time (s): cpu = 00:00:08 ; elapsed = 00:00:02 . Memory (MB): peak = 1942.762 ; gain = 86.688 ; free physical = 909 ; free virtual = 125262default:defaulth px� 
Z
Releasing license: %s
83*common2"
Implementation2default:defaultZ17-83h px� 
�
G%s Infos, %s Warnings, %s Critical Warnings and %s Errors encountered.
28*	vivadotcl2
362default:default2
22default:default2
02default:default2
02default:defaultZ4-41h px� 
^
%s completed successfully
29*	vivadotcl2 
place_design2default:defaultZ4-42h px� 
D
Writing placer database...
1603*designutilsZ20-1893h px� 
=
Writing XDEF routing.
211*designutilsZ20-211h px� 
J
#Writing XDEF routing logical nets.
209*designutilsZ20-209h px� 
J
#Writing XDEF routing special nets.
210*designutilsZ20-210h px� 
�
r%sTime (s): cpu = %s ; elapsed = %s . Memory (MB): peak = %s ; gain = %s ; free physical = %s ; free virtual = %s
480*common2)
Write XDEF Complete: 2default:default2
00:00:00.192default:default2
00:00:00.152default:default2
1942.7622default:default2
0.0002default:default2
9062default:default2
125262default:defaultZ17-722h px� 
�
�report_io: Time (s): cpu = 00:00:00.11 ; elapsed = 00:00:00.14 . Memory (MB): peak = 1942.762 ; gain = 0.000 ; free physical = 907 ; free virtual = 12525
*commonh px� 
�
�report_utilization: Time (s): cpu = 00:00:00.08 ; elapsed = 00:00:00.10 . Memory (MB): peak = 1942.762 ; gain = 0.000 ; free physical = 906 ; free virtual = 12523
*commonh px� 
�
�report_control_sets: Time (s): cpu = 00:00:00.04 ; elapsed = 00:00:00.08 . Memory (MB): peak = 1942.762 ; gain = 0.000 ; free physical = 906 ; free virtual = 12523
*commonh px� 


End Record