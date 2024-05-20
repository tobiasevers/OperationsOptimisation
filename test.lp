\ Model UAVstrike
\ LP format - for model browsing. Use MPS format to capture full model detail.
Minimize
  0.1 C7 + 0.1 C8 + 0.1 C9 + C10
Subject To
 R0: C1 = 1
 R1: C3 = 1
 R2: C0 + C2 = 1
 R3: C1 <= 1
 R4: C3 <= 1
 R5: C0 + C2 <= 1
 R6: C1 + C2 + C3 <= 1
 R7: C4 + C5 <= 1
 R8: C4 <= 1
 R9: C0 + C2 <= 1
 R10: C2 + C3 <= 1
 R11: C3 - C4 <= 0
 R12: - C0 + C1 - C4 <= 0
 R13: C0 + C2 + C4 <= 1
 R14: - C1 - C2 - C3 + C4 <= 0
 R15: C1 + C2 + C3 + C5 = 1
 R16: C0 - C1 <= 0
 R17: 100 C1 - C6 + C7 <= 101
 R18: - 100 C1 - C6 + C7 >= -99
 R19: 100 C2 - C6 + C8 <= 103
 R20: - 100 C2 - C6 + C8 >= -97
 R21: 100 C3 - C6 + C9 <= 107
 R22: - 100 C3 - C6 + C9 >= -93
 R23: C7 - C8 <= -1
 R24: C8 - C9 <= -1
 R25: C1 + 3 C2 + 7 C3 <= 100
Bounds
Binaries
 C0 C1 C2 C3 C4 C5
End
