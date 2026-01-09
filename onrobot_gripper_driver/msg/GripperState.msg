std_msgs/Header header

# Gripper identification
string gripper_type              # "2FG7", "VG10", "3FG15", "MG10", "RG2" etc.
uint16 product_code              # From register 0x600

# Common status
uint8 status                     # Status bits
uint8 error_code                 # Error code
bool is_busy                     # Motion in progress
bool is_gripping                 # Object detected

# Position/Width/Diameter (context-dependent)
float64 position_mm              # 2FG/RG: width, 3FG: diameter, MG: not used
float64 target_position_mm       

# Force/Vacuum/Strength (context-dependent)
float64 force_n                  # 2FG/RG/3FG: force, VG: vacuum%, MG: strength%

# Type-specific data (only populated for relevant gripper types)
# VG grippers
float64[] channel_vacuum_pct     # VG: vacuum % per channel
bool[] channel_gripping          # VG: grip status per channel

# 3FG grippers  
float64 diameter_raw_mm          # 3FG: raw diameter
