# Navigation Satellite fix for HuaXin MEMS driver GHFPD

Header header

# euler angles
# heading: 0 - 360
#   pitch: -90 - 90
#    roll: -180 - 180
float64 heading
float64 pitch
float64 roll

# LLT degees
float64 latitude
float64 longitude
float64 altitude

# speed, m/s
# vn: north speed
# ve: east speed
# vd: ground speed
float64 vn
float64 ve
float64 vd

# base line length, m
float64 baseline

# number of satellite
uint8 nav1
uint8 nav2

int8 status
int8 ERROR = -1
int8 PREPARE = 0
int8 ALIGNMENT = 1
int8 ONLY_INS = 2
int8 INS_DMI = 5
int8 INS_GNSS = 7
int8 INS_GNSS_DMI = 8