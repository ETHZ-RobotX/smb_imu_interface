# This is a super-minimized message to hold data from a mangetometer
#
# values are raw data and have to be scaled!
#
# To minimize the amount of data transfered by this message (e.g. form an Arduino through rosserial)
# the amount of data is truncated to its absolute minimum.

std_msgs/Time time

int16 magx
int16 magy
int16 magz
