uint64 timestamp			# time since system start (microseconds)

uint8 BUTTON_SOURCE_UNKNOWN	= 0
uint8 BUTTON_SOURCE_GPIO	= 1
uint8 BUTTON_SOURCE_PX4IO	= 2
uint8 BUTTON_SOURCE_UAVCAN	= 3

uint8 source				# Source of the button
bool switch_available		# Set to true if a switch is connected
bool triggered				# Set to true if the event is triggered

# TOPICS button_event safety_button