# Progess status message: continuously published by the SpeechSynthesis Service 
# to let the motion module know when speech will be done.

# -- Special header message types: http://wiki.ros.org/Messages

# Two-integer timestamp that is expressed as:
# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')
# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')
# time-handling sugar is provided by the client library
#time stamp

# -- Actual Payload
string audio # the sentence currently being synthesized
uint8 progress 	# how much of the sentence is spoken so far: 0 < progress < 100