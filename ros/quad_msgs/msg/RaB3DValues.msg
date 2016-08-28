#Number of receivers
int8 rn

#Binary variable for each receiver that specifies if it is active and with valid values
int8[] ON

#Number of emitters sent in the message
int8 emitn

#Emitter slots in the receiver queue
int16[] slot

#Detected emitters
int8[] detected

#Emitter codes
int16[] code

#Receiver Signal Strength that each receiver measures from each emitter in the world. All measurements of a single receiver are contiguous in memory
int16[] values
