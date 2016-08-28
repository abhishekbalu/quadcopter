#Number of receivers
int8 rn

#Receivers positions - (x,y,z) coordinates of each receiver are contiguous in memory
float64[] x

#Receiver headings - (x,y,z) coordinates of the headings are contiguous in memory
#(do we need to convert the positions back to a normal (x,y,z) reference frame?)
float64[] r

#Absorption coefficient model parameters - they are contiguous in memory for each receiver
float64[] f

#Number of branches of the amplification stage model (depends of each receiver)
int8[] bn

#The transitions that define each branch of the amplification stage model
float64[] s_it #inputs
float64[] s_ot #outputs

#Amplication stage model parameters - they are contiguous in memeory for each branch (going from s_i^0 to s_i^2). The branches are in ascending order
#All parameters for each receiver are contiguous in memeory
float64[] g #model
float64[] dg #derivatives
