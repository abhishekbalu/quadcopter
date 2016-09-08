from Tkinter import *

def show_values():
    print (w1.get(), w2.get())

def print2file(val):
	file = open("pipe.txt", "w")
	file.write("%d\n%d\n %f\n%f\n%f\n%f\n %f\n %f\n%f\n%f\n %d\n%d\n " % 
		(fx.get(), fy.get(), kx1.get(), ky1.get(), kx2.get(), ky2.get(), 
			theta_calib.get(), szx.get(), szy.get(), szLED.get(), thsz.get(), thd.get(), dt_min.get()))
	file.close()


master = Tk()
fx = Scale(master, from_=450, to=480, command=print2file)
fx.set(19)
fx.pack(side=RIGHT)
fy = Scale(master, from_=450, to=480, command=print2file)
fy.set(23)
fy.pack(side=RIGHT)

kx1 = Scale(master, from_=0.0000010, to=0.0000100, command=print2file, resolution=0.0000001)
kx1.set(0.0000055)
kx1.pack(side=RIGHT)
kx2 = Scale(master, from_=1e-10, to=3e-10, command=print2file, resolution=1e-11)
kx2.set(2e-10)
kx2.pack(side=RIGHT)
ky1 = Scale(master, from_=0.0000001, to=0.0000100, resolution=0.0000001, command=print2file)
ky1.set(0.0000020)
ky1.pack(side=RIGHT)
ky2 = Scale(master, from_=0.0, to=1e-10, resolution=1e-11, command=print2file)
ky2.set(0.0)
ky2.pack(side=RIGHT)

theta_calib = Scale(master, from_=0.00, to=3.1415, resolution=0.1, command=print2file)
theta_calib.set(0.00)
theta_calib.pack(side=RIGHT)

szx = Scale(master, from_=0.2, to=1.0, resolution=0.1, command=print2file)
szx.set(0.4)
szx.pack(side=RIGHT)
szy = Scale(master, from_=0.2, to=1.0, resolution=0.1, command=print2file)
szy.set(0.4)
szy.pack(side=RIGHT)
szLED = Scale(master, from_=0.0, to=0.2, resolution=0.01, command=print2file)
szLED.set(0.08)
szLED.pack(side=RIGHT)

thsz = Scale(master, from_=0, to=10, resolution=1, command=print2file)
thsz.set(1)
thsz.pack(side=RIGHT)
thd = Scale(master, from_=100, to=1000, resolution=50, command=print2file)
thd.set(400)
thd.pack(side=RIGHT)
dt_min = Scale(master, from_=0.0, to=0.5, resolution=0.1, command=print2file)
dt_min.set(0.1)
dt_min.pack(side=RIGHT)
Button(master, text='Show', command=show_values).pack()



mainloop()