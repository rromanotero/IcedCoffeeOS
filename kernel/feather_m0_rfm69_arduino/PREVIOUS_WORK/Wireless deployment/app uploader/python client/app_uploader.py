from tkinter import Tk, Label, Button
from tkinter.filedialog import askopenfilename
import serial, sys

class AppUploader:

	serial_port = "COM4"

	def __init__(self, master):
		self.master = master
		master.title("Rupert App Uploader")
		
		self.label = Label(master, text="Upload a python script (yes just one!) and run it")
		self.label.pack()
		
		self.greet_button = Button(master, text="Upload", command=self.upload)
		self.greet_button.pack()
		
		self.close_button = Button(master, text="Close", command=master.quit)
		self.close_button.pack()
		
	def upload(self):
		#get filename
		print(  "Reading filename from user" )
		filename = askopenfilename()
		
		#read file
		print(  "Reading file {}".format(filename) )
		file_contents = ""
		with open( filename, "r" ) as f:
			file_contents = f.read()
		
		if file_contents == "" :
			print( "Empty File" )
			return
		
		#turn file contents into binary
		#binary_contents = bytearray( file_contents, 'utf-8' )
		
		#send to base station
		print(  "Looking for base station in {}".format( self.serial_port ) )
		with serial.Serial( self.serial_port, 9600, timeout=1 ) as ser:
			
			# -- Transmit --
			print( "Base station found. Sending file contents" )
			
			#Signal start of transmission
			ser.write( b'+' )   
			
			#send away
			for c in file_contents:
				ser.write( bytes( c, 'utf-8' ) ) 
				
				
			#Signal end of transmission
			ser.write( b'-' )   
		
			#Print whatever the base station replies
			while True:
				raw = ser.read(1)
				c = ""
				try:
					c = raw.decode( "utf-8" )
				except:
					c = "[NON UTF-8 CHAR]"
					
				
				if c == '-':
					#Base sation is done
					break 
				else:	
					#print
					sys.stdout.write( "{}".format( c  ) )
					sys.stdout.flush()
					
		print( "Base station connection closed\n" ) 
						
root = Tk()
my_gui = AppUploader(root)
root.mainloop()