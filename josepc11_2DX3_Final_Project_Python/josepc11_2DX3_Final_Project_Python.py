import serial
import numpy as np
import open3d as o3d
import math


#josepc11 2DX3 Final Project

#Christy Joseph-Anton
#josepc11
#400325365

#April 8, 2022

#STEP 1: receive measurements from MCU

#opens serial port for UART COM4, 
s=serial.Serial('COM4',115200, timeout=10)

print("Opening Port: " + s.name)

# reset the buffers of the UART port to delete the remaining data in the buffers
s.reset_output_buffer()
s.reset_input_buffer()

# wait for user's signal to start the program
input("Press Enter to start communication...")
# send the character 'b' to MCU via UART to signal that python code is ready for data transmission
s.write('b'.encode())

#2D array to hold distance measurements for 32 measurements per rotation, 10 displacements
measurements=[[0]*32 for _ in range(10)]

#tells user sensor is booting up
print("Booting Sensor...")
# recieve start up sequence from UART of MCU
for i in range(20):
    x = s.readline()
    #if sensor boot success, exits the loop and is ready to receieve measurements
    if x.decode()=='SensorInit Successful.\r\n':
        print("Sensor Booted!")
        break

#runs 10 times for each x displacement
for x in range(10):
    
    # recieve 32 measurements from UART of MCU
    i=0;
    while i<32:
        distance = s.readline()
        #only writes UART line from MCU if line is not empty
        if(distance.decode()!=''):
            measurements[x][i]=int(distance.decode())
            print(str(i+1)+". "+distance.decode())
            i=i+1;

#tells user when each x value is complete
    print("x = "+str(x*20)+" cm completed. Move to next x value")



#close the port
print("Closing: " + s.name)
s.close()



#STEP 2: calculate x y z coordinates and write to file

#calculate y z values and store back in measurements array
#angle is 11.25 deg 


if __name__ == "__main__":

    f = open("josepc11_2DX3_FinalProject_data.xyz", "w")    #create a new file for writing 
    
    #writes increments of 20cm(200mm) to x value
    #writes sin of incremented angle multiplied by distance to y value
    #writes cos of incremented angle multiplied by distance to z value
    for x in range(10):
        angle=0;
        for i in range(32):
            f.write(str(x*200)+' '+str(measurements[x][i]*math.cos(angle*math.pi/180))+' '+str(measurements[x][i]*math.sin(angle*math.pi/180))+'\n')    #write x,0,0 (xyz) to file as p
            angle=angle+11.25

    f.close()   #there should now be a file containing 320 vertex coordinates




#step 3: make 3D model using external file

        #Read the test data in from the file created        
    print("Read in the prism point cloud data (pcd)")
    pcd = o3d.io.read_point_cloud("josepc11_2DX3_FinalProject_data.xyz", format="xyz")

    #Lets see what our point cloud data looks like numerically       
    print("The PCD array:")
    print(np.asarray(pcd.points))

    #Give each vertex a unique number
    yz_slice_vertex = []
    for x in range(0,320):
        yz_slice_vertex.append([x])

    #Define coordinates to connect lines in each yz slice        
    lines = []  
    for x in range(0,320,32):
        lines.append([yz_slice_vertex[x], yz_slice_vertex[x+1]])
        lines.append([yz_slice_vertex[x+1], yz_slice_vertex[x+2]])
        lines.append([yz_slice_vertex[x+2], yz_slice_vertex[x+3]])
        lines.append([yz_slice_vertex[x+3], yz_slice_vertex[x+4]])
        lines.append([yz_slice_vertex[x+4], yz_slice_vertex[x+5]])
        lines.append([yz_slice_vertex[x+5], yz_slice_vertex[x+6]])
        lines.append([yz_slice_vertex[x+6], yz_slice_vertex[x+7]])
        lines.append([yz_slice_vertex[x+7], yz_slice_vertex[x+8]])
        lines.append([yz_slice_vertex[x+8], yz_slice_vertex[x+9]])
        lines.append([yz_slice_vertex[x+9], yz_slice_vertex[x+10]])
        lines.append([yz_slice_vertex[x+10], yz_slice_vertex[x+11]])
        lines.append([yz_slice_vertex[x+11], yz_slice_vertex[x+12]])
        lines.append([yz_slice_vertex[x+12], yz_slice_vertex[x+13]])
        lines.append([yz_slice_vertex[x+13], yz_slice_vertex[x+14]])
        lines.append([yz_slice_vertex[x+14], yz_slice_vertex[x+15]])
        lines.append([yz_slice_vertex[x+15], yz_slice_vertex[x+16]])
        lines.append([yz_slice_vertex[x+16], yz_slice_vertex[x+17]])
        lines.append([yz_slice_vertex[x+17], yz_slice_vertex[x+18]])
        lines.append([yz_slice_vertex[x+18], yz_slice_vertex[x+19]])
        lines.append([yz_slice_vertex[x+19], yz_slice_vertex[x+20]])
        lines.append([yz_slice_vertex[x+20], yz_slice_vertex[x+21]])
        lines.append([yz_slice_vertex[x+21], yz_slice_vertex[x+22]])
        lines.append([yz_slice_vertex[x+22], yz_slice_vertex[x+23]])
        lines.append([yz_slice_vertex[x+23], yz_slice_vertex[x+24]])
        lines.append([yz_slice_vertex[x+24], yz_slice_vertex[x+25]])
        lines.append([yz_slice_vertex[x+25], yz_slice_vertex[x+26]])
        lines.append([yz_slice_vertex[x+26], yz_slice_vertex[x+27]])
        lines.append([yz_slice_vertex[x+27], yz_slice_vertex[x+28]])
        lines.append([yz_slice_vertex[x+28], yz_slice_vertex[x+29]])
        lines.append([yz_slice_vertex[x+29], yz_slice_vertex[x+30]])
        lines.append([yz_slice_vertex[x+30], yz_slice_vertex[x+31]])
        lines.append([yz_slice_vertex[x+31], yz_slice_vertex[x]])
        


    #Define coordinates to connect lines between current and next yz slice        
    for x in range(0,287,32):
        lines.append([yz_slice_vertex[x], yz_slice_vertex[x+32]])
        lines.append([yz_slice_vertex[x+1], yz_slice_vertex[x+33]])
        lines.append([yz_slice_vertex[x+2], yz_slice_vertex[x+34]])
        lines.append([yz_slice_vertex[x+3], yz_slice_vertex[x+35]])
        lines.append([yz_slice_vertex[x+4], yz_slice_vertex[x+36]])
        lines.append([yz_slice_vertex[x+5], yz_slice_vertex[x+37]])
        lines.append([yz_slice_vertex[x+6], yz_slice_vertex[x+38]])
        lines.append([yz_slice_vertex[x+7], yz_slice_vertex[x+39]])
        lines.append([yz_slice_vertex[x+8], yz_slice_vertex[x+40]])
        lines.append([yz_slice_vertex[x+9], yz_slice_vertex[x+41]])
        lines.append([yz_slice_vertex[x+10], yz_slice_vertex[x+42]])
        lines.append([yz_slice_vertex[x+11], yz_slice_vertex[x+43]])
        lines.append([yz_slice_vertex[x+12], yz_slice_vertex[x+44]])
        lines.append([yz_slice_vertex[x+13], yz_slice_vertex[x+45]])
        lines.append([yz_slice_vertex[x+14], yz_slice_vertex[x+46]])
        lines.append([yz_slice_vertex[x+15], yz_slice_vertex[x+47]])
        lines.append([yz_slice_vertex[x+16], yz_slice_vertex[x+48]])
        lines.append([yz_slice_vertex[x+17], yz_slice_vertex[x+49]])
        lines.append([yz_slice_vertex[x+18], yz_slice_vertex[x+50]])
        lines.append([yz_slice_vertex[x+19], yz_slice_vertex[x+51]])
        lines.append([yz_slice_vertex[x+20], yz_slice_vertex[x+52]])
        lines.append([yz_slice_vertex[x+21], yz_slice_vertex[x+53]])
        lines.append([yz_slice_vertex[x+22], yz_slice_vertex[x+54]])
        lines.append([yz_slice_vertex[x+23], yz_slice_vertex[x+55]])
        lines.append([yz_slice_vertex[x+24], yz_slice_vertex[x+56]])
        lines.append([yz_slice_vertex[x+25], yz_slice_vertex[x+57]])
        lines.append([yz_slice_vertex[x+26], yz_slice_vertex[x+58]])
        lines.append([yz_slice_vertex[x+27], yz_slice_vertex[x+59]])
        lines.append([yz_slice_vertex[x+28], yz_slice_vertex[x+60]])
        lines.append([yz_slice_vertex[x+29], yz_slice_vertex[x+61]])
        lines.append([yz_slice_vertex[x+30], yz_slice_vertex[x+62]])
        lines.append([yz_slice_vertex[x+31], yz_slice_vertex[x+63]])
        

    #This line maps the lines to the 3d coordinate vertices
    line_set = o3d.geometry.LineSet(points=o3d.utility.Vector3dVector(np.asarray(pcd.points)),lines=o3d.utility.Vector2iVector(lines))

    #Lets see what our point cloud data with lines looks like graphically       
    o3d.visualization.draw_geometries([line_set])
