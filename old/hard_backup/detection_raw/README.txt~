#leonfrickensmith@gmail.com

#If any of these steps fail, see HELP.
#To build the project, you need gcc installed and working.
#cd into the same directory as this text file and the Makefile, enter:
make

#This should build the project.
#To run the project, enter:
./obs_det




#====HELP====#
#====HELP====#
#====HELP====#

#In order to make this code compile and run the program will look for libfreenect.so.0.1 and libfreenect.so
#Things not working on the first try is expected, as the makefile is configured to expect the following:
#libfreenect.so		IN THE DIRECTORY (Makefile_directory)/lib
#libfreenect.so		IN THE DIRECTORY (LD_LIBRARY_PATH)
#libfreenect.so.0.1	IN THE DIRECTORY (LD_LIBRARY_PATH)

#NOTE: libfreenect.s0.0.1 is either a renamed copy of, or a link to a libfreenect.so file.
#So for the ones that are expected in (LD_LIBRARY_PATH) you can either add the libraries to a path already pointed to by LD_LIBRARY_PATH, or add a new path to the set of paths, and put the libraries in there.

#I recommend you do the following, but you can customize these steps as long as you understand thier purpose. Read over them first!
#Start in a folder with libfreenect.so in it.

1.
#Copy the libfreenect.so file to /usr/local/lib:
sudo cp libfreenect.so /usr/local/lib/libfreenect.so

2.
#Create a link in /usr/local/lib to point to the libfreenect.so file in that directory:
cd /usr/local/lib
sudo ln -s libfreenect.so libfreenect.so.0.1

3.
#Open the .bashrc file, this file launches everytime you open a terminal:
gedit ~/.bashrc

4.
#Add /usr/local/lib to the LD_LIBRARY_PATH permanently by adding this line to the bottom of the file, and save it. DO NOT EDIT ANYTHING ELSE
export LD_LIBRARY_PATH="$LD_LIBRARY_PATH:/usr/local/lib"

5.
#Restart your command line for these changes to take effect.
#Now anytime you attempt to launch a program needing those libraries through the command line, they should be found!













