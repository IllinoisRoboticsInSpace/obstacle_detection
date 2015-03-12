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

VERSIONS: libfreenect-dev

#====PROBLEM 1====#
#====PROBLEM 1====#
#====PROBLEM 1====#
If you are having undefined reference to "function" errors, that means the program can't find the functions in the .so file. Your .so file may be out of date, or the program can't find the .so file when linking.


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




#====PROBLEM 2====#
#====PROBLEM 2====#
#====PROBLEM 2====#
lib/libfreenect.so: could not read symbols: File in wrong format
Another problem is when the .so file is not compatible with the local machine (symbols in libfreenect.so). To fix this, you need to aquire a new libfreenect.so filefrom the software center(libfreenect-dev or freenect), or build your own from source. Either way, once you can find the new libfreenect.so file, try using it. If the version changes significantly, you will get undefined refernces to functions or variables, because the headers point to functions, variables, and defines that no longer are supported by the new lib you just built, so you will need to get the new headers (libfreenect.h and .hpp) associated with that new library. They should come with the lib whether you got it from the software center, or build from source. Then change any functions or variables in the main programs to their new names, or arguments.


#====NOTES====#
#====NOTES====#
#====NOTES====#
The Makefile could be configured so that object files get put in an obj directory to clean things up, but this would make the Makefile more complicated, so it's not really worth it.







