# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.10

# Delete rule output on recipe failure.
.DELETE_ON_ERROR:


#=============================================================================
# Special targets provided by cmake.

# Disable implicit rules so canonical targets will work.
.SUFFIXES:


# Remove some rules from gmake that .SUFFIXES does not remove.
SUFFIXES =

.SUFFIXES: .hpux_make_needs_suffix_list


# Suppress display of executed commands.
$(VERBOSE).SILENT:


# A target that is always out of date.
cmake_force:

.PHONY : cmake_force

#=============================================================================
# Set environment variables for the build.

# The shell in which to execute make rules.
SHELL = /bin/sh

# The CMake executable.
CMAKE_COMMAND = /usr/bin/cmake

# The command to remove a file.
RM = /usr/bin/cmake -E remove -f

# Escaping for special characters.
EQUALS = =

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /home/shuoge/Git/fobot_tools/aaa/SCServo_Linux

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/shuoge/Git/fobot_tools/aaa/SCServo_Linux/build

# Include any dependencies generated for this target.
include CMakeFiles/Super_GoHome.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/Super_GoHome.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/Super_GoHome.dir/flags.make

CMakeFiles/Super_GoHome.dir/examples/SUPER/GoHome/main.cc.o: CMakeFiles/Super_GoHome.dir/flags.make
CMakeFiles/Super_GoHome.dir/examples/SUPER/GoHome/main.cc.o: ../examples/SUPER/GoHome/main.cc
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/shuoge/Git/fobot_tools/aaa/SCServo_Linux/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/Super_GoHome.dir/examples/SUPER/GoHome/main.cc.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/Super_GoHome.dir/examples/SUPER/GoHome/main.cc.o -c /home/shuoge/Git/fobot_tools/aaa/SCServo_Linux/examples/SUPER/GoHome/main.cc

CMakeFiles/Super_GoHome.dir/examples/SUPER/GoHome/main.cc.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/Super_GoHome.dir/examples/SUPER/GoHome/main.cc.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/shuoge/Git/fobot_tools/aaa/SCServo_Linux/examples/SUPER/GoHome/main.cc > CMakeFiles/Super_GoHome.dir/examples/SUPER/GoHome/main.cc.i

CMakeFiles/Super_GoHome.dir/examples/SUPER/GoHome/main.cc.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/Super_GoHome.dir/examples/SUPER/GoHome/main.cc.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/shuoge/Git/fobot_tools/aaa/SCServo_Linux/examples/SUPER/GoHome/main.cc -o CMakeFiles/Super_GoHome.dir/examples/SUPER/GoHome/main.cc.s

CMakeFiles/Super_GoHome.dir/examples/SUPER/GoHome/main.cc.o.requires:

.PHONY : CMakeFiles/Super_GoHome.dir/examples/SUPER/GoHome/main.cc.o.requires

CMakeFiles/Super_GoHome.dir/examples/SUPER/GoHome/main.cc.o.provides: CMakeFiles/Super_GoHome.dir/examples/SUPER/GoHome/main.cc.o.requires
	$(MAKE) -f CMakeFiles/Super_GoHome.dir/build.make CMakeFiles/Super_GoHome.dir/examples/SUPER/GoHome/main.cc.o.provides.build
.PHONY : CMakeFiles/Super_GoHome.dir/examples/SUPER/GoHome/main.cc.o.provides

CMakeFiles/Super_GoHome.dir/examples/SUPER/GoHome/main.cc.o.provides.build: CMakeFiles/Super_GoHome.dir/examples/SUPER/GoHome/main.cc.o


CMakeFiles/Super_GoHome.dir/SCS.cpp.o: CMakeFiles/Super_GoHome.dir/flags.make
CMakeFiles/Super_GoHome.dir/SCS.cpp.o: ../SCS.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/shuoge/Git/fobot_tools/aaa/SCServo_Linux/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Building CXX object CMakeFiles/Super_GoHome.dir/SCS.cpp.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/Super_GoHome.dir/SCS.cpp.o -c /home/shuoge/Git/fobot_tools/aaa/SCServo_Linux/SCS.cpp

CMakeFiles/Super_GoHome.dir/SCS.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/Super_GoHome.dir/SCS.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/shuoge/Git/fobot_tools/aaa/SCServo_Linux/SCS.cpp > CMakeFiles/Super_GoHome.dir/SCS.cpp.i

CMakeFiles/Super_GoHome.dir/SCS.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/Super_GoHome.dir/SCS.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/shuoge/Git/fobot_tools/aaa/SCServo_Linux/SCS.cpp -o CMakeFiles/Super_GoHome.dir/SCS.cpp.s

CMakeFiles/Super_GoHome.dir/SCS.cpp.o.requires:

.PHONY : CMakeFiles/Super_GoHome.dir/SCS.cpp.o.requires

CMakeFiles/Super_GoHome.dir/SCS.cpp.o.provides: CMakeFiles/Super_GoHome.dir/SCS.cpp.o.requires
	$(MAKE) -f CMakeFiles/Super_GoHome.dir/build.make CMakeFiles/Super_GoHome.dir/SCS.cpp.o.provides.build
.PHONY : CMakeFiles/Super_GoHome.dir/SCS.cpp.o.provides

CMakeFiles/Super_GoHome.dir/SCS.cpp.o.provides.build: CMakeFiles/Super_GoHome.dir/SCS.cpp.o


CMakeFiles/Super_GoHome.dir/SCSCL.cpp.o: CMakeFiles/Super_GoHome.dir/flags.make
CMakeFiles/Super_GoHome.dir/SCSCL.cpp.o: ../SCSCL.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/shuoge/Git/fobot_tools/aaa/SCServo_Linux/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_3) "Building CXX object CMakeFiles/Super_GoHome.dir/SCSCL.cpp.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/Super_GoHome.dir/SCSCL.cpp.o -c /home/shuoge/Git/fobot_tools/aaa/SCServo_Linux/SCSCL.cpp

CMakeFiles/Super_GoHome.dir/SCSCL.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/Super_GoHome.dir/SCSCL.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/shuoge/Git/fobot_tools/aaa/SCServo_Linux/SCSCL.cpp > CMakeFiles/Super_GoHome.dir/SCSCL.cpp.i

CMakeFiles/Super_GoHome.dir/SCSCL.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/Super_GoHome.dir/SCSCL.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/shuoge/Git/fobot_tools/aaa/SCServo_Linux/SCSCL.cpp -o CMakeFiles/Super_GoHome.dir/SCSCL.cpp.s

CMakeFiles/Super_GoHome.dir/SCSCL.cpp.o.requires:

.PHONY : CMakeFiles/Super_GoHome.dir/SCSCL.cpp.o.requires

CMakeFiles/Super_GoHome.dir/SCSCL.cpp.o.provides: CMakeFiles/Super_GoHome.dir/SCSCL.cpp.o.requires
	$(MAKE) -f CMakeFiles/Super_GoHome.dir/build.make CMakeFiles/Super_GoHome.dir/SCSCL.cpp.o.provides.build
.PHONY : CMakeFiles/Super_GoHome.dir/SCSCL.cpp.o.provides

CMakeFiles/Super_GoHome.dir/SCSCL.cpp.o.provides.build: CMakeFiles/Super_GoHome.dir/SCSCL.cpp.o


CMakeFiles/Super_GoHome.dir/SCSerial.cpp.o: CMakeFiles/Super_GoHome.dir/flags.make
CMakeFiles/Super_GoHome.dir/SCSerial.cpp.o: ../SCSerial.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/shuoge/Git/fobot_tools/aaa/SCServo_Linux/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_4) "Building CXX object CMakeFiles/Super_GoHome.dir/SCSerial.cpp.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/Super_GoHome.dir/SCSerial.cpp.o -c /home/shuoge/Git/fobot_tools/aaa/SCServo_Linux/SCSerial.cpp

CMakeFiles/Super_GoHome.dir/SCSerial.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/Super_GoHome.dir/SCSerial.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/shuoge/Git/fobot_tools/aaa/SCServo_Linux/SCSerial.cpp > CMakeFiles/Super_GoHome.dir/SCSerial.cpp.i

CMakeFiles/Super_GoHome.dir/SCSerial.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/Super_GoHome.dir/SCSerial.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/shuoge/Git/fobot_tools/aaa/SCServo_Linux/SCSerial.cpp -o CMakeFiles/Super_GoHome.dir/SCSerial.cpp.s

CMakeFiles/Super_GoHome.dir/SCSerial.cpp.o.requires:

.PHONY : CMakeFiles/Super_GoHome.dir/SCSerial.cpp.o.requires

CMakeFiles/Super_GoHome.dir/SCSerial.cpp.o.provides: CMakeFiles/Super_GoHome.dir/SCSerial.cpp.o.requires
	$(MAKE) -f CMakeFiles/Super_GoHome.dir/build.make CMakeFiles/Super_GoHome.dir/SCSerial.cpp.o.provides.build
.PHONY : CMakeFiles/Super_GoHome.dir/SCSerial.cpp.o.provides

CMakeFiles/Super_GoHome.dir/SCSerial.cpp.o.provides.build: CMakeFiles/Super_GoHome.dir/SCSerial.cpp.o


CMakeFiles/Super_GoHome.dir/SMSBL.cpp.o: CMakeFiles/Super_GoHome.dir/flags.make
CMakeFiles/Super_GoHome.dir/SMSBL.cpp.o: ../SMSBL.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/shuoge/Git/fobot_tools/aaa/SCServo_Linux/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_5) "Building CXX object CMakeFiles/Super_GoHome.dir/SMSBL.cpp.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/Super_GoHome.dir/SMSBL.cpp.o -c /home/shuoge/Git/fobot_tools/aaa/SCServo_Linux/SMSBL.cpp

CMakeFiles/Super_GoHome.dir/SMSBL.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/Super_GoHome.dir/SMSBL.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/shuoge/Git/fobot_tools/aaa/SCServo_Linux/SMSBL.cpp > CMakeFiles/Super_GoHome.dir/SMSBL.cpp.i

CMakeFiles/Super_GoHome.dir/SMSBL.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/Super_GoHome.dir/SMSBL.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/shuoge/Git/fobot_tools/aaa/SCServo_Linux/SMSBL.cpp -o CMakeFiles/Super_GoHome.dir/SMSBL.cpp.s

CMakeFiles/Super_GoHome.dir/SMSBL.cpp.o.requires:

.PHONY : CMakeFiles/Super_GoHome.dir/SMSBL.cpp.o.requires

CMakeFiles/Super_GoHome.dir/SMSBL.cpp.o.provides: CMakeFiles/Super_GoHome.dir/SMSBL.cpp.o.requires
	$(MAKE) -f CMakeFiles/Super_GoHome.dir/build.make CMakeFiles/Super_GoHome.dir/SMSBL.cpp.o.provides.build
.PHONY : CMakeFiles/Super_GoHome.dir/SMSBL.cpp.o.provides

CMakeFiles/Super_GoHome.dir/SMSBL.cpp.o.provides.build: CMakeFiles/Super_GoHome.dir/SMSBL.cpp.o


CMakeFiles/Super_GoHome.dir/SMSCL.cpp.o: CMakeFiles/Super_GoHome.dir/flags.make
CMakeFiles/Super_GoHome.dir/SMSCL.cpp.o: ../SMSCL.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/shuoge/Git/fobot_tools/aaa/SCServo_Linux/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_6) "Building CXX object CMakeFiles/Super_GoHome.dir/SMSCL.cpp.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/Super_GoHome.dir/SMSCL.cpp.o -c /home/shuoge/Git/fobot_tools/aaa/SCServo_Linux/SMSCL.cpp

CMakeFiles/Super_GoHome.dir/SMSCL.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/Super_GoHome.dir/SMSCL.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/shuoge/Git/fobot_tools/aaa/SCServo_Linux/SMSCL.cpp > CMakeFiles/Super_GoHome.dir/SMSCL.cpp.i

CMakeFiles/Super_GoHome.dir/SMSCL.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/Super_GoHome.dir/SMSCL.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/shuoge/Git/fobot_tools/aaa/SCServo_Linux/SMSCL.cpp -o CMakeFiles/Super_GoHome.dir/SMSCL.cpp.s

CMakeFiles/Super_GoHome.dir/SMSCL.cpp.o.requires:

.PHONY : CMakeFiles/Super_GoHome.dir/SMSCL.cpp.o.requires

CMakeFiles/Super_GoHome.dir/SMSCL.cpp.o.provides: CMakeFiles/Super_GoHome.dir/SMSCL.cpp.o.requires
	$(MAKE) -f CMakeFiles/Super_GoHome.dir/build.make CMakeFiles/Super_GoHome.dir/SMSCL.cpp.o.provides.build
.PHONY : CMakeFiles/Super_GoHome.dir/SMSCL.cpp.o.provides

CMakeFiles/Super_GoHome.dir/SMSCL.cpp.o.provides.build: CMakeFiles/Super_GoHome.dir/SMSCL.cpp.o


# Object files for target Super_GoHome
Super_GoHome_OBJECTS = \
"CMakeFiles/Super_GoHome.dir/examples/SUPER/GoHome/main.cc.o" \
"CMakeFiles/Super_GoHome.dir/SCS.cpp.o" \
"CMakeFiles/Super_GoHome.dir/SCSCL.cpp.o" \
"CMakeFiles/Super_GoHome.dir/SCSerial.cpp.o" \
"CMakeFiles/Super_GoHome.dir/SMSBL.cpp.o" \
"CMakeFiles/Super_GoHome.dir/SMSCL.cpp.o"

# External object files for target Super_GoHome
Super_GoHome_EXTERNAL_OBJECTS =

Super_GoHome: CMakeFiles/Super_GoHome.dir/examples/SUPER/GoHome/main.cc.o
Super_GoHome: CMakeFiles/Super_GoHome.dir/SCS.cpp.o
Super_GoHome: CMakeFiles/Super_GoHome.dir/SCSCL.cpp.o
Super_GoHome: CMakeFiles/Super_GoHome.dir/SCSerial.cpp.o
Super_GoHome: CMakeFiles/Super_GoHome.dir/SMSBL.cpp.o
Super_GoHome: CMakeFiles/Super_GoHome.dir/SMSCL.cpp.o
Super_GoHome: CMakeFiles/Super_GoHome.dir/build.make
Super_GoHome: CMakeFiles/Super_GoHome.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/shuoge/Git/fobot_tools/aaa/SCServo_Linux/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_7) "Linking CXX executable Super_GoHome"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/Super_GoHome.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/Super_GoHome.dir/build: Super_GoHome

.PHONY : CMakeFiles/Super_GoHome.dir/build

CMakeFiles/Super_GoHome.dir/requires: CMakeFiles/Super_GoHome.dir/examples/SUPER/GoHome/main.cc.o.requires
CMakeFiles/Super_GoHome.dir/requires: CMakeFiles/Super_GoHome.dir/SCS.cpp.o.requires
CMakeFiles/Super_GoHome.dir/requires: CMakeFiles/Super_GoHome.dir/SCSCL.cpp.o.requires
CMakeFiles/Super_GoHome.dir/requires: CMakeFiles/Super_GoHome.dir/SCSerial.cpp.o.requires
CMakeFiles/Super_GoHome.dir/requires: CMakeFiles/Super_GoHome.dir/SMSBL.cpp.o.requires
CMakeFiles/Super_GoHome.dir/requires: CMakeFiles/Super_GoHome.dir/SMSCL.cpp.o.requires

.PHONY : CMakeFiles/Super_GoHome.dir/requires

CMakeFiles/Super_GoHome.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/Super_GoHome.dir/cmake_clean.cmake
.PHONY : CMakeFiles/Super_GoHome.dir/clean

CMakeFiles/Super_GoHome.dir/depend:
	cd /home/shuoge/Git/fobot_tools/aaa/SCServo_Linux/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/shuoge/Git/fobot_tools/aaa/SCServo_Linux /home/shuoge/Git/fobot_tools/aaa/SCServo_Linux /home/shuoge/Git/fobot_tools/aaa/SCServo_Linux/build /home/shuoge/Git/fobot_tools/aaa/SCServo_Linux/build /home/shuoge/Git/fobot_tools/aaa/SCServo_Linux/build/CMakeFiles/Super_GoHome.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/Super_GoHome.dir/depend

