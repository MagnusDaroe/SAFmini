	cmake_minimum_required(VERSION 3.5)
	project(saf)

	# Default to C++14
	if(NOT CMAKE_CXX_STANDARD)
	  set(CMAKE_CXX_STANDARD 14)
	endif()

	if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
	  add_compile_options(-Wall -Wextra -Wpedantic)
	endif()

	# Find dependencies
	find_package(ament_cmake REQUIRED)
	find_package(ament_cmake_python REQUIRED)
	find_package(rclcpp REQUIRED)
	find_package(rclpy REQUIRED)
	
	find_package(builtin_interfaces REQUIRED)
	find_package(rosidl_default_generators REQUIRED)
	if(NOT builtin_interfaces_FOUND)
    		message(FATAL_ERROR "Could not find builtin_interfaces package")
	endif()

	#find_package(drone REQUIRED)  

	rosidl_generate_interfaces(${PROJECT_NAME}
	  "msg/Processtime.msg"
	  "srv/ProcessTimeService.srv"
	  DEPENDENCIES builtin_interfaces)
	
	# Include Cpp "include" directory
	include_directories(include)

	# Create Cpp executable
	add_executable(cpp_executable src/process_time.cpp) # change name of node
	ament_target_dependencies(processtime rclcpp)
	# Install Cpp executables
	install(TARGETS
	  cpp_executable
	  DESTINATION lib/${PROJECT_NAME}
	)
	# Install Python modules
	ament_python_install_package(${PROJECT_NAME})
	# Install Python executables
	install(PROGRAMS
	  scripts/server_node.py
	  scripts/processtime_node.py
	  DESTINATION lib/${PROJECT_NAME} # change name of node in python
	)

	# Install launch files.
	install(DIRECTORY
	launch
	DESTINATION share/${PROJECT_NAME}/
	)
	ament_package()
