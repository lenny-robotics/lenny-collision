if(${LENNY_PROJECT}_BUILD_APPS)
	add_subdirectory(apps)
endif()
if(NOT TARGET lenny::collision)
	add_subdirectory(collision)
endif()
