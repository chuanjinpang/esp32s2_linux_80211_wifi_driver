set(srcs "main.c" "link_glue.c" "usb_link.c"
	
	)

idf_component_register(SRCS ${srcs}
					INCLUDE_DIRS ".")
