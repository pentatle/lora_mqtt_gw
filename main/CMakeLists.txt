set(component_srcs "app_main.c")

idf_component_register(SRCS "${component_srcs}"
                       INCLUDE_DIRS ".")
