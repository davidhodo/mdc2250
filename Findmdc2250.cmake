find_path(mdc2250_INCLUDE_DIRS mdc2250.h /usr/include "$ENV{NAMER_ROOT}")

find_library(mdc2250_LIBRARIES mdc2250 /usr/lib "$ENV{NAMER_ROOT}")

set(mdc2250_FOUND TRUE)

if (NOT mdc2250_INCLUDE_DIRS)
    set(mdc2250_FOUND FALSE)
endif (NOT mdc2250_INCLUDE_DIRS)

if (NOT mdc2250_LIBRARIES)
    set(mdc2250_FOUND FALSE)
endif (NOT mdc2250_LIBRARIES)
