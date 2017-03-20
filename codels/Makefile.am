lib_LTLIBRARIES = librmp440_codels.la

librmp440_codels_la_SOURCES=	rmp440_c_types.h
librmp440_codels_la_SOURCES+=	rmp440_codels.c
librmp440_codels_la_SOURCES+=	rmp440_MotionTask_codels.c
librmp440_codels_la_SOURCES+=	rmp440_TrackTask_codels.c

librmp440_codels_la_SOURCES+=	codels.h joystick.c motion_helpers.c \
				odoProba.c odoProba.h orMathLib.h track.c

librmp440_codels_la_SOURCES+=	rmp440_Log.c rmp440_Log.h

librmp440_codels_la_CPPFLAGS=	$(requires_CFLAGS)
librmp440_codels_la_LDFLAGS=	$(requires_LIBS)
librmp440_codels_la_LDFLAGS+=	-release $(PACKAGE_VERSION)


# idl  mappings
BUILT_SOURCES=	rmp440_c_types.h
CLEANFILES=	${BUILT_SOURCES}
rmp440_c_types.h: ${top_srcdir}/rmp440.gen
	${GENOM3}  mappings \
	  -MD -MF .deps/$@.d -MT $@ --signature -l c $< >$@

-include .deps/rmp440_c_types.h.d
