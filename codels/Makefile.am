lib_LTLIBRARIES = librmp440_codels.la

librmp440_codels_la_SOURCES=	rmp440_c_types.h
librmp440_codels_la_SOURCES+=	rmp440_codels.cc
librmp440_codels_la_SOURCES+=	rmp440_MotionTask_codels.cc
librmp440_codels_la_SOURCES+=	rmp440_TrackTask_codels.cc

librmp440_codels_la_SOURCES+=	codels.h joystick.cc motion_helpers.cc \
				odoProba.cc odoProba.h orMathLib.h

librmp440_codels_la_SOURCES+=	rmp440_Log.cc rmp440_Log.h

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
