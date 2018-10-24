CGAL_LDFLAGS =
include $(CGAL_MAKEFILE)

# The GCC FAQ miscellaneous section (http://gcc.gnu.org/faq.html#misc)
# claims that -Wreturn-type is memory-hungry when compiling template-heavy
# code.  That warning is turned on by -Wall (set in CGAL_CXXFLAGS), so
# we turn it off explicitly using -Wno-return-type.  
# Sadly, it doesn't seem to make much difference in terms of compile time.
# (also, it is not porable to non-GCC)

AM_CXXFLAGS = $(CGAL_CXXFLAGS) -DUSE_GMPZ
INCLUDES = -I$(top_srcdir) $(CPPUNIT_CFLAGS)
LDADD = $(CGAL_LDFLAGS) $(CPPUNIT_LIBS)


if HAVE_CPPUNIT
TESTS = $(check_PROGRAMS)
check_PROGRAMS = cppunit_test cppunit_test2 cppunit_test_PR
endif

cppunit_test_SOURCES = tester.cc tester.h	\
	boxdef.h				\
	Segment_3_Point_3.cc               	\
	Segment_3_Segment_3.cc			\
	Segment_3_Triangle_3.cc            	\
	Triangle_3_Triangle_3.cc           	\
	Triangle_3_Triangle_3_degenerate.cc 	\
	Triangle_3_Plane_3.cc           	\
	Triangle_3_Point_3.cc           	\
	Line_3_Line_3.cc 			\
	Halfplane_3.cc 				\
	Plane_3_Triangle_3.cc           	\
	Iso_cuboid_3_Iso_cuboid_3.cc       	\
	Box_d_Box_d.cc                     	\
	Box_d_Box_d_batch.cc               	\
	Polygon_2_Line_2.cc                	\
	Object.cc

cppunit_test2_SOURCES = tester.cc tester.h 	\
	boxdef.h				\
	Box_d_Box_d_batch2.cc			

cppunit_test_PR_SOURCES = tester.cc tester.h PR_tests.cc
