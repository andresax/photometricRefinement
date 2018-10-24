#include <iostream>

#include <cppunit/TestSuite.h>
#include <cppunit/TestResult.h>
#include <cppunit/extensions/HelperMacros.h>
#include <cppunit/ui/text/TestRunner.h>

//#include <CORE/Expr.h>

#include <CGAL/basic.h>
#include <CGAL/IO/Verbose_ostream.h>


// Use verr.set_verbose(true) to enable this.
CGAL::Verbose_ostream verr;

// For boxdef.h
int next_box_id = 1;


int main( int argc, char* argv[] )
{
    CGAL::set_pretty_mode( std::cout );

    CppUnit::TextUi::TestRunner runner;
    CppUnit::TestFactoryRegistry& registry 
	= CppUnit::TestFactoryRegistry::getRegistry();

    runner.addTest( registry.makeTest() );
    runner.run();

    return 0;  //FIXME: status from runner?
}
