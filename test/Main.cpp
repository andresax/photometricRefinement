#include <iostream>
#include <PhotometricRefinement.h>

int main(int argc, char **argv) {

  if(argc == 2){
    PhotometricRefinement pr(argv[1]);
    pr.run();
  }else{
    std::cout<<"ERROR: Main --> configuration file path is missing"<<std::endl;
  }
  return 0;
}

