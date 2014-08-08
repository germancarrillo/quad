#include <iostream>
#include <fstream>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <stdint.h>

using namespace std;

int main () {
  ofstream myfile;
  myfile.open ("example.txt");
  cout<<"writing 1"<<endl;
  myfile << "1 Writing this to a file.\n";  myfile.seekp(0);
  cout<<"sleeping"<<endl;
  usleep(10000000);
  cout<<"writing 2"<<endl;
  myfile << "2 Writing this to a file.\n"; myfile.seekp(0);
  usleep(10000000);
  myfile.close();
  return 0;
}
